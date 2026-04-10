#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""ROS topic 快速检查脚本：输出 topic 的含义(内置少量规则) / type / pub-sub / 数据结构位置 / 发布行为 / 带宽(粗略)。

用法示例：
  1) 终端先确保有 ROS master 和节点在跑，然后：
     source /root/ocs2_ws/devel/setup.bash
    python3 topic_inspect.py --md > topics.md

  2) 指定只看部分 topic：
     python3 topic_inspect.py --topics /tf /tf_static

说明：
    - “含义”很难从系统自动推断；脚本仅对常见 topic 做了少量内置解释，其余留空。
    - “发布行为/带宽”是基于：AnyMsg 订阅采样(消息数 + 原始序列化字节数) 的经验估计。
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple


def _run(cmd: Sequence[str], timeout_s: float = 2.0) -> Tuple[int, str, str]:
    try:
        p = subprocess.run(
            list(cmd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
            check=False,
            text=True,
        )
        return p.returncode, p.stdout.strip(), p.stderr.strip()
    except subprocess.TimeoutExpired:
        return 124, "", "timeout"


def _require_cmd(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(f"找不到命令: {name}。请先 source ROS 环境，并确保已安装相关工具。")


def _rostopic_list() -> List[str]:
    code, out, _ = _run(["rostopic", "list"], timeout_s=3.0)
    if code != 0 or not out:
        return []
    return sorted([line.strip() for line in out.splitlines() if line.strip()])


def _rostopic_type(topic: str) -> str:
    code, out, _ = _run(["rostopic", "type", topic], timeout_s=2.0)
    if code != 0:
        return ""
    return out.splitlines()[-1].strip()


def _rostopic_info(topic: str) -> str:
    code, out, _ = _run(["rostopic", "info", topic], timeout_s=2.0)
    if code != 0:
        return ""
    return out


def _parse_rostopic_info_nodes(info_text: str) -> Tuple[List[str], List[str], Optional[bool]]:
    pubs: List[str] = []
    subs: List[str] = []
    latched: Optional[bool] = None

    section = None
    for line in info_text.splitlines():
        if line.startswith("Publishers"):
            section = "pub"
            continue
        if line.startswith("Subscribers"):
            section = "sub"
            continue
        m_latch = re.search(r"Latched:\s*(True|False)", line, re.IGNORECASE)
        if m_latch:
            latched = m_latch.group(1).lower() == "true"

        line = line.strip()
        if not line.startswith("*"):
            continue

        # Example: * /node_name (http://...)
        node = line.lstrip("* ").split(" ")[0].strip()
        if not node:
            continue
        # Keep output stable: ignore this script's anonymous node
        if node.startswith("/topic_inspect"):
            continue

        if section == "pub":
            pubs.append(node)
        elif section == "sub":
            subs.append(node)

    return pubs, subs, latched


def _rospack_find(pkg: str) -> str:
    code, out, _ = _run(["rospack", "find", pkg], timeout_s=2.0)
    if code != 0:
        return ""
    return out.strip().splitlines()[-1].strip()


def _guess_msg_file(msg_type: str) -> Tuple[str, str]:
    """Return (structure_hint, command_hint)."""
    if not msg_type or "/" not in msg_type:
        return "", ""
    pkg, name = msg_type.split("/", 1)
    pkg_path = _rospack_find(pkg)
    if pkg_path:
        candidate = os.path.join(pkg_path, "msg", f"{name}.msg")
        if os.path.exists(candidate):
            return candidate, f"rosmsg show {msg_type}"
        # Some messages are generated or in srv/action; still provide command.
        return os.path.join(pkg_path, "msg", f"{name}.msg"), f"rosmsg show {msg_type}"
    return "", f"rosmsg show {msg_type}"


def _rosmsg_show(msg_type: str) -> str:
    if not msg_type:
        return ""
    code, out, _ = _run(["rosmsg", "show", msg_type], timeout_s=3.0)
    if code != 0:
        return ""
    return out


def _compress_fields(rosmsg_text: str, limit: int = 220) -> str:
    """把 rosmsg show 的多行字段结构压成单行，适合放进 Markdown 表格。"""
    if not rosmsg_text:
        return ""
    # drop constants lines like: uint8 XXX=0
    lines = []
    for line in rosmsg_text.splitlines():
        s = line.strip()
        if not s:
            continue
        if re.match(r"^(u?int|float|string|bool|time|duration)\d*\s+\w+\s*=", s):
            continue
        lines.append(s)
    one = "; ".join(lines)
    one = re.sub(r"\s+", " ", one).strip()
    if len(one) > limit:
        return one[: max(0, limit - 1)] + "…"
    return one


@dataclass
class TopicRow:
    topic: str
    meaning: str
    msg_type: str
    pubs: List[str]
    subs: List[str]
    struct_hint: str
    struct_cmd: str
    fields: str
    mode: str
    avg_bytes: Optional[float]
    bandwidth_bps: Optional[float]


_MEANING_HINTS: Dict[str, str] = {
    "/mobile_manipulator_mpc_observation": "MPC 观测(当前 time/state/input/mode)",
    "/mobile_manipulator_mpc_target": "MPC 目标轨迹(TargetTrajectories)",
    "/mobile_manipulator_mpc_policy": "MPC 求解结果(策略/轨迹/性能指标)",
    "/mobile_manipulator_mode_schedule": "(可选) mode schedule 输入", 
    "/mobile_manipulator/optimizedPoseTrajectory": "优化轨迹可视化(PoseArray)",
    "/mobile_manipulator/optimizedStateTrajectory": "优化状态可视化(MarkerArray)",
    "/distance_markers": "距离/约束等可视化(MarkerArray)",
    "/simple_marker/update": "InteractiveMarker 增量更新(server->RViz)",
    "/simple_marker/update_full": "InteractiveMarker 全量初始化(server->client)",
    "/simple_marker/feedback": "InteractiveMarker 交互反馈(RViz->server)",
    "/tf": "动态 TF", 
    "/tf_static": "静态 TF", 
    "/joint_states": "关节状态(通常由驱动/仿真发布)",
    "/rosout": "ROS 日志", 
    "/rosout_agg": "ROS 日志聚合", 
    "/clicked_point": "RViz 工具: Publish Point", 
    "/initialpose": "RViz 工具: 2D Pose Estimate", 
    "/move_base_simple/goal": "RViz 工具: 2D Nav Goal",
}


def _meaning_for(topic: str) -> str:
    return _MEANING_HINTS.get(topic, "")


def _sample_rate_anymsg(topic: str, duration_s: float) -> Tuple[int, float, Optional[float]]:
    """Subscribe with rospy.AnyMsg and count msgs for duration_s.

    Returns:
      - count: number of received messages
      - hz: count / elapsed
      - avg_bytes: average serialized bytes per message (if available)
    """
    try:
        import rospy
        from rospy.msg import AnyMsg
    except Exception:
        return 0, 0.0, None

    count = 0
    total_bytes = 0

    def cb(_msg: AnyMsg) -> None:
        nonlocal count
        nonlocal total_bytes
        count += 1
        raw = getattr(_msg, "_buff", None)
        if raw is None:
            return
        try:
            total_bytes += len(raw)
        except Exception:
            pass

    # Avoid re-init errors if script is reused in same interpreter.
    if not rospy.core.is_initialized():
        rospy.init_node("topic_inspect", anonymous=True, disable_signals=True)

    sub = rospy.Subscriber(topic, AnyMsg, cb, queue_size=100)
    start = time.time()
    # rospy.sleep will process callbacks.
    try:
        rospy.sleep(max(0.05, duration_s))
    except Exception:
        pass
    sub.unregister()
    elapsed = max(1e-6, time.time() - start)
    hz = float(count) / elapsed
    avg_bytes = (float(total_bytes) / count) if (count > 0 and total_bytes > 0) else None
    return count, hz, avg_bytes


def _classify_mode(
    topic: str,
    pubs: List[str],
    latched: Optional[bool],
    sample_duration_s: float,
) -> Tuple[str, int, float, Optional[float], Optional[float]]:
    if not pubs:
        return "无发布者", 0, 0.0, None, None

    if latched is True:
        # still sample once in case latched isn't correctly exposed
        count, hz, avg_bytes = _sample_rate_anymsg(topic, sample_duration_s)
        bps = (hz * avg_bytes) if (avg_bytes is not None) else None
        return "latched(静态/一次性)", count, hz, avg_bytes, bps

    # If rostopic info didn't say latched, fall back to sampling.
    count, hz, avg_bytes = _sample_rate_anymsg(topic, sample_duration_s)
    bps = (hz * avg_bytes) if (avg_bytes is not None) else None
    if count <= 0:
        return f"触发/空闲(采样{sample_duration_s:.1f}s无数据)", 0, 0.0, None, None

    if hz >= 0.5:
        return f"持续(~{hz:.1f} Hz)", count, hz, avg_bytes, bps

    return f"低频/触发(~{hz:.2f} Hz)", count, hz, avg_bytes, bps


def _format_pubsub(pubs: List[str], subs: List[str]) -> str:
    pubs_s = ",".join(pubs) if pubs else "-"
    subs_s = ",".join(subs) if subs else "-"
    return f"pub:{pubs_s}; sub:{subs_s}"


def _to_md_table(rows: List[TopicRow]) -> str:
    # Extended single-line table.
    lines = []
    lines.append("| topic | 语义 | type | 字段结构(压缩) | .msg位置/命令 | 谁发谁收 | 发布行为 | 平均大小 | 带宽估计 |")
    lines.append("|---|---|---|---|---|---|---|---:|---:|")
    for r in rows:
        struct = r.struct_hint if r.struct_hint else r.struct_cmd
        who = _format_pubsub(r.pubs, r.subs)
        fields = r.fields if r.fields else "-"
        meaning = r.meaning if r.meaning else "-"
        avg = (f"{r.avg_bytes:.0f} B" if isinstance(r.avg_bytes, (int, float)) else "-")
        bw = (f"{r.bandwidth_bps/1024.0:.1f} KiB/s" if isinstance(r.bandwidth_bps, (int, float)) else "-")
        lines.append(
            f"| {r.topic} | {meaning} | {r.msg_type or '-'} | {fields} | {struct} | {who} | {r.mode} | {avg} | {bw} |"
        )
    return "\n".join(lines)


_TABLE_BEGIN = "<!-- TOPIC_TABLE:BEGIN -->"
_TABLE_END = "<!-- TOPIC_TABLE:END -->"


def _update_md_file(path: str, table_md: str) -> None:
    """Replace content between markers with table_md; if absent, append."""
    try:
        with open(path, "r", encoding="utf-8") as f:
            content = f.read()
    except FileNotFoundError:
        content = ""

    block = f"{_TABLE_BEGIN}\n\n{table_md}\n\n{_TABLE_END}"
    if _TABLE_BEGIN in content and _TABLE_END in content:
        pre = content.split(_TABLE_BEGIN, 1)[0]
        post = content.split(_TABLE_END, 1)[1]
        new_content = pre + block + post
    else:
        if content and not content.endswith("\n"):
            content += "\n"
        new_content = content + "\n" + block + "\n"

    with open(path, "w", encoding="utf-8") as f:
        f.write(new_content)


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("--topics", nargs="*", default=None, help="只检查指定 topics；不填则 rostopic list")
    parser.add_argument("--duration", type=float, default=0.5, help="采样时长(秒)，用于粗略判断持续/触发")
    parser.add_argument("--md", action="store_true", help="输出 Markdown 单行表格")
    parser.add_argument("--update-md", default="", help="把生成的表格写回到指定 md 文件(用标记段覆盖)")
    args = parser.parse_args(argv)

    for c in ("rostopic", "rospack", "rosmsg"):
        _require_cmd(c)

    topics = args.topics if args.topics else _rostopic_list()
    if not topics:
        print("未获取到任何 topic。确认 roscore 和节点已启动，且已 source 对应 setup.bash。", file=sys.stderr)
        return 2

    rows: List[TopicRow] = []
    for t in topics:
        meaning = _meaning_for(t)
        msg_type = _rostopic_type(t)
        info = _rostopic_info(t)
        pubs, subs, latched = _parse_rostopic_info_nodes(info) if info else ([], [], None)
        struct_hint, struct_cmd = _guess_msg_file(msg_type)
        rosmsg_text = _rosmsg_show(msg_type)
        fields = _compress_fields(rosmsg_text)
        mode, _count, _hz, avg_bytes, bandwidth_bps = _classify_mode(t, pubs, latched, args.duration)
        rows.append(
            TopicRow(
                topic=t,
                meaning=meaning,
                msg_type=msg_type,
                pubs=pubs,
                subs=subs,
                struct_hint=struct_hint,
                struct_cmd=struct_cmd,
                fields=fields,
                mode=mode,
                avg_bytes=avg_bytes,
                bandwidth_bps=bandwidth_bps,
            )
        )

    table_md = _to_md_table(rows)
    if args.update_md:
        _update_md_file(args.update_md, table_md)

    if args.md:
        print(table_md)
    else:
        for r in rows:
            print(f"== {r.topic} ==")
            if r.meaning:
                print(f"meaning: {r.meaning}")
            print(f"type: {r.msg_type or '-'}")
            print(f"who:  {_format_pubsub(r.pubs, r.subs)}")
            if r.struct_hint:
                print(f"struct: {r.struct_hint}")
            print(f"cmd:   {r.struct_cmd}")
            if r.fields:
                print(f"fields: {r.fields}")
            print(f"mode:  {r.mode}")
            if r.avg_bytes is not None:
                print(f"avg_bytes: {r.avg_bytes:.0f}")
            if r.bandwidth_bps is not None:
                print(f"bandwidth: {r.bandwidth_bps/1024.0:.1f} KiB/s")
            print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
