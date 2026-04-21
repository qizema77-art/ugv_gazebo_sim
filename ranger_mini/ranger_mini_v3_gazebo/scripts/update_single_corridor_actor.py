#!/usr/bin/env python3
"""
更新 single_corridor.world 中的动态行人 actor。

设计目标：
- 只需要指定 speed（m/s）就自动计算 waypoint 时间。
- 起点/终点/y/z/转身时间可选覆盖，不传则使用默认值。

默认参数：
- start_x=12.0, end_x=4.0, y=0.0, z=0.0
- turn_time=0.5
- heading_forward=0.0, heading_backward=3.14159

用法示例：
python3 scripts/update_single_corridor_actor.py --speed 0.8
"""

from __future__ import annotations

import argparse
from pathlib import Path


AUTO_BEGIN = "<!-- AUTO_ACTOR_BEGIN -->"
AUTO_END = "<!-- AUTO_ACTOR_END -->"


def fmt(v: float) -> str:
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s else "0"


def build_actor_block(
    start_x: float,
    end_x: float,
    y: float,
    z: float,
    speed: float,
    turn_time: float,
    heading_forward: float,
    heading_backward: float,
) -> str:
    if speed <= 0:
        raise ValueError("speed 必须大于 0")

    travel_time = abs(start_x - end_x) / speed
    t0 = 0.0
    t1 = travel_time
    t2 = t1 + turn_time
    t3 = t2 + travel_time
    t4 = t3 + turn_time

    return f"""{AUTO_BEGIN}
    <actor name=\"walking_dynamic_obstacle_single_corridor\">
      <pose>{fmt(start_x)} {fmt(y)} {fmt(z)} 0 0 0</pose>
      <skin>
        <filename>walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name=\"walking\">
        <filename>walk.dae</filename>
        <scale>1.0</scale>
        <interpolate_x>true</interpolate_x>
      </animation>
      <link name=\"actor_link\">
        <pose>0 0 1.0 0 0 0</pose>
        <collision name=\"Box_Collision\">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>1.8</length>
            </cylinder>
          </geometry>
        </collision>
      </link>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id=\"0\" type=\"walking\">
          <waypoint>
            <time>{fmt(t0)}</time>
            <pose>{fmt(start_x)} {fmt(y)} {fmt(z)} 0 0 {fmt(heading_backward)}</pose>
          </waypoint>
          <waypoint>
            <time>{fmt(t1)}</time>
            <pose>{fmt(end_x)} {fmt(y)} {fmt(z)} 0 0 {fmt(heading_backward)}</pose>
          </waypoint>
          <waypoint>
            <time>{fmt(t2)}</time>
            <pose>{fmt(end_x)} {fmt(y)} {fmt(z)} 0 0 {fmt(heading_forward)}</pose>
          </waypoint>
          <waypoint>
            <time>{fmt(t3)}</time>
            <pose>{fmt(start_x)} {fmt(y)} {fmt(z)} 0 0 {fmt(heading_forward)}</pose>
          </waypoint>
          <waypoint>
            <time>{fmt(t4)}</time>
            <pose>{fmt(start_x)} {fmt(y)} {fmt(z)} 0 0 {fmt(heading_backward)}</pose>
          </waypoint>
        </trajectory>
      </script>
      <plugin name=\"actor_collisions_plugin\" filename=\"libActorCollisionsPlugin.so\">
        <scaling collision=\"Box_Collision\">
          <x>1.0</x>
          <y>1.0</y>
          <z>1.0</z>
        </scaling>
      </plugin>
    </actor>
    <!-- AUTO_ACTOR_END -->"""


def replace_actor_block(world_text: str, actor_block: str) -> str:
    start = world_text.find(AUTO_BEGIN)
    end = world_text.find(AUTO_END)

    if start != -1 and end != -1 and end > start:
        end = end + len(AUTO_END)
        return world_text[:start] + actor_block + world_text[end:]

    insert_pos = world_text.rfind("</world>")
    if insert_pos == -1:
        raise RuntimeError("world 文件中未找到 </world> 标签")

    return world_text[:insert_pos] + "\n    " + actor_block + "\n" + world_text[insert_pos:]


def main() -> None:
    parser = argparse.ArgumentParser(description="按速度自动计算时间并更新 single_corridor actor")
    parser.add_argument("--speed", type=float, required=True, help="行人速度 (m/s)")
    parser.add_argument("--start-x", type=float, default=12.0, help="起点 x")
    parser.add_argument("--end-x", type=float, default=4.0, help="终点 x")
    parser.add_argument("--y", type=float, default=0.0, help="行走 y")
    parser.add_argument("--z", type=float, default=0.0, help="行走 z")
    parser.add_argument("--turn-time", type=float, default=0.5, help="端点转身停留时间 (s)")
    parser.add_argument("--heading-forward", type=float, default=0.0, help="回程朝向 yaw")
    parser.add_argument("--heading-backward", type=float, default=3.14159, help="去程朝向 yaw")
    parser.add_argument(
        "--world",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "worlds" / "single_corridor.world",
        help="world 文件路径",
    )

    args = parser.parse_args()

    actor_block = build_actor_block(
        start_x=args.start_x,
        end_x=args.end_x,
        y=args.y,
        z=args.z,
        speed=args.speed,
        turn_time=args.turn_time,
        heading_forward=args.heading_forward,
        heading_backward=args.heading_backward,
    )

    world_path: Path = args.world
    text = world_path.read_text(encoding="utf-8")
    new_text = replace_actor_block(text, actor_block)
    world_path.write_text(new_text, encoding="utf-8")

    travel_time = abs(args.start_x - args.end_x) / args.speed
    cycle_time = 2 * travel_time + 2 * args.turn_time
    print(
        f"[OK] updated: {world_path}\n"
        f"speed={args.speed} m/s, start_x={args.start_x}, end_x={args.end_x}, y={args.y}, z={args.z}\n"
        f"travel_time={travel_time:.3f} s, cycle_time={cycle_time:.3f} s"
    )


if __name__ == "__main__":
    main()
