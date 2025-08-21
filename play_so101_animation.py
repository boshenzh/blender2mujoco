#!/usr/bin/env python3
"""
Play a Blender-exported keyframe animation (so101_animation.json) on the SO101 MuJoCo model.

- Starts from the first keyframe.
- Drives each joint with a MuJoCo position actuator so qpos follows the JSON targets.
- Time is respected (uses each frame's "time" to step the sim for the right duration).

Tested with mujoco>=3.1 and mujoco-python.  If you're on mujoco-py, adapt viewer bits.
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np

import mujoco
from mujoco import viewer


@dataclass
class AnimSpec:
    fps: float
    sample_hz: float
    duration_sec: float
    joint_order: List[str]
    units_angles_deg: bool
    frames: List[Dict[str, float]]  # each has "time" plus per-joint angles


def load_animation(json_path: str) -> AnimSpec:
    with open(json_path, "r") as f:
        raw = json.load(f)

    units_angles_deg = False
    if "units" in raw and "angles" in raw["units"]:
        units_angles_deg = str(raw["units"]["angles"]).lower().startswith("deg")

    frames: List[Dict[str, float]] = raw["frames"]
    # Ensure frames are sorted by time
    frames = sorted(frames, key=lambda d: float(d["time"]))

    return AnimSpec(
        fps=float(raw.get("fps", 0.0)),
        sample_hz=float(raw.get("sample_hz", 0.0)),
        duration_sec=float(raw.get("duration_sec", frames[-1]["time"] if frames else 0.0)),
        joint_order=list(raw["joint_order"]),
        units_angles_deg=units_angles_deg,
        frames=frames,
    )


def deg2rad_if_needed(x: float, units_angles_deg: bool) -> float:
    return math.radians(x) if units_angles_deg else x


def build_actuator_map(model: mujoco.MjModel, joint_names: Sequence[str]) -> Dict[str, int]:
    """
    Map each required joint name -> actuator id.
    We assume each joint in joint_names has a position actuator with the same *name*
    in the MJCF (as in your so101_new_calib.xml).

    Falls back to trying to find an actuator that drives the joint if names differ.
    """
    name_to_id: Dict[str, int] = {}

    # First try direct actuator name lookups.
    for jn in joint_names:
        try:
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jn)
            if aid != -1:
                name_to_id[jn] = aid
                continue
        except Exception:
            pass

        # Fallback: find actuator that targets this joint
        try:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        except Exception as e:
            raise RuntimeError(f"Joint '{jn}' not found in model: {e}")

        # Find first actuator whose trnid points to this joint id
        found = False
        for aid in range(model.nu):
            # For position actuators, trnid[aid,0] = joint id (when acting on a joint)
            if model.actuator_trnid[aid, 0] == jid:
                name_to_id[jn] = aid
                found = True
                break
        if not found:
            raise RuntimeError(f"No actuator found for joint '{jn}'. "
                               "Ensure your XML defines a 'position' actuator for it.")
    return name_to_id


def set_qpos_from_frame(model: mujoco.MjModel,
                        data: mujoco.MjData,
                        joint_order: Sequence[str],
                        frame: Dict[str, float],
                        units_angles_deg: bool):
    """Set data.qpos (only for the named joints) to the frame's target angles."""
    # Build joint name -> qpos address map once
    # Cache on the function object
    if not hasattr(set_qpos_from_frame, "_cache") or set_qpos_from_frame._cache_model is not model:
        qpos_addr = {}
        for jn in joint_order:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid == -1:
                raise RuntimeError(f"Joint '{jn}' not found in model.")
            adr = model.jnt_qposadr[jid]
            qpos_addr[jn] = int(adr)
        set_qpos_from_frame._cache = qpos_addr
        set_qpos_from_frame._cache_model = model
    else:
        qpos_addr = set_qpos_from_frame._cache

    for jn in joint_order:
        target = float(frame[jn])
        val_rad = deg2rad_if_needed(target, units_angles_deg)
        data.qpos[qpos_addr[jn]] = val_rad

    mujoco.mj_forward(model, data)


def clip_to_ctrlrange(model: mujoco.MjModel, aid: int, val: float) -> float:
    lo, hi = model.actuator_ctrlrange[aid]
    if np.isfinite(lo) and np.isfinite(hi):
        return float(np.clip(val, lo, hi))
    return val


def play_animation(xml_path: str,
                   json_path: str,
                   realtime: bool = True,
                   headless: bool = False,
                   loop: bool = False):
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    anim = load_animation(json_path)

    # Map joints -> actuators
    act_map = build_actuator_map(model, anim.joint_order)

    # Start at first frame exactly
    first = anim.frames[0]
    
    print("First frame:", first)
    set_qpos_from_frame(model, data, anim.joint_order, first, anim.units_angles_deg)
    print("First frame after :", first)

    # Viewer (optional)
    ctx = None
    if not headless:
        ctx = viewer.launch_passive(model, data) if realtime else viewer.launch_passive(model, data)

    # Simulation timing
    sim_dt = model.opt.timestep
    # Iterate frames; for the last frame, hold for a small grace period
    grace_hold = 0.25  # seconds

    def apply_targets(frame_dict: Dict[str, float]):
        # Set position targets on actuators (MuJoCo position actuators take the desired qpos as ctrl)
        for jn, aid in act_map.items():
            target = float(frame_dict[jn])
            val_rad = deg2rad_if_needed(target, anim.units_angles_deg)
            data.ctrl[aid] = clip_to_ctrlrange(model, aid, val_rad)

    # Main loop (optionally loop forever)
    try:
        keep_playing = True
        while keep_playing:
            # Apply first frame before stepping
            apply_targets(first)

            for i in range(len(anim.frames) - 1):
                f_now = anim.frames[i]
                f_next = anim.frames[i + 1]

                # Apply targets for the current frame
                apply_targets(f_now)

                # Step for the duration until next frame's timestamp
                t_now = float(f_now["time"])
                t_next = float(f_next["time"])
                duration = max(0.0, t_next - t_now)

                steps = max(1, int(round(duration / sim_dt)))
                for _ in range(steps):
                    mujoco.mj_step(model, data)
                    if ctx is not None and realtime:
                        ctx.sync()

            # Last frame: set targets and hold briefly
            last = anim.frames[-1]
            apply_targets(last)
            hold_steps = max(1, int(round(grace_hold / sim_dt)))
            for _ in range(hold_steps):
                mujoco.mj_step(model, data)
                if ctx is not None and realtime:
                    ctx.sync()

            keep_playing = loop
    finally:
        if ctx is not None:
            ctx.close()


def main():
    default_xml = os.environ.get("SO101_XML", "robots/so101/scene.xml")
    default_json = os.environ.get("SO101_ANIM", "data/so101_animation.json")

    parser = argparse.ArgumentParser(description="Play SO101 animation on MuJoCo model")
    parser.add_argument("--xml", type=str, default=default_xml,
                        help="Path to MuJoCo XML (e.g., so101_new_calib.xml or scene.xml)")
    parser.add_argument("--anim", type=str, default=default_json,
                        help="Path to so101_animation.json")
    parser.add_argument("--headless", action="store_true", help="Run without viewer")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false", help="Disable realtime viewer sync")
    parser.add_argument("--loop", action="store_true", help="Loop playback")

    args = parser.parse_args()
    play_animation(args.xml, args.anim, realtime=args.realtime, headless=args.headless, loop=args.loop)


if __name__ == "__main__":
    main()
