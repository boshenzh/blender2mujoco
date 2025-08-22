#!/usr/bin/env python3
"""
Blender → MuJoCo (object-level) keyframe exporter
(… docstring unchanged …)
"""

import argparse, json, math, os, sys, glob
from collections import OrderedDict
import bpy
from mathutils import Vector, Quaternion
import math

# ------------------------- Joint sign config -------------------------
# Set to +1 or -1 per joint to flip directions as needed.
# If you only want one or two joints flipped, just change those to -1.
JOINT_SIGN = {
    "shoulder_pan":  +1,
    "shoulder_lift": +1,
    "elbow_flex":    +1,
    "wrist_flex":    +1,
    "wrist_roll":    +1,
    "gripper":       +1,
}
AXIS_DEFAULTS = OrderedDict([
    ("shoulder_link", "Z"),                 # pan (yaw)
    ("upper_arm_link", "Y"),                # shoulder_lift (pitch)
    ("lower_arm_link", "Y"),                # elbow_flex
    ("wrist_link", "Y"),                    # wrist_flex
    ("gripper_link", "Z"),                  # wrist_roll
    ("moving_jaw_so101_v1_link", "Z"),      # gripper
])


# ------------------------- CLI -------------------------

def parse_kv_list(values, label):
    out = OrderedDict()
    for v in values or []:
        if ":" not in v:
            raise SystemExit(f"Invalid {label} entry '{v}'. Expected name:value")
        k, val = v.split(":", 1)
        out[k] = val
    return out

def axis_from_name(name: str) -> Vector:
    name = name.strip().upper()
    sign = 1.0
    if name.startswith("-"):
        sign = -1.0
        name = name[1:]
    base = {"X": Vector((1,0,0)), "Y": Vector((0,1,0)), "Z": Vector((0,0,1))}[name]
    return base * sign, ("-" if sign < 0 else "") + name

def open_blend_from_blend_dir(user_blend: str | None):
    proj_root = os.getcwd()
    blend_dir = os.path.join(proj_root, "blend")
    if not os.path.isdir(blend_dir):
        raise SystemExit(f"blend/ directory not found at: {blend_dir}")

    if user_blend:
        path = user_blend if user_blend.endswith(".blend") else f"{user_blend}.blend"
        if not os.path.isabs(path):
            path = os.path.join(blend_dir, path)
        if not os.path.isfile(path):
            raise SystemExit(f"Blend file not found: {path}")
        bpy.ops.wm.open_mainfile(filepath=path)
        print(f"[INFO] Opened blend: {path}")
        return

    candidates = sorted(glob.glob(os.path.join(blend_dir, "*.blend")))
    if len(candidates) == 0:
        raise SystemExit(f"No .blend files found in {blend_dir}")
    if len(candidates) > 1:
        names = [os.path.basename(p) for p in candidates]
        raise SystemExit(f"Multiple .blend files in {blend_dir}. Pass one with --blend. Found: {names}")
    bpy.ops.wm.open_mainfile(filepath=candidates[0])
    print(f"[INFO] Opened blend: {candidates[0]}")

# ------------------------- Rotation math -------------------------

def twist_about_axis(obj: bpy.types.Object, local_axis: Vector) -> float:
    """
    Signed twist angle (radians) of obj's local rotation about 'local_axis'.
    Uses swing-twist decomposition: q = q_swing * q_twist, return angle(q_twist).
    """
    # get quaternion in object local space
    if obj.rotation_mode == 'QUATERNION':
        q = obj.rotation_quaternion.copy()
    elif obj.rotation_mode == 'AXIS_ANGLE':
        ang, ax, ay, az = obj.rotation_axis_angle
        axis = Vector((ax, ay, az)).normalized()
        q = Quaternion(axis, ang)
    else:
        q = obj.rotation_euler.to_quaternion()

    a = Vector(local_axis).normalized()

    # q_twist has axis parallel to 'a', with vector part = projection of q.xyz onto a
    vx, vy, vz = q.x, q.y, q.z
    px = a.x * (vx*a.x + vy*a.y + vz*a.z)
    py = a.y * (vx*a.x + vy*a.y + vz*a.z)
    pz = a.z * (vx*a.x + vy*a.y + vz*a.z)
    q_twist = Quaternion((q.w, px, py, pz))
    q_twist.normalize()

    # signed angle about +a: angle = 2*atan2(|v|, w), sign from dot(v, a)
    angle = 2.0 * math.atan2(math.sqrt(q_twist.x*q_twist.x + q_twist.y*q_twist.y + q_twist.z*q_twist.z), q_twist.w)
    sign = 1.0 if (q_twist.x*a.x + q_twist.y*a.y + q_twist.z*a.z) >= 0.0 else -1.0
    return sign * angle

# ------------------------- Main -------------------------

def main():
    if "--" in sys.argv:
        extra = sys.argv[sys.argv.index("--")+1:]
    else:
        extra = sys.argv[1:]
    
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="output/so101_animation.json", help="Output JSON path")
    ap.add_argument("--blend", default="pan.blend", help="Which .blend in ./blend/ to open")
    ap.add_argument("--sample_hz", type=float, default=None, help="If not set, sample per scene frame (fps)")
    ap.add_argument("--mapping", nargs="*", help="Pairs blenderObject:muJoCoJoint (space separated)")
    ap.add_argument("--axisOverride", nargs="*", help="Pairs blenderObject:axis where axis∈{X,Y,Z,-X,-Y,-Z}")
    ap.add_argument("--start_frame", type=int, default=None)
    ap.add_argument("--end_frame", type=int, default=None)
    # Optional CLI override for signs, e.g., --signs shoulder_pan:-1 wrist_roll:-1
    ap.add_argument("--signs", nargs="*", help="Pairs joint:+1|-1 to override JOINT_SIGN at runtime")
    args = ap.parse_args(extra)

    # Open the .blend from ./blend/
    open_blend_from_blend_dir(args.blend)

    scene = bpy.context.scene
    fps = scene.render.fps / (scene.render.fps_base or 1.0)

    # Default mapping for your SO101 object names
    link_to_joint = parse_kv_list(args.mapping, "mapping") if args.mapping else OrderedDict([
        ("shoulder_link", "shoulder_pan"),
        ("upper_arm_link", "shoulder_lift"),
        ("lower_arm_link", "elbow_flex"),
        ("wrist_link", "wrist_flex"),
        ("gripper_link", "wrist_roll"),
        ("moving_jaw_so101_v1_link", "gripper"),
    ])

    # Resolve per-object axis (vector + label)
    per_obj_axis = {}
    per_joint_axis_label = {}
    # If no CLI override, fall back to AXIS_DEFAULTS
    axis_override = parse_kv_list(args.axisOverride, "axisOverride") if args.axisOverride else AXIS_DEFAULTS

    for obj_name, joint in link_to_joint.items():
        axis_name = axis_override.get(obj_name, "Z")
        vec, label = axis_from_name(axis_name)
        per_obj_axis[obj_name] = vec
        per_joint_axis_label[joint] = f"local_{label}"
    # Validate objects exist
    missing = [name for name in link_to_joint.keys() if name not in bpy.data.objects]
    if missing:
        present = ", ".join(sorted(o.name for o in bpy.data.objects))
        raise SystemExit(f"[ERROR] Missing Blender objects: {missing}\nAvailable: {present}")

    # Frame range
    f0 = args.start_frame if args.start_frame is not None else scene.frame_start
    f1 = args.end_frame   if args.end_frame   is not None else scene.frame_end
    if f1 < f0:
        raise SystemExit(f"[ERROR] end_frame({f1}) < start_frame({f0})")

    depsgraph = bpy.context.evaluated_depsgraph_get()
    # --- Baseline (URDF zero) measurement ---
    # Remember current frame to restore later
    frame_prev = scene.frame_current

    # Evaluate at baseline frame (URDF zero)
    scene.frame_set(0); depsgraph.update() #NOTE: I assume animator start animate from frame 1; so frame 0 matches the urdf zero position. 

    baseline_deg = {}
    for obj_name, joint in link_to_joint.items():
        obj = bpy.data.objects[obj_name]
        ang0 = twist_about_axis(obj, per_obj_axis[obj_name])  # radians
        baseline_deg[joint] = math.degrees(ang0)


    scene.frame_set(f0); depsgraph.update()

    # Effective sign map (combine constant + optional CLI overrides)
    sign_map = dict(JOINT_SIGN)
    if args.signs:
        for k, v in parse_kv_list(args.signs, "signs").items():
            try:
                sign_map[k] = int(v)
                if sign_map[k] not in (-1, 1):
                    raise ValueError
            except Exception:
                raise SystemExit(f"[ERROR] Invalid sign for joint '{k}': '{v}'. Use +1 or -1.")
    # Ensure all mapped joints have a sign
    for j in link_to_joint.values():
        sign_map.setdefault(j, +1)

    # Measure zero offsets at first frame
    zero_offset_deg = {}
    for obj_name, joint in link_to_joint.items():
        obj = bpy.data.objects[obj_name]
        ang0 = twist_about_axis(obj, per_obj_axis[obj_name])
        zero_offset_deg[joint] = math.degrees(ang0)

    # Sampling strategy
    frames = []
    joint_order = list(link_to_joint.values())
    if args.sample_hz is None:
        # Per-frame sampling
        print(f"[INFO] Sampling per frame: {f0}..{f1} at fps={fps}")
        for f in range(f0, f1 + 1):
            scene.frame_set(f); depsgraph.update()
            t = (f - f0) / float(fps)
            row = {"time": float(t)}
            for obj_name, joint in link_to_joint.items():
                obj = bpy.data.objects[obj_name]
                ang_rad = twist_about_axis(obj, per_obj_axis[obj_name])
                val_deg = math.degrees(ang_rad) - baseline_deg[joint]
                row[joint] = sign_map.get(joint, 1) * val_deg

            frames.append(row)
        sample_hz = fps
        duration_sec = (f1 - f0) / float(fps)
    else:
        # Fixed-rate sampling
        sample_hz = float(args.sample_hz)
        if sample_hz <= 0:
            raise SystemExit("--sample_hz must be > 0")
        duration_sec = max(0.0, (f1 - f0) / float(fps))
        n = max(1, int(round(duration_sec * sample_hz)) + 1)
        print(f"[INFO] Sampling at {sample_hz} Hz for {n} samples over {duration_sec:.3f}s")
        for i in range(n):
            t = i / sample_hz
            f_float = f0 + t * fps
            scene.frame_set(int(round(f_float))); depsgraph.update()
            row = {"time": float(t)}
            for obj_name, joint in link_to_joint.items():
                obj = bpy.data.objects[obj_name]
                ang_rad = twist_about_axis(obj, per_obj_axis[obj_name])

                val_deg = math.degrees(ang_rad) - baseline_deg[joint]
                row[joint] = sign_map.get(joint, 1) * val_deg
            frames.append(row)

    data = OrderedDict([
        ("fps", float(fps)),
        ("sample_hz", float(sample_hz)),
        ("duration_sec", float(duration_sec)),
        ("joint_order", joint_order),
        ("axis_used", per_joint_axis_label),
        ("units", {"angles": "deg"}),
        ("frames", frames),
        ("zero_offsets_deg", zero_offset_deg),
        ("signs_used", sign_map),
    ])

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
    print(f"[OK] Wrote {args.out} with {len(frames)} samples for joints: {', '.join(joint_order)}")

if __name__ == "__main__":
    main()
