import json, sys

inp = sys.argv[1]  # so101_animation.json
out = sys.argv[2] if len(sys.argv) > 2 else inp.replace(".json", "_absolute.json")

with open(inp, "r") as f:
    D = json.load(f)

signs = D.get("signs_used", {})
zero = D.get("zero_offsets_deg", {})
J = D["joint_order"]

abs_frames = []
for fr in D["frames"]:
    row = {"time": fr["time"]}
    for j in J:
        s = signs.get(j, 1)
        row[j] = float(fr[j]) + s * float(zero.get(j, 0.0))
    abs_frames.append(row)

D["frames"] = abs_frames
D["note"] = "Converted to absolute: frame[j] += signs_used[j] * zero_offsets_deg[j]"
with open(out, "w") as f:
    json.dump(D, f, indent=2)
print("Wrote", out)
