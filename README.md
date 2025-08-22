# Blender2Mujoco

A Python toolkit for exporting robot animations from Blender to MuJoCo physics simulations.

## Overview

Blender2Mujoco enables you to create robot animations in Blender and seamlessly transfer them to MuJoCo for physics simulation. The project specifically supports the SO-101 robotic arm but is designed to be extensible to other robots. 

Note: Currently only support [SO101](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101), will add more in upcoming days.. 

## Features

- **Blender Animation Export**: Extract joint rotations from Blender animations and convert them to keyframe data
- **MuJoCo Integration**: Import and play animations in MuJoCo physics simulator with proper joint control
- **Flexible Joint Mapping**: Configure custom mappings between Blender objects and MuJoCo joints
- **Multiple Sampling Modes**: Export at scene frame rate or custom sampling frequency
- **URDF Compatibility**: Support for URDF-based robot models with automatic zero-position handling

## Installation

### Prerequisites

- Python 3.11+
- Blender (with Python API access)
- MuJoCo 3.3.5+

### Setup

```bash
# Clone the repository
git clone <repository-url>
cd blender2mujoco


# Create and activate virtual environment using uv
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
uv sync

# Alternatively, install in development mode
uv pip install -e .
```

## Quick Start

### 1. Download rigs and create your own animation with Blender
rigs are provided under rigs/. To support different robot, import robot URDF to blender (with [phobos](https://github.com/dfki-ric/phobos) plugin). 
Open rigs, create your own animation in "object mode", be creative! 
### 1. Export Animation from Blender

Place your `.blend` files in the `blend/` directory, then run:

```bash
python exporter_min.py --out output/my_animation.json --blend my_robot.blend
```

### 2. Play Animation in MuJoCo

```bash
python play_so101_animation.py --xml robots/so101/scene.xml --anim output/my_animation.json
```

## Project Structure

```
blender2m/
     blend/                    # Blender animation files (.blend)
     robots/so101/            # SO-101 robot model files
        scene.xml           # MuJoCo scene configuration
        so101_new_calib.xml # Robot URDF/XML definition
        assets/             # 3D model assets (.stl files)
        output/                 # Exported animation JSON files
        mujoco_wasm/           # MuJoCo WebAssembly build (optional)
        emsdk/                 # Emscripten SDK for WASM builds
        exporter_min.py        # Main Blender animation exporter
        play_so101_animation.py # MuJoCo animation player
        main.py                # Entry point script
```

## Usage

### Animation Export Options

```bash
python exporter_min.py \
  --out output/animation.json \
  --blend my_animation.blend \
  --sample_hz 30 \
  --start_frame 1 \
  --end_frame 100 \
  --mapping shoulder_link:shoulder_pan upper_arm_link:shoulder_lift \
  --signs shoulder_pan:-1 wrist_roll:-1
```

**Parameters:**
- `--out`: Output JSON file path
- `--blend`: Blender file to process (searches in `blend/` directory)
- `--sample_hz`: Sampling frequency (defaults to scene FPS)
- `--start_frame`/`--end_frame`: Frame range to export
- `--mapping`: Custom object-to-joint mappings
- `--signs`: Joint rotation direction overrides (+1 or -1)

### Animation Playback Options

```bash
python play_so101_animation.py \
  --xml robots/so101/scene.xml \
  --anim output/animation.json \
  --loop \
  --headless
```

**Parameters:**
- `--xml`: MuJoCo model XML file
- `--anim`: Animation JSON file to play
- `--loop`: Loop animation playback
- `--headless`: Run without GUI viewer
- `--no-realtime`: Disable real-time synchronization

## Default Joint Mappings (SO-101)

| Blender Object | MuJoCo Joint | Axis |
|---------------|-------------|------|
| `shoulder_link` | `shoulder_pan` | Z |
| `upper_arm_link` | `shoulder_lift` | Y |
| `lower_arm_link` | `elbow_flex` | Y |
| `wrist_link` | `wrist_flex` | Y |
| `gripper_link` | `wrist_roll` | Z |
| `moving_jaw_so101_v1_link` | `gripper` | Z |

## Animation Data Format

Exported JSON contains:
```json
{
  "fps": 24.0,
  "sample_hz": 24.0,
  "duration_sec": 4.167,
  "joint_order": ["shoulder_pan", "shoulder_lift", ...],
  "units": {"angles": "deg"},
  "frames": [
    {
      "time": 0.0,
      "shoulder_pan": 0.0,
      "shoulder_lift": -15.2,
      ...
    }
  ]
}
```

## Dependencies

- `bpy>=4.5.1` - Blender Python API
- `mujoco>=3.3.5` - MuJoCo physics engine
- `numpy` - Numerical computations
- `mathutils` - 3D math utilities (from Blender)

## Contributing

happy to discuss! discord:zbs#8590. email: 809699809@qq.com
## TODO
- [ ] Web-based animation viewer using MuJoCo WASM
- [ ] support for other robot
- [ ] support for controlling real robot like [bambot](bambot.org)

## License

See LICENSE file for details.

## Troubleshooting
 