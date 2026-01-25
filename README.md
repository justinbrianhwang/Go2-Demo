# Go2-Like Quadruped Robot Demo (PyBullet)

<img width="1216" height="743" alt="image" src="https://github.com/user-attachments/assets/5c90c2b0-4798-45cd-b049-e6e551f72875" />



A **demo-level quadruped robot simulation** inspired by the Unitree Go2 Air, implemented entirely in **PyBullet**.
This project is NOT an official Unitree product, nor does it attempt to perfectly replicate Unitree's mechanical or control systems.
It is a **non-commercial educational demo** for robotics learning, simulation, and prototyping.

**NEW in Phase 3**: Autonomous navigation with ray-based obstacle detection and automatic jumping!

---

## ✨ Features (Phase 3 Complete!)

### 🤖 Autonomous Navigation (Phase 3 - NEW!)
- **AUTO Mode** - Press **P** to toggle autonomous navigation
- **Ray-based Obstacle Detection** - 4-direction ray casting (forward, left 45°, right 45°, upward)
- **Intelligent Avoidance** - Automatically turns away from obstacles
- **Automatic Jumping** - Detects low obstacles and jumps over them
- **Exploration Behavior** - Random direction changes for area coverage
- **Real-time Decision Making** - Adaptive navigation based on sensor data

### 🗺️ Multi-Environment Testing (Phase 2)
- **5 Diverse Maps** with different testing purposes:
  - **Map 1: Urban Park** - Flat terrain with grass/pavement mix, perfect for stability testing

    <img width="1403" height="842" alt="image" src="https://github.com/user-attachments/assets/4c052766-7138-4eba-882b-0b30fc20845b" />
    
  - **Map 2: Campus** - Stairs, ramps, and walkways for navigation testing

    <img width="1388" height="845" alt="image" src="https://github.com/user-attachments/assets/646c6a5e-5334-4c0a-8c83-a48054ce7bdb" />

  - **Map 3: Warehouse** - Repetitive structure ideal for SLAM and path planning
 
    <img width="1416" height="849" alt="image" src="https://github.com/user-attachments/assets/1193c897-77b1-4dfd-9c3f-8aebdd4ef555" />
 
    
  - **Map 4: Parking Lot** - Slopes, pillars, and narrow spaces for sensor robustness
 
    <img width="1414" height="848" alt="image" src="https://github.com/user-attachments/assets/213acac0-584a-4e0f-b765-8e6e381b9e95" />


  - **Map 5: Exhibition Hall** - Large indoor space for GPS-free navigation
 
    <img width="1409" height="859" alt="image" src="https://github.com/user-attachments/assets/0f225d36-3d99-47ed-9e36-ac4b0723dab1" />



- **Easy Map Switching** - Press **M** to cycle through maps
- **Auto-Reset** - Robot position and trajectory reset when changing maps

### 🤖 Advanced Motion Control (Phase 1)
- **12-DoF quadruped robot URDF** with realistic silver design
- **Multiple Gaits**:
  - Walking (trot-style stepping)
  - Running (faster gait with Shift modifier)
- **Complex Actions**:
  - **Jump** - Complete jump sequence (crouch → jump → land)
  - **Stretch** - 3-second stretching motion
  - **Wave** - Front left leg waving gesture (2 seconds)
- **Postures**:
  - Standing
  - Sitting
  - Lying down (stable, recoverable)
- **Trajectory mapping** - Real-time red path visualization on ground
- **Camera sensor simulation** - First-person RGB/Depth camera

### 🎨 Enhanced Visual Design
- **Realistic Go2 appearance**: Silver-gray metallic body
- **Cylindrical legs** with spherical joints
- **Black rubber foot pads** for realistic contact
- **Multi-layered body design** with darker top section

---

## 📁 Project Structure
```
Go2-Demo/
├── go2_sim.py           # Main simulation entry point
├── go2_like.urdf        # Robot URDF model
├── README.md
├── LICENSE
├── robot/               # Robot module
│   ├── __init__.py
│   └── go2_robot.py     # Go2LikeRobot class
├── maps/                # Map environments
│   ├── __init__.py
│   ├── map_loader.py    # Map loader & manager
│   ├── urban_park.py    # Map 1: Urban Park
│   ├── campus.py        # Map 2: Campus
│   ├── warehouse.py     # Map 3: Warehouse
│   ├── parking.py       # Map 4: Parking Lot
│   └── exhibition.py    # Map 5: Exhibition Hall
└── controllers/         # Future: Advanced controllers
    └── __init__.py
```


## Installation

```bash
pip install pybullet
```

## Running

```bash
python go2_sim.py
```

## Quick Start Guide

1. **Manual Control**: Use WASD keys to move the robot around
2. **Try Actions**: Press SPACE to jump, R to stretch, T to wave
3. **Change Maps**: Press M to cycle through 5 different environments
4. **Go Autonomous**: Press P to enable AUTO mode and watch the robot navigate by itself!
5. **Exit**: Press ESC to quit

## 🎮 Controls

### Movement
| Key | Action |
|-----|--------|
| **W** | Move forward |
| **S** | Move backward |
| **A** | Turn left (while moving) |
| **D** | Turn right (while moving) |
| **Q** | Rotate left (in place) |
| **E** | Rotate right (in place) |
| **Shift + WASD** | Run (faster movement) |

### Actions
| Key | Action |
|-----|--------|
| **Spacebar** | Jump |
| **R** | Stretch (3 seconds) |
| **T** | Wave (2 seconds) |

### Postures
| Key | Action |
|-----|--------|
| **Z** | Sit down |
| **X** | Stand up |
| **C** | Lie down |

### Autonomous Control
| Key | Action |
|-----|--------|
| **P** | Toggle AUTO mode (autonomous navigation) |

### Environment
| Key | Action |
|-----|--------|
| **M** | Switch to next map |

### System
| Key | Action |
|-----|--------|
| **ESC** | Quit simulation |

> **Note**: PyBullet's default keyboard shortcuts are disabled. Camera control is mouse-only.

## 🔧 Technical Details

### Gait Algorithm
- **Trot Pattern**: Diagonal leg pairs move together
  - FL (Front Left) + RR (Rear Right)
  - FR (Front Right) + RL (Rear Left)
- **Adaptive Frequency**: Gait speed adjusts with velocity
- **Swing Amplitude**: Proportional to forward speed

### Joint Control
- **Control Mode**: Position Control (PD controller)
- **Max Torque**: 30 Nm per joint
- **Joint Limits**: Defined in URDF for realistic motion

### Physics
- **Gravity**: 9.81 m/s²
- **Timestep**: 1/240 seconds
- **Ground Friction**: Environment-specific

### Autonomous Navigation (Phase 3)
- **Sensor System**: Ray casting in 4 directions
  - Forward ray: 2.0m detection range
  - Left/Right rays: 45° angles for side detection
  - Upward ray: 1.0m range for jump decision
- **Decision Algorithm**:
  - **Obstacle < 0.8m ahead**: Check if jumpable (low obstacle) or turn away (high obstacle)
  - **Side obstacle < 0.5m**: Turn away from closer side
  - **Path clear**: Continue forward
  - **Random exploration**: Occasional direction changes every 15 seconds
- **Jump Logic**: Triggers when obstacle is low enough (upward clearance > 0.4m)
- **Cooldown System**: Prevents rapid decision changes (0.5-2.0 seconds)

### Map Environments
Each map is designed for specific testing scenarios:

#### Map 1: Urban Park 🌳
- **Terrain**: Flat walkways with grass/pavement mix
- **Obstacles**: Benches, lampposts, trees, trash bins
- **Features**: Small hill for slope testing
- **Test Focus**: Walking stability, ground adaptation, low-speed tracking

#### Map 2: Campus 🏫
- **Terrain**: Stairs, ramps, mixed walkways
- **Obstacles**: Building walls, benches, bike racks, signs
- **Features**: 5-step staircase, 15° ramp
- **Test Focus**: Stair approach detection, path planning, crowd handling

#### Map 3: Warehouse 📦
- **Terrain**: Concrete floor with lane markings
- **Obstacles**: Shelving units (3x4 grid), pallets, safety cones
- **Features**: Repetitive structure, wide sight lines
- **Test Focus**: SLAM, obstacle avoidance, path reliability

#### Map 4: Parking Lot 🚗
- **Terrain**: Asphalt with parking markings, 12° ramp
- **Obstacles**: Parked cars (11 vehicles), pillars, speed bumps
- **Features**: Slopes, narrow spaces, lighting variation
- **Test Focus**: Low-light vision, ramp handling, tight space navigation

#### Map 5: Exhibition Hall 🖼️
- **Terrain**: Marble floor, large open indoor space
- **Obstacles**: Exhibition booths (20+ units), central statue, info desks
- **Features**: 12m x 12m enclosed hall, pillars
- **Test Focus**: GPS-free navigation, crowd avoidance, dynamic obstacles

---

## 🚀 Roadmap

### ✅ Phase 1 - Basic Motion Control (COMPLETE)
- [x] Multiple gaits (walk, run)
- [x] Complex actions (jump, stretch, wave)
- [x] Posture control (stand, sit, lie)
- [x] Trajectory visualization
- [x] Camera simulation

### ✅ Phase 2 - Multi-Environment Testing (COMPLETE)
- [x] 5 diverse test environments
- [x] Map switching system
- [x] Modular code structure

### ✅ Phase 3 - Autonomous Navigation (COMPLETE)
- [x] Ray-based obstacle detection (4 directions)
- [x] Autonomous navigation with intelligent pathfinding
- [x] Automatic obstacle avoidance
- [x] Automatic jumping over low obstacles
- [x] Exploration behavior with random direction changes
- [x] Real-time decision making system

### 📅 Phase 4 - Advanced Features (Planned)
- [ ] LiDAR simulation (360° scanning)
- [ ] SLAM (Simultaneous Localization and Mapping)
- [ ] Advanced path planning (A*, RRT)
- [ ] Stair climbing capability
- [ ] Target following mode
- [ ] IMU sensor simulation
- [ ] Battery & energy management
- [ ] Deep learning-based navigation

---

## Notes & Disclaimer
- **Phase 3 Complete**: Autonomous navigation with obstacle avoidance fully implemented!
- This is an educational demo, not a production-grade simulation.
- Does not replicate full physics/controls of real quadruped robots.
- Gait controller is simplified for demonstration purposes.
- For academic/commercial robotics: consider ROS2, Isaac Gym, MuJoCo, or Bullet Real-Time API.

## 🐛 Known Issues
- None currently! All Phase 1, 2 & 3 features are stable.
- Phase 4 features (LiDAR, SLAM, AI navigation) coming soon!

## 🤝 Contributing
Bug reports and feature suggestions are welcome via issues!


## Demo Video
## Demo Video

[![Demo Video](https://img.youtube.com/vi/3vraZ8u-9C8/hqdefault.jpg)](https://youtu.be/3vraZ8u-9C8)


## 🙏 Acknowledgments
- **Unitree Robotics** - Go2 robot design inspiration
- **PyBullet** - Excellent physics simulation engine
- **Open Source Community** - For continuous support

---

## 📄 License
MIT License

This project is released for educational and research purposes only.

---

**Made with ❤️ using PyBullet and Python**



