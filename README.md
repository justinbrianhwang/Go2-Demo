# Go2-Like Quadruped Robot Demo (PyBullet)

<img width="1216" height="743" alt="image" src="https://github.com/user-attachments/assets/5c90c2b0-4798-45cd-b049-e6e551f72875" />



A **demo-level quadruped robot simulation** inspired by the Unitree Go2 Air, implemented entirely in **PyBullet**.
This project is NOT an official Unitree product, nor does it attempt to perfectly replicate Unitree's mechanical or control systems.
It is a **non-commercial educational demo** for robotics learning, simulation, and prototyping.

---

## ✨ Features (Phase 1 Complete!)

### 🤖 Advanced Motion Control
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

## Project Structure
```
├── go2_like.urdf # Quadruped robot model
├── go2_sim.py # Main PyBullet simulation and controller
└── README.md
```


## Installation

```bash
pip install pybullet
```
## Running

```bash
python go2_sim.py
```

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
- **Ground Plane**: PyBullet default friction

---

## 🚀 Future Roadmap

### Phase 2 - Intermediate Features (Planned)
- [ ] Obstacle avoidance using depth camera
- [ ] Auto-follow mode (track targets)
- [ ] IMU sensor simulation
- [ ] Battery simulation

### Phase 3 - Advanced Features (Planned)
- [ ] LiDAR simulation (ray casting)
- [ ] SLAM (mapping & path planning)
- [ ] Obstacle climbing
- [ ] AI-based autonomous navigation

---

## Notes & Disclaimer
- **Phase 1 Complete**: All basic motion controls and actions are now implemented!
- This is an educational demo, not a production-grade simulation.
- Does not replicate full physics/controls of real quadruped robots.
- Gait controller is simplified for demonstration purposes.
- For academic/commercial robotics: consider ROS2, Isaac Gym, MuJoCo, or Bullet Real-Time API.

## 🐛 Known Issues
- None currently! All Phase 1 features are stable.

## 🤝 Contributing
Bug reports and feature suggestions are welcome via issues!


## Demo Video
[video](https://www.youtube.com/watch?v=QqI0uDcRvp8)

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



