# Go2-Like Quadruped Robot Demo (PyBullet)

<img width="963" height="615" alt="image" src="https://github.com/user-attachments/assets/1a2d55df-ae27-4d04-991e-b1bfd007691c" />


A **demo-level quadruped robot simulation** inspired by the Unitree Go2 Air, implemented entirely in **PyBullet**.  
This project is NOT an official Unitree product, nor does it attempt to perfectly replicate Unitree's mechanical or control systems.  
It is a **non-commercial educational demo** for robotics learning, simulation, and prototyping.

---

##  Features

- **12-DoF quadruped robot URDF**
- **WASD manual walking control**
- **Auto-walk mode (`P` key)**  
  Simple gait controller with trot-style stepping
- **Basic camera sensor simulation**
- **Trajectory mapping** by drawing path lines on the ground
- Fully contained Python + URDF implementation

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

## Controls
| Key     | Action                                          |
| ------- | ----------------------------------------------- |
| **W/S** | Move forward / backward                         |
| **A/D** | Turn left / right                               |
| **P**   | Toggle auto-walk mode                           |
| **C**   | Capture camera frame (info printed to terminal) |
| **Q**   | Quit simulation                                 |

## Notes & Disclaimer
- This is a demo version, intended for research, education, and experimentation.
- It does not attempt to replicate the full physics, controls, or sensing capabilities of real quadruped robots.
- The gait controller is intentionally simplified.
- For academic or commercial-grade robotics simulation, consider combining with ROS2, Isaac Gym, Mujoco, or Bullet Real-Time API.


## Demo Video
[video](https://www.youtube.com/watch?v=QqI0uDcRvp8)

## License
MIT License

This project is released for educational and research purposes only.





