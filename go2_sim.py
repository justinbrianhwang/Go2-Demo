import math
import time

import pybullet as p
import pybullet_data


class Go2LikeRobot:
    def __init__(self, urdf_path: str):
        # PyBullet 연결
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)

        # 평면 생성
        self.plane_id = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, self.plane_id)
        start_pos = [0, 0, 0.45]
        start_orn = p.getQuaternionFromEuler([0, 0, 0])

        self.robot_id = p.loadURDF(
            urdf_path,
            start_pos,
            start_orn,
            flags=p.URDF_USE_INERTIA_FROM_FILE
        )

        # 조인트 이름 -> 인덱스 매핑
        self.joint_name_to_id = {}
        num_joints = p.getNumJoints(self.robot_id)
        for j in range(num_joints):
            info = p.getJointInfo(self.robot_id, j)
            name = info[1].decode("utf-8")
            self.joint_name_to_id[name] = j

        # 다리 조인트 (hip_roll, hip_pitch, knee)
        self.legs = {
            "fl": [
                self.joint_name_to_id["fl_hip_roll_joint"],
                self.joint_name_to_id["fl_hip_pitch_joint"],
                self.joint_name_to_id["fl_knee_joint"],
            ],
            "fr": [
                self.joint_name_to_id["fr_hip_roll_joint"],
                self.joint_name_to_id["fr_hip_pitch_joint"],
                self.joint_name_to_id["fr_knee_joint"],
            ],
            "rl": [
                self.joint_name_to_id["rl_hip_roll_joint"],
                self.joint_name_to_id["rl_hip_pitch_joint"],
                self.joint_name_to_id["rl_knee_joint"],
            ],
            "rr": [
                self.joint_name_to_id["rr_hip_roll_joint"],
                self.joint_name_to_id["rr_hip_pitch_joint"],
                self.joint_name_to_id["rr_knee_joint"],
            ],
        }

        # 기본 서 있는 포즈
        self.stand_hip_pitch = 0.6
        self.stand_knee = -1.2
        self.max_torque = 30.0

        # 간단한 궤적 맵
        self.traj_points = []
        self.last_traj_draw_pos = None

    def set_stand_pose(self):
        """기본 서 있는 포즈."""
        for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
            p.setJointMotorControl2(
                self.robot_id,
                hip_roll,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0.0,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                hip_pitch,
                controlMode=p.POSITION_CONTROL,
                targetPosition=self.stand_hip_pitch,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                knee,
                controlMode=p.POSITION_CONTROL,
                targetPosition=self.stand_knee,
                force=self.max_torque,
            )

    def apply_gait(self, vx_cmd: float, yaw_cmd: float, t: float, auto: bool = False):
        """아주 단순한 trot 스타일 보행 제어.
        vx_cmd: 전후 명령 (-1 ~ 1)
        yaw_cmd: yaw 회전 명령 (-1 ~ 1)
        """
        # 명령 범위 클램핑
        vx = max(-1.0, min(1.0, vx_cmd))
        yaw = max(-1.0, min(1.0, yaw_cmd))

        # 기본 관절 각
        base_hip_pitch = self.stand_hip_pitch
        base_knee = self.stand_knee

        # 보행 파라미터
        freq = 2.5 + 1.5 * abs(vx)   # [rad/s]
        step_amp = 0.4 * vx          # 앞뒤 스윙 크기
        lift_amp = 0.2               # 무릎 들어올리는 크기

        # yaw -> 좌우 롤 차이
        roll_amp = 0.2 * yaw

        # trot 패턴 (FL+RR vs FR+RL)
        phase = freq * t
        phases = {
            "fl": phase,
            "rr": phase,
            "fr": phase + math.pi,
            "rl": phase + math.pi,
        }

        for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
            leg_phase = phases[leg_name]

            # Hip pitch: 앞/뒤로 스윙
            hip_pitch_target = base_hip_pitch + step_amp * math.sin(leg_phase)

            # Knee: 스윙 구간에서 더 굽힘
            knee_target = base_knee + lift_amp * max(0.0, math.sin(leg_phase))

            # Hip roll: yaw 명령에 따라 좌/우 기울기
            if leg_name in ("fl", "rl"):
                hip_roll_target = +roll_amp
            else:
                hip_roll_target = -roll_amp

            p.setJointMotorControl2(
                self.robot_id,
                hip_roll,
                controlMode=p.POSITION_CONTROL,
                targetPosition=hip_roll_target,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                hip_pitch,
                controlMode=p.POSITION_CONTROL,
                targetPosition=hip_pitch_target,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                knee,
                controlMode=p.POSITION_CONTROL,
                targetPosition=knee_target,
                force=self.max_torque,
            )

        # 단순한 바디 이동 (정확한 동역학 X, 시각적으로만 이동)
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        yaw_euler = p.getEulerFromQuaternion(base_orn)[2]
        v_scale = 0.003  # 걷는 속도 스케일 (필요하면 조정)
        new_x = base_pos[0] + math.cos(yaw_euler) * vx * v_scale
        new_y = base_pos[1] + math.sin(yaw_euler) * vx * v_scale
        new_yaw = yaw_euler + yaw * 0.01

        new_orn = p.getQuaternionFromEuler([0, 0, new_yaw])
        p.resetBasePositionAndOrientation(self.robot_id, [new_x, new_y, base_pos[2]], new_orn)

    def update_trajectory_map(self):
        """베이스 위치를 기록하고 바닥에 선으로 궤적 표시."""
        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        x, y, z = base_pos

        current_point = (x, y, 0.01)
        self.traj_points.append(current_point)

        if self.last_traj_draw_pos is not None:
            p.addUserDebugLine(
                self.last_traj_draw_pos,
                current_point,
                [1, 0, 0],   # 빨간색
                lineWidth=1,
                lifeTime=0   # 계속 유지
            )
        self.last_traj_draw_pos = current_point

    def get_camera_image(self, width=128, height=128):
        """로봇에 붙어있는 카메라에서 RGB/Depth 한 프레임 가져오기."""
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        x, y, z = base_pos

        # 쿼터니언에서 로컬 x축(앞 방향) 추출
        rot_mat = p.getMatrixFromQuaternion(base_orn)
        forward = [rot_mat[0], rot_mat[3], rot_mat[6]]

        cam_eye = [x, y, z + 0.1]
        cam_target = [
            x + forward[0],
            y + forward[1],
            z + 0.1 + forward[2],
        ]
        cam_up = [0, 0, 1]

        view = p.computeViewMatrix(cam_eye, cam_target, cam_up)
        proj = p.computeProjectionMatrixFOV(
            fov=60.0,
            aspect=float(width) / float(height),
            nearVal=0.1,
            farVal=5.0,
        )
        img = p.getCameraImage(
            width,
            height,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
        )
        # img[2]가 RGB, img[3]가 depth
        return img

    def shutdown(self):
        p.disconnect(self.physics_client)


def run_simulation(urdf_path: str = "go2_like.urdf"):
    robot = Go2LikeRobot(urdf_path)
    robot.set_stand_pose()

    auto_mode = False
    print("=== Controls ===")
    print("W/S: forward/backward")
    print("A/D: turn left/right")
    print("P  : toggle auto-walk mode")
    print("C  : capture camera image (terminal info only)")
    print("Q  : quit\n")

    sim_start_real = time.time()

    try:
        while True:
            sim_time = time.time() - sim_start_real
            keys = p.getKeyboardEvents()

            vx_cmd = 0.0
            yaw_cmd = 0.0

            for k, v in keys.items():
                # P: 자동 모드 토글
                if k in (ord('p'), ord('P')) and (v & p.KEY_WAS_TRIGGERED):
                    auto_mode = not auto_mode
                    mode_str = "AUTO" if auto_mode else "MANUAL"
                    print(f"[INFO] Auto mode: {mode_str}")

                # Q: 종료
                if k in (ord('q'), ord('Q')) and (v & p.KEY_WAS_TRIGGERED):
                    print("[INFO] Quit requested.")
                    return

                # 수동 모드에서만 WASD 반영
                if not auto_mode and (v & p.KEY_IS_DOWN):
                    if k in (ord('w'), ord('W')):
                        vx_cmd += 1.0
                    if k in (ord('s'), ord('S')):
                        vx_cmd -= 1.0
                    if k in (ord('a'), ord('A')):
                        yaw_cmd += 1.0
                    if k in (ord('d'), ord('D')):
                        yaw_cmd -= 1.0

                # C: 카메라 테스트
                if k in (ord('c'), ord('C')) and (v & p.KEY_WAS_TRIGGERED):
                    img = robot.get_camera_image()
                    print("[INFO] Captured camera image (RGB size: {}x{})".format(img[0], img[1]))

            # 자동 모드면 간단한 패턴으로 알아서 움직임
            if auto_mode:
                vx_cmd = math.sin(sim_time * 0.5)        # 천천히 속도 변화
                yaw_cmd = math.sin(sim_time * 0.3) * 0.5 # 좌우로 약간씩 회전

            # 보행 + 맵 업데이트
            robot.apply_gait(vx_cmd, yaw_cmd, sim_time, auto=auto_mode)
            robot.update_trajectory_map()

            # 시뮬레이션 스텝
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    finally:
        robot.shutdown()


if __name__ == "__main__":
    run_simulation()
