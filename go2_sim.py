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

        # PyBullet GUI 기본 키보드 단축키 비활성화 (우리 컨트롤만 사용)
        p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

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

        # 동작 상태
        self.current_mode = "standing"  # standing, walking, running, jumping, sitting, lying, stretching, waving
        self.jump_start_time = None
        self.stretch_start_time = None
        self.wave_start_time = None

        # 간단한 궤적 맵
        self.traj_points = []
        self.last_traj_draw_pos = None

        # UI 텍스트
        self.status_text_id = None

    def update_status_display(self, mode, vx=0, yaw=0):
        """상태 표시 업데이트 (비활성화됨)"""
        # UI 텍스트 표시 비활성화
        pass

    def set_stand_pose(self):
        """기본 서 있는 포즈"""
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

    def set_sit_pose(self):
        """앉기 자세"""
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
                targetPosition=1.2,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                knee,
                controlMode=p.POSITION_CONTROL,
                targetPosition=-2.0,
                force=self.max_torque,
            )

    def set_lie_pose(self):
        """눕기 자세 - 더 안정적으로 개선"""
        for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
            # 앞다리와 뒷다리를 앞뒤로 펼침 (더 안정적)
            if leg_name in ("fl", "fr"):
                # 앞다리는 앞으로
                pitch_target = 0.2
                roll_target = 0.15 if leg_name == "fl" else -0.15
            else:
                # 뒷다리는 뒤로
                pitch_target = 1.0
                roll_target = 0.15 if leg_name == "rl" else -0.15

            p.setJointMotorControl2(
                self.robot_id,
                hip_roll,
                controlMode=p.POSITION_CONTROL,
                targetPosition=roll_target,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                hip_pitch,
                controlMode=p.POSITION_CONTROL,
                targetPosition=pitch_target,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                knee,
                controlMode=p.POSITION_CONTROL,
                targetPosition=-0.3,
                force=self.max_torque,
            )

    def perform_jump(self, t: float):
        """점프 동작"""
        if self.jump_start_time is None:
            self.jump_start_time = t

        elapsed = t - self.jump_start_time

        if elapsed < 0.2:
            # 준비 동작 (몸 낮춤)
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
                    targetPosition=1.0,
                    force=self.max_torque,
                )
                p.setJointMotorControl2(
                    self.robot_id,
                    knee,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=-1.8,
                    force=self.max_torque,
                )
        elif elapsed < 0.4:
            # 점프 (다리 펴기)
            for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
                p.setJointMotorControl2(
                    self.robot_id,
                    hip_roll,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0.0,
                    force=self.max_torque * 2,
                )
                p.setJointMotorControl2(
                    self.robot_id,
                    hip_pitch,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=-0.3,
                    force=self.max_torque * 2,
                )
                p.setJointMotorControl2(
                    self.robot_id,
                    knee,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=0.2,
                    force=self.max_torque * 2,
                )
        elif elapsed < 0.8:
            # 착지 준비
            self.set_stand_pose()
        else:
            # 점프 완료
            self.current_mode = "standing"
            self.jump_start_time = None

    def perform_stretch(self, t: float):
        """스트레칭 동작"""
        if self.stretch_start_time is None:
            self.stretch_start_time = t

        elapsed = t - self.stretch_start_time
        stretch_phase = math.sin(elapsed * 2.0)

        if elapsed > 3.0:
            # 스트레칭 완료
            self.current_mode = "standing"
            self.stretch_start_time = None
            self.set_stand_pose()
            return

        # 앞뒤로 늘이기
        for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
            if leg_name in ("fl", "fr"):
                # 앞다리
                pitch = 0.3 + stretch_phase * 0.3
            else:
                # 뒷다리
                pitch = 0.8 - stretch_phase * 0.3

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
                targetPosition=pitch,
                force=self.max_torque,
            )
            p.setJointMotorControl2(
                self.robot_id,
                knee,
                controlMode=p.POSITION_CONTROL,
                targetPosition=-1.0,
                force=self.max_torque,
            )

    def perform_wave(self, t: float):
        """손 흔들기 동작"""
        if self.wave_start_time is None:
            self.wave_start_time = t

        elapsed = t - self.wave_start_time
        wave_phase = math.sin(elapsed * 8.0)  # 빠르게 흔들기

        if elapsed > 2.0:
            # 흔들기 완료
            self.current_mode = "standing"
            self.wave_start_time = None
            self.set_stand_pose()
            return

        # 왼쪽 앞다리 흔들기
        for leg_name, (hip_roll, hip_pitch, knee) in self.legs.items():
            if leg_name == "fl":
                # 왼쪽 앞다리만 움직임
                p.setJointMotorControl2(
                    self.robot_id,
                    hip_roll,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=wave_phase * 0.3,
                    force=self.max_torque,
                )
                p.setJointMotorControl2(
                    self.robot_id,
                    hip_pitch,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=-0.2,
                    force=self.max_torque,
                )
                p.setJointMotorControl2(
                    self.robot_id,
                    knee,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=-0.8 + wave_phase * 0.3,
                    force=self.max_torque,
                )
            else:
                # 나머지 다리는 서있는 자세
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

    def apply_gait(self, vx_cmd: float, yaw_cmd: float, t: float, is_running: bool = False):
        """보행 제어 (걷기/뛰기)"""
        # 명령 범위 클램핑
        vx = max(-1.0, min(1.0, vx_cmd))
        yaw = max(-1.0, min(1.0, yaw_cmd))

        # 기본 관절 각
        base_hip_pitch = self.stand_hip_pitch
        base_knee = self.stand_knee

        # 보행 파라미터
        if is_running:
            freq = 4.0 + 2.0 * abs(vx)   # 뛰기는 더 빠름
            step_amp = 0.6 * vx
            lift_amp = 0.35
        else:
            freq = 2.5 + 1.5 * abs(vx)
            step_amp = 0.4 * vx
            lift_amp = 0.2

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

        # 단순한 바디 이동
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        yaw_euler = p.getEulerFromQuaternion(base_orn)[2]

        # 뛰기는 이동 속도가 더 빠름
        v_scale = 0.005 if is_running else 0.003

        new_x = base_pos[0] + math.cos(yaw_euler) * vx * v_scale
        new_y = base_pos[1] + math.sin(yaw_euler) * vx * v_scale
        new_yaw = yaw_euler + yaw * 0.01

        new_orn = p.getQuaternionFromEuler([0, 0, new_yaw])
        p.resetBasePositionAndOrientation(self.robot_id, [new_x, new_y, base_pos[2]], new_orn)

    def update_trajectory_map(self):
        """베이스 위치를 기록하고 바닥에 선으로 궤적 표시"""
        base_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        x, y, z = base_pos

        current_point = (x, y, 0.01)
        self.traj_points.append(current_point)

        if self.last_traj_draw_pos is not None:
            p.addUserDebugLine(
                self.last_traj_draw_pos,
                current_point,
                [1, 0, 0],
                lineWidth=1,
                lifeTime=0
            )
        self.last_traj_draw_pos = current_point

    def get_camera_image(self, width=128, height=128):
        """로봇에 붙어있는 카메라에서 RGB/Depth 한 프레임 가져오기"""
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
        return img

    def shutdown(self):
        p.disconnect(self.physics_client)


def run_simulation(urdf_path: str = "go2_like.urdf"):
    robot = Go2LikeRobot(urdf_path)
    robot.set_stand_pose()

    print("=== Go2 Robot Simulator ===")
    print("Controls:")
    print("  WASD     - Move forward/back/left/right")
    print("  Q/E      - Rotate left/right")
    print("  SPACE    - Jump")
    print("  R        - Stretch")
    print("  T        - Wave")
    print("  Z        - Sit")
    print("  X        - Stand")
    print("  C        - Lie down")
    print("  ESC      - Quit")
    print()

    sim_start_real = time.time()

    try:
        while True:
            sim_time = time.time() - sim_start_real
            keys = p.getKeyboardEvents()

            vx_cmd = 0.0
            yaw_cmd = 0.0
            is_running = False

            # ESC 키 처리 (키코드 27)
            if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
                print("[INFO] Quit requested.")
                break

            # 동작 모드 전환 (한 번만 트리거되도록)
            if ord('z') in keys and keys[ord('z')] & p.KEY_WAS_TRIGGERED:
                robot.current_mode = "sitting"
                robot.set_sit_pose()
                print("[INFO] Mode: Sitting")

            if ord('x') in keys and keys[ord('x')] & p.KEY_WAS_TRIGGERED:
                robot.current_mode = "standing"
                robot.set_stand_pose()
                print("[INFO] Mode: Standing")

            if ord('c') in keys and keys[ord('c')] & p.KEY_WAS_TRIGGERED:
                robot.current_mode = "lying"
                robot.set_lie_pose()
                print("[INFO] Mode: Lying down")

            if ord(' ') in keys and keys[ord(' ')] & p.KEY_WAS_TRIGGERED:
                if robot.current_mode not in ["jumping"]:
                    robot.current_mode = "jumping"
                    robot.jump_start_time = None
                    print("[INFO] Mode: Jumping")

            if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
                if robot.current_mode not in ["stretching"]:
                    robot.current_mode = "stretching"
                    robot.stretch_start_time = None
                    print("[INFO] Mode: Stretching")

            if ord('t') in keys and keys[ord('t')] & p.KEY_WAS_TRIGGERED:
                if robot.current_mode not in ["waving"]:
                    robot.current_mode = "waving"
                    robot.wave_start_time = None
                    print("[INFO] Mode: Waving")

            # WASD 이동 (지속적으로 누르고 있을 때)
            if robot.current_mode in ["standing", "walking", "running"]:
                # Shift 키로 달리기
                if p.B3G_SHIFT in keys and keys[p.B3G_SHIFT] & p.KEY_IS_DOWN:
                    is_running = True

                if ord('w') in keys and keys[ord('w')] & p.KEY_IS_DOWN:
                    vx_cmd += 1.0
                    robot.current_mode = "running" if is_running else "walking"

                if ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
                    vx_cmd -= 1.0
                    robot.current_mode = "running" if is_running else "walking"

                if ord('a') in keys and keys[ord('a')] & p.KEY_IS_DOWN:
                    yaw_cmd += 1.0
                    robot.current_mode = "running" if is_running else "walking"

                if ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
                    yaw_cmd -= 1.0
                    robot.current_mode = "running" if is_running else "walking"

                if ord('q') in keys and keys[ord('q')] & p.KEY_IS_DOWN:
                    yaw_cmd += 1.0

                if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
                    yaw_cmd -= 1.0

                # 움직임이 없으면 서있기로 전환
                if vx_cmd == 0 and yaw_cmd == 0:
                    robot.current_mode = "standing"

            # 현재 모드에 따라 동작 수행
            if robot.current_mode == "jumping":
                robot.perform_jump(sim_time)
            elif robot.current_mode == "stretching":
                robot.perform_stretch(sim_time)
            elif robot.current_mode == "waving":
                robot.perform_wave(sim_time)
            elif robot.current_mode in ["walking", "running"]:
                robot.apply_gait(vx_cmd, yaw_cmd, sim_time, is_running)
                robot.update_trajectory_map()
            elif robot.current_mode == "standing":
                robot.set_stand_pose()
            # sitting과 lying은 이미 자세가 설정됨

            # 상태 표시 업데이트
            robot.update_status_display(robot.current_mode, vx_cmd, yaw_cmd)

            # 시뮬레이션 스텝
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    finally:
        robot.shutdown()


if __name__ == "__main__":
    run_simulation()
