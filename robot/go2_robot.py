"""Go2 Robot class definition"""
import math
import pybullet as p


class Go2LikeRobot:
    def __init__(self, urdf_path: str):
        """Initialize Go2 robot"""
        # URDF 로드
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
        self.current_mode = "standing"
        self.jump_start_time = None
        self.stretch_start_time = None
        self.wave_start_time = None

        # 간단한 궤적 맵
        self.traj_points = []
        self.last_traj_draw_pos = None

        # 자동 모드 변수
        self.auto_mode = False
        self.auto_direction = 1.0  # 1.0 = 전진
        self.auto_turn_direction = 0.0
        self.auto_obstacle_cooldown = 0.0
        self.auto_last_decision_time = 0.0

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
            freq = 4.0 + 2.0 * abs(vx)
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

    def detect_obstacles(self):
        """
        Ray casting을 사용하여 장애물 감지
        Returns: (전방 거리, 좌측 거리, 우측 거리, 상단 거리)
        """
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot_id)
        x, y, z = base_pos

        # 로봇의 전방 방향 계산
        rot_mat = p.getMatrixFromQuaternion(base_orn)
        forward = [rot_mat[0], rot_mat[3], rot_mat[6]]
        right = [rot_mat[1], rot_mat[4], rot_mat[7]]

        # 센서 높이 (로봇 중심)
        sensor_height = z

        # Ray 시작점
        ray_from = [x, y, sensor_height]

        # 여러 방향으로 Ray casting
        ray_length = 2.0  # 2미터 전방 감지

        # 전방
        ray_to_forward = [
            x + forward[0] * ray_length,
            y + forward[1] * ray_length,
            sensor_height
        ]

        # 좌측 (45도)
        left_forward = [
            forward[0] - right[0],
            forward[1] - right[1],
            0
        ]
        left_mag = math.sqrt(left_forward[0]**2 + left_forward[1]**2)
        if left_mag > 0:
            left_forward = [left_forward[0]/left_mag, left_forward[1]/left_mag, 0]

        ray_to_left = [
            x + left_forward[0] * ray_length,
            y + left_forward[1] * ray_length,
            sensor_height
        ]

        # 우측 (45도)
        right_forward = [
            forward[0] + right[0],
            forward[1] + right[1],
            0
        ]
        right_mag = math.sqrt(right_forward[0]**2 + right_forward[1]**2)
        if right_mag > 0:
            right_forward = [right_forward[0]/right_mag, right_forward[1]/right_mag, 0]

        ray_to_right = [
            x + right_forward[0] * ray_length,
            y + right_forward[1] * ray_length,
            sensor_height
        ]

        # 상단 (점프 필요 여부 확인)
        ray_to_up = [
            x + forward[0] * 1.0,
            y + forward[1] * 1.0,
            sensor_height + 0.5
        ]

        # Ray casting 수행
        result_forward = p.rayTest(ray_from, ray_to_forward)[0]
        result_left = p.rayTest(ray_from, ray_to_left)[0]
        result_right = p.rayTest(ray_from, ray_to_right)[0]
        result_up = p.rayTest(ray_from, ray_to_up)[0]

        # 거리 계산 (hit fraction * ray_length)
        dist_forward = result_forward[2] * ray_length
        dist_left = result_left[2] * ray_length
        dist_right = result_right[2] * ray_length
        dist_up = result_up[2] * 1.0

        return dist_forward, dist_left, dist_right, dist_up

    def auto_navigate(self, t: float):
        """
        자동 네비게이션 - 장애물 회피 및 자율 이동
        Returns: (vx_cmd, yaw_cmd, should_jump)
        """
        # 장애물 감지
        dist_forward, dist_left, dist_right, dist_up = self.detect_obstacles()

        # 쿨다운 감소
        if self.auto_obstacle_cooldown > 0:
            self.auto_obstacle_cooldown -= 0.01

        vx_cmd = 0.6  # 기본 전진 속도
        yaw_cmd = 0.0
        should_jump = False

        # 결정 주기 (너무 자주 바꾸지 않도록)
        decision_interval = 1.0
        if t - self.auto_last_decision_time > decision_interval:
            self.auto_last_decision_time = t

            # 전방 장애물 감지
            if dist_forward < 0.8:  # 0.8m 이내 장애물
                # 상단 체크 - 점프 가능 여부
                if dist_up > 0.4 and dist_forward > 0.4:
                    # 낮은 장애물 - 점프!
                    should_jump = True
                    self.auto_obstacle_cooldown = 2.0
                else:
                    # 높은 장애물 또는 가까운 장애물 - 회전
                    # 좌우 중 더 넓은 쪽으로 회전
                    if dist_left > dist_right:
                        self.auto_turn_direction = 1.0  # 좌회전
                    else:
                        self.auto_turn_direction = -1.0  # 우회전

                    self.auto_obstacle_cooldown = 1.5
            elif dist_left < 0.5:
                # 왼쪽 가까이 장애물 - 우회전
                self.auto_turn_direction = -0.5
                self.auto_obstacle_cooldown = 0.8
            elif dist_right < 0.5:
                # 오른쪽 가까이 장애물 - 좌회전
                self.auto_turn_direction = 0.5
                self.auto_obstacle_cooldown = 0.8
            else:
                # 경로 clear - 직진
                self.auto_turn_direction = 0.0

        # 쿨다운 중에는 회전 적용
        if self.auto_obstacle_cooldown > 0:
            yaw_cmd = self.auto_turn_direction
            if self.auto_obstacle_cooldown > 1.0:
                vx_cmd = 0.2  # 천천히 이동
            else:
                vx_cmd = 0.4

        # 가끔 랜덤하게 방향 바꾸기 (탐험 행동)
        if t % 15.0 < 0.1 and self.auto_obstacle_cooldown <= 0:
            import random
            self.auto_turn_direction = random.choice([-0.5, 0.5])
            self.auto_obstacle_cooldown = 0.5

        return vx_cmd, yaw_cmd, should_jump
