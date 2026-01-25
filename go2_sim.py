"""
Go2 Robot Simulator - Main Entry Point
Phase 1: Basic motion control
Phase 2: Multi-environment testing with 5 different maps
Phase 3: Autonomous navigation with obstacle avoidance
"""
import os
import time
import pybullet as p
import pybullet_data

# Import robot and maps
from robot import Go2LikeRobot
from maps import (
    MapLoader,
    UrbanParkMap,
    CampusMap,
    WarehouseMap,
    ParkingLotMap,
    ExhibitionHallMap
)


def run_simulation(urdf_path: str = "go2_like.urdf"):
    """Main simulation loop"""

    # 스크립트 위치로 작업 디렉토리 변경 (한글 경로 문제 해결)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    original_dir = os.getcwd()
    os.chdir(script_dir)

    # PyBullet 연결
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # PyBullet GUI 기본 키보드 단축키 비활성화
    p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)

    # 맵 로더 초기화
    map_loader = MapLoader()
    available_maps = [
        UrbanParkMap(),
        CampusMap(),
        WarehouseMap(),
        ParkingLotMap(),
        ExhibitionHallMap(),
    ]
    map_loader.register_maps(available_maps)

    # 로봇 생성 (상대 경로 사용 - 이미 작업 디렉토리 변경됨)
    robot = Go2LikeRobot(urdf_path)
    robot.set_stand_pose()

    print("=== Go2 Robot Simulator (Phase 3) ===")
    print(f"Current Map: {map_loader.get_map_name()}")
    print("\nControls:")
    print("  WASD     - Move forward/back/left/right")
    print("  Q/E      - Rotate left/right")
    print("  Shift+W/S/A/D - Run (faster movement)")
    print("  SPACE    - Jump")
    print("  R        - Stretch")
    print("  T        - Wave")
    print("  Z        - Sit")
    print("  X        - Stand")
    print("  C        - Lie down")
    print("  P        - Toggle AUTO mode (autonomous navigation)")
    print("  M        - Switch to next map")
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

            # P 키 - 자동 모드 토글
            if ord('p') in keys and keys[ord('p')] & p.KEY_WAS_TRIGGERED:
                robot.auto_mode = not robot.auto_mode
                mode_str = "AUTO" if robot.auto_mode else "MANUAL"
                print(f"[INFO] Control mode: {mode_str}")
                if robot.auto_mode:
                    # 자동 모드 시작 - 서있는 상태로
                    robot.current_mode = "standing"
                    robot.auto_last_decision_time = sim_time

            # M 키 - 맵 전환
            if ord('m') in keys and keys[ord('m')] & p.KEY_WAS_TRIGGERED:
                next_map = map_loader.next_map()
                if next_map:
                    # 로봇 위치 리셋
                    p.resetBasePositionAndOrientation(
                        robot.robot_id,
                        [0, 0, 0.45],
                        p.getQuaternionFromEuler([0, 0, 0])
                    )
                    # 궤적 리셋
                    robot.traj_points = []
                    robot.last_traj_draw_pos = None
                    print(f"[INFO] Switched to: {map_loader.get_map_name()}")

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

            # 자동 모드 vs 수동 모드
            if robot.auto_mode and robot.current_mode in ["standing", "walking", "running"]:
                # 자동 네비게이션
                vx_cmd, yaw_cmd, should_jump = robot.auto_navigate(sim_time)

                # 자동 점프 실행
                if should_jump and robot.current_mode not in ["jumping"]:
                    robot.current_mode = "jumping"
                    robot.jump_start_time = None
                    print("[AUTO] Jumping over obstacle!")
                elif robot.current_mode not in ["jumping"]:
                    robot.current_mode = "walking"

            # WASD 이동 (수동 모드, 지속적으로 누르고 있을 때)
            elif not robot.auto_mode and robot.current_mode in ["standing", "walking", "running"]:
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

            # 시뮬레이션 스텝
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    finally:
        p.disconnect(physics_client)


if __name__ == "__main__":
    run_simulation()
