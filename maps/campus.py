"""Map 2: Campus - 대학교 캠퍼스"""
import pybullet as p
import math
from .map_loader import BaseMap


class CampusMap(BaseMap):
    """
    대학교 캠퍼스 맵
    - 계단, 램프
    - 건물 벽
    - 보도
    - 벤치와 자전거 거치대

    실험 포인트:
    - 지도 기반 자율주행
    - 계단 접근 판단
    - 복잡한 경로 계획
    """

    def __init__(self):
        super().__init__(
            name="Map 2: Campus",
            description="캠퍼스 환경 - 계단·램프·보도 혼재"
        )

    def build(self):
        """Build campus environment"""
        objects = []

        # 1. Ground plane
        ground = self.create_ground_plane()
        p.changeVisualShape(ground, -1, rgbaColor=[0.65, 0.65, 0.65, 1])
        objects.append(ground)

        # 2. 건물 벽 (회색)
        wall_color = [0.7, 0.7, 0.8, 1]
        walls = [
            # 왼쪽 건물
            ([0.2, 4, 1.5], [-6, 0, 1.5]),
            # 오른쪽 건물
            ([0.2, 3, 1.5], [6, 2, 1.5]),
            # 뒤쪽 건물
            ([3, 0.2, 1.5], [0, -6, 1.5]),
        ]
        for size, pos in walls:
            wall = self.create_box(size, pos, color=wall_color)
            objects.append(wall)

        # 3. 계단 (5단)
        stair_color = [0.6, 0.6, 0.7, 1]
        stair_width = 1.5
        stair_depth = 0.3
        stair_height = 0.1
        for i in range(5):
            stair_pos = [3, 0, 0.05 + i * stair_height]
            stair_size = [stair_depth, stair_width, stair_height / 2]
            stair = self.create_box(stair_size, stair_pos, color=stair_color)
            objects.append(stair)

        # 4. 램프 (경사로)
        ramp_color = [0.65, 0.65, 0.75, 1]
        ramp_angle = math.radians(15)  # 15도 경사
        ramp = self.create_box(
            [1.5, 1.0, 0.1],
            [0, 3, 0.3],
            orientation=[0, ramp_angle, 0],
            color=ramp_color
        )
        objects.append(ramp)

        # 5. 보도 (밝은 회색)
        path_color = [0.8, 0.8, 0.8, 1]
        paths = [
            # 메인 통로
            ([6, 0.8, 0.02], [0, 0, 0.02]),
            # 교차 통로
            ([0.8, 4, 0.02], [0, 0, 0.02]),
        ]
        for size, pos in paths:
            path = self.create_box(size, pos, color=path_color)
            objects.append(path)

        # 6. 벤치 (캠퍼스 스타일)
        bench_color = [0.3, 0.2, 0.15, 1]
        benches = [
            ([0.8, 0.15, 0.05], [2, 2, 0.3]),
            ([0.8, 0.15, 0.05], [-2, 2, 0.3]),
            ([0.8, 0.15, 0.05], [2, -2, 0.3]),
        ]
        for size, pos in benches:
            bench = self.create_box(size, pos, color=bench_color)
            objects.append(bench)

        # 7. 자전거 거치대 (금속 느낌)
        rack_color = [0.5, 0.5, 0.5, 1]
        racks = [
            [4, 2, 0.3],
            [4, -2, 0.3],
        ]
        for pos in racks:
            # 수평 바
            rack1 = self.create_box([0.05, 1.5, 0.05], pos, color=rack_color)
            # 수직 지지대
            rack2_pos = [pos[0], pos[1] - 0.7, 0.15]
            rack2 = self.create_box([0.05, 0.05, 0.15], rack2_pos, color=rack_color)
            rack3_pos = [pos[0], pos[1] + 0.7, 0.15]
            rack3 = self.create_box([0.05, 0.05, 0.15], rack3_pos, color=rack_color)
            objects.extend([rack1, rack2, rack3])

        # 8. 가로수 (캠퍼스 조경)
        trunk_color = [0.3, 0.15, 0.05, 1]
        leaves_color = [0.15, 0.6, 0.15, 1]
        trees = [
            [-4, -3, 0],
            [-4, 3, 0],
            [5, -3, 0],
        ]
        for pos in trees:
            trunk = self.create_cylinder(0.12, 1.0, [pos[0], pos[1], 0.5], color=trunk_color)
            leaves_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.5)
            leaves_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.5, rgbaColor=leaves_color)
            leaves = p.createMultiBody(0, leaves_shape, leaves_vis, [pos[0], pos[1], 1.2])
            objects.extend([trunk, leaves])

        # 9. 표지판
        sign_color = [0.2, 0.3, 0.6, 1]
        signs = [
            [1, 4, 1.2],
            [-3, -2, 1.2],
        ]
        for pos in signs:
            # 기둥
            pole = self.create_cylinder(0.04, 1.2, [pos[0], pos[1], 0.6], color=[0.5, 0.5, 0.5, 1])
            # 표지판 판
            sign_board = self.create_box([0.3, 0.02, 0.2], pos, color=sign_color)
            objects.extend([pole, sign_board])

        return objects
