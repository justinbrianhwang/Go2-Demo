"""Map 3: Warehouse - 물류 창고"""
import pybullet as p
from .map_loader import BaseMap


class WarehouseMap(BaseMap):
    """
    물류 창고 맵
    - 반복적인 선반 구조
    - 넓은 시야
    - 팔레트와 박스
    - 명확한 통로

    실험 포인트:
    - LiDAR 기반 장애물 회피
    - 경로 계획
    - 반복 경로 신뢰성
    - SLAM
    """

    def __init__(self):
        super().__init__(
            name="Map 3: Warehouse",
            description="물류 창고 - SLAM·경로계획 검증"
        )

    def build(self):
        """Build warehouse environment"""
        objects = []

        # 1. Ground plane (콘크리트 바닥)
        ground = self.create_ground_plane()
        p.changeVisualShape(ground, -1, rgbaColor=[0.55, 0.55, 0.55, 1])
        objects.append(ground)

        # 2. 선반 구조 (격자 배치)
        shelf_color = [0.6, 0.5, 0.3, 1]
        box_color = [0.7, 0.6, 0.4, 1]

        # 선반 3열 x 4행
        for row in range(4):
            for col in range(3):
                x = -4 + col * 3
                y = -6 + row * 4

                # 선반 프레임 (수직 기둥)
                for corner in [(-0.6, -0.4), (0.6, -0.4), (-0.6, 0.4), (0.6, 0.4)]:
                    pole_pos = [x + corner[0], y + corner[1], 0.8]
                    pole = self.create_cylinder(0.05, 1.6, pole_pos, color=shelf_color)
                    objects.append(pole)

                # 선반 층 (3층)
                for level in [0.5, 1.0, 1.5]:
                    shelf_pos = [x, y, level]
                    shelf = self.create_box([0.6, 0.4, 0.02], shelf_pos, color=shelf_color)
                    objects.append(shelf)

                    # 박스들 (일부 선반에만)
                    if (row + col) % 2 == 0:
                        box_pos = [x, y, level + 0.15]
                        box = self.create_box([0.2, 0.2, 0.15], box_pos, color=box_color)
                        objects.append(box)

        # 3. 통로 표시선 (노란색)
        lane_color = [0.9, 0.8, 0.1, 1]
        lanes = [
            # 세로 통로
            ([8, 0.05, 0.01], [-4, 0, 0.01]),
            ([8, 0.05, 0.01], [-1, 0, 0.01]),
            ([8, 0.05, 0.01], [2, 0, 0.01]),
            ([8, 0.05, 0.01], [5, 0, 0.01]),
            # 가로 통로
            ([0.05, 12, 0.01], [0, 0, 0.01]),
        ]
        for size, pos in lanes:
            lane = self.create_box(size, pos, color=lane_color)
            objects.append(lane)

        # 4. 팔레트 (목재 팔레트)
        pallet_color = [0.4, 0.3, 0.2, 1]
        pallets = [
            [6, 6, 0.08],
            [6, -6, 0.08],
            [-6, 6, 0.08],
            [-6, -6, 0.08],
        ]
        for pos in pallets:
            # 팔레트 베이스
            pallet = self.create_box([0.6, 0.6, 0.08], pos, color=pallet_color)
            objects.append(pallet)
            # 팔레트 위 박스 스택
            for i in range(3):
                box_pos = [pos[0], pos[1], pos[2] + 0.08 + (i * 0.3) + 0.15]
                box = self.create_box([0.5, 0.5, 0.15], box_pos, color=box_color)
                objects.append(box)

        # 5. 리프트 (지게차 대용)
        lift_color = [0.9, 0.6, 0.1, 1]
        lifts = [
            [7, 4, 0.3],
            [-7, -4, 0.3],
        ]
        for pos in lifts:
            # 본체
            body = self.create_box([0.4, 0.3, 0.3], pos, color=lift_color)
            # 포크
            fork1_pos = [pos[0] + 0.6, pos[1] - 0.1, 0.15]
            fork1 = self.create_box([0.3, 0.05, 0.05], fork1_pos, color=[0.7, 0.7, 0.7, 1])
            fork2_pos = [pos[0] + 0.6, pos[1] + 0.1, 0.15]
            fork2 = self.create_box([0.3, 0.05, 0.05], fork2_pos, color=[0.7, 0.7, 0.7, 1])
            objects.extend([body, fork1, fork2])

        # 6. 기둥 (건물 지지대)
        column_color = [0.5, 0.5, 0.5, 1]
        columns = [
            [-8, -8, 2.0],
            [-8, 8, 2.0],
            [8, -8, 2.0],
            [8, 8, 2.0],
        ]
        for pos in columns:
            column = self.create_cylinder(0.2, 4.0, pos, color=column_color)
            objects.append(column)

        # 7. 안전 콘 (주황색)
        cone_color = [1.0, 0.5, 0.0, 1]
        cones = [
            [3, 3, 0.2],
            [3, -3, 0.2],
            [-3, 3, 0.2],
            [-3, -3, 0.2],
        ]
        for pos in cones:
            cone_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.15, height=0.4)
            cone_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.15, length=0.4, rgbaColor=cone_color)
            cone = p.createMultiBody(0, cone_shape, cone_vis, pos)
            objects.append(cone)

        return objects
