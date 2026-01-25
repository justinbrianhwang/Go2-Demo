"""Map 4: Parking Lot - 주차장"""
import pybullet as p
import math
from .map_loader import BaseMap


class ParkingLotMap(BaseMap):
    """
    주차장 맵
    - 주차 공간 구획
    - 경사로 (램프)
    - 기둥
    - 자동차 장애물

    실험 포인트:
    - 경사 대응
    - 저조도 비전 (시뮬레이션)
    - 반사체 처리
    - 좁은 공간 회피
    """

    def __init__(self):
        super().__init__(
            name="Map 4: Parking Lot",
            description="주차장 - 경사·기둥·조도 변화"
        )

    def build(self):
        """Build parking lot environment"""
        objects = []

        # 1. Ground plane (아스팔트)
        ground = self.create_ground_plane()
        p.changeVisualShape(ground, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
        objects.append(ground)

        # 2. 주차 구획선 (흰색)
        line_color = [0.9, 0.9, 0.9, 1]

        # 세로 구획선
        for i in range(6):
            x = -6 + i * 2.5
            line = self.create_box([0.05, 4, 0.01], [x, 0, 0.01], color=line_color)
            objects.append(line)

        # 가로 구획선
        for i in range(5):
            y = -4 + i * 2
            line = self.create_box([6, 0.05, 0.01], [0, y, 0.01], color=line_color)
            objects.append(line)

        # 3. 주차된 차량들 (단순화된 박스 형태)
        car_colors = [
            [0.8, 0.1, 0.1, 1],  # 빨강
            [0.1, 0.1, 0.8, 1],  # 파랑
            [0.1, 0.7, 0.1, 1],  # 녹색
            [0.9, 0.9, 0.9, 1],  # 흰색
            [0.1, 0.1, 0.1, 1],  # 검정
        ]

        parking_spots = [
            [-5, -3, 0.4],
            [-2.5, -3, 0.4],
            [2.5, -3, 0.4],
            [-5, -1, 0.4],
            [0, -1, 0.4],
            [5, -1, 0.4],
            [-2.5, 1, 0.4],
            [2.5, 1, 0.4],
            [5, 1, 0.4],
            [-5, 3, 0.4],
            [0, 3, 0.4],
        ]

        for i, pos in enumerate(parking_spots):
            color = car_colors[i % len(car_colors)]
            # 차체
            car = self.create_box([1.0, 0.45, 0.35], pos, color=color)
            # 루프 (더 작은 박스)
            roof_pos = [pos[0], pos[1], pos[2] + 0.4]
            roof = self.create_box([0.8, 0.4, 0.15], roof_pos, color=color)
            objects.extend([car, roof])

        # 4. 기둥 (구조 기둥)
        column_color = [0.6, 0.6, 0.6, 1]
        columns = [
            [-7, -5, 1.5],
            [-7, 0, 1.5],
            [-7, 5, 1.5],
            [7, -5, 1.5],
            [7, 0, 1.5],
            [7, 5, 1.5],
        ]
        for pos in columns:
            column = self.create_cylinder(0.25, 3.0, pos, color=column_color)
            objects.append(column)

        # 5. 경사로 (지하 주차장 진입로)
        ramp_color = [0.35, 0.35, 0.35, 1]
        ramp_angle = math.radians(12)  # 12도 경사
        ramp = self.create_box(
            [2.0, 1.5, 0.1],
            [-8, -6, 0.5],
            orientation=[0, ramp_angle, 0],
            color=ramp_color
        )
        objects.append(ramp)

        # 6. 속도 방지턱
        bump_color = [0.9, 0.8, 0.1, 1]
        bumps = [
            [0, -5, 0.05],
            [0, 5, 0.05],
        ]
        for pos in bumps:
            bump = self.create_cylinder(0.1, 6.0, pos, orientation=[0, 0, math.pi/2], color=bump_color)
            objects.append(bump)

        # 7. 표지판
        sign_color = [0.2, 0.4, 0.8, 1]
        signs = [
            [6, -4, 1.5],
            [6, 4, 1.5],
            [-6, 0, 1.5],
        ]
        for pos in signs:
            # 기둥
            pole = self.create_cylinder(0.05, 1.5, [pos[0], pos[1], 0.75], color=[0.5, 0.5, 0.5, 1])
            # 표지판
            sign = self.create_box([0.4, 0.03, 0.3], pos, color=sign_color)
            objects.extend([pole, sign])

        # 8. 조명 (천장 조명 시뮬레이션)
        light_color = [0.95, 0.95, 0.8, 1]
        lights = [
            [0, -3, 2.8],
            [0, 0, 2.8],
            [0, 3, 2.8],
            [-4, -3, 2.8],
            [-4, 3, 2.8],
            [4, -3, 2.8],
            [4, 3, 2.8],
        ]
        for pos in lights:
            light = self.create_cylinder(0.2, 0.1, pos, color=light_color)
            objects.append(light)

        # 9. 소화기 박스 (빨간색)
        fire_color = [0.9, 0.1, 0.1, 1]
        fire_boxes = [
            [-7, -2.5, 0.5],
            [7, -2.5, 0.5],
            [-7, 2.5, 0.5],
            [7, 2.5, 0.5],
        ]
        for pos in fire_boxes:
            box = self.create_box([0.15, 0.1, 0.3], pos, color=fire_color)
            objects.append(box)

        # 10. 쓰레기통
        bin_color = [0.2, 0.5, 0.2, 1]
        bins = [
            [6, -5.5, 0.35],
            [6, 5.5, 0.35],
        ]
        for pos in bins:
            bin_obj = self.create_cylinder(0.2, 0.7, pos, color=bin_color)
            objects.append(bin_obj)

        return objects
