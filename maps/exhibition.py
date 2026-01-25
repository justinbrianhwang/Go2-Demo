"""Map 5: Exhibition Hall - 대형 실내 공간 (전시장)"""
import pybullet as p
from .map_loader import BaseMap


class ExhibitionHallMap(BaseMap):
    """
    전시장 맵
    - 넓은 실내 공간
    - 전시 부스
    - 동상과 작품들
    - 안내 데스크

    실험 포인트:
    - GPS 없이 순수 센서 항법
    - 군중 회피 (시뮬레이션)
    - 동적 장애물
    - 음향/시각 간섭
    """

    def __init__(self):
        super().__init__(
            name="Map 5: Exhibition Hall",
            description="전시장 - 실내 순수 센서 항법 검증"
        )

    def build(self):
        """Build exhibition hall environment"""
        objects = []

        # 1. Ground plane (대리석 바닥)
        ground = self.create_ground_plane()
        p.changeVisualShape(ground, -1, rgbaColor=[0.85, 0.85, 0.9, 1])
        objects.append(ground)

        # 2. 외벽 (큰 홀)
        wall_color = [0.9, 0.9, 0.95, 1]
        walls = [
            # 북쪽 벽
            ([0.3, 12, 2], [12, 0, 2]),
            # 남쪽 벽
            ([0.3, 12, 2], [-12, 0, 2]),
            # 동쪽 벽
            ([12, 0.3, 2], [0, 12, 2]),
            # 서쪽 벽
            ([12, 0.3, 2], [0, -12, 2]),
        ]
        for size, pos in walls:
            wall = self.create_box(size, pos, color=wall_color)
            objects.append(wall)

        # 3. 전시 부스 (정사각형 배치)
        booth_color = [0.7, 0.7, 0.8, 1]
        booth_positions = [
            # 첫 번째 행
            [-8, -8], [-4, -8], [0, -8], [4, -8], [8, -8],
            # 두 번째 행
            [-8, -4], [-4, -4], [4, -4], [8, -4],
            # 세 번째 행 (중앙 비움)
            [-8, 0], [8, 0],
            # 네 번째 행
            [-8, 4], [-4, 4], [4, 4], [8, 4],
            # 다섯 번째 행
            [-8, 8], [-4, 8], [0, 8], [4, 8], [8, 8],
        ]

        for pos in booth_positions:
            # 부스 벽 (3면)
            wall1 = self.create_box([0.05, 1.5, 1.0], [pos[0] - 0.75, pos[1], 1.0], color=booth_color)
            wall2 = self.create_box([0.05, 1.5, 1.0], [pos[0] + 0.75, pos[1], 1.0], color=booth_color)
            wall3 = self.create_box([0.75, 0.05, 1.0], [pos[0], pos[1] - 0.75, 1.0], color=booth_color)
            # 부스 테이블
            table = self.create_box([0.6, 0.6, 0.03], [pos[0], pos[1], 0.7], color=[0.5, 0.4, 0.3, 1])
            objects.extend([wall1, wall2, wall3, table])

            # 전시품 (다양한 색상)
            exhibit_colors = [
                [0.8, 0.2, 0.2, 1],
                [0.2, 0.2, 0.8, 1],
                [0.2, 0.8, 0.2, 1],
                [0.8, 0.8, 0.2, 1],
            ]
            color = exhibit_colors[len(objects) % len(exhibit_colors)]
            exhibit = self.create_box([0.15, 0.15, 0.2], [pos[0], pos[1], 0.85], color=color)
            objects.append(exhibit)

        # 4. 중앙 메인 전시물 (큰 조각상)
        statue_color = [0.7, 0.6, 0.5, 1]
        # 받침대
        pedestal = self.create_cylinder(0.8, 0.3, [0, 0, 0.15], color=[0.6, 0.6, 0.6, 1])
        objects.append(pedestal)
        # 조각상 (구와 원기둥 조합)
        statue_base = self.create_cylinder(0.5, 1.5, [0, 0, 1.05], color=statue_color)
        statue_head = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
        statue_head_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.4, rgbaColor=statue_color)
        statue_head_obj = p.createMultiBody(0, statue_head, statue_head_vis, [0, 0, 2.1])
        objects.extend([statue_base, statue_head_obj])

        # 5. 안내 데스크 (입구 근처)
        desk_color = [0.4, 0.3, 0.25, 1]
        desk_pos = [-10, -10, 0.5]
        # 데스크 카운터
        counter = self.create_box([1.5, 0.5, 0.5], desk_pos, color=desk_color)
        # 의자
        chair_pos = [-10, -9, 0.3]
        chair = self.create_box([0.3, 0.3, 0.3], chair_pos, color=[0.3, 0.3, 0.3, 1])
        objects.extend([counter, chair])

        # 6. 기둥 (천장 지지)
        column_color = [0.8, 0.8, 0.85, 1]
        columns = [
            [-10, -10, 2.5],
            [-10, 10, 2.5],
            [10, -10, 2.5],
            [10, 10, 2.5],
            [-6, 0, 2.5],
            [6, 0, 2.5],
        ]
        for pos in columns:
            column = self.create_cylinder(0.3, 5.0, pos, color=column_color)
            objects.append(column)

        # 7. 천장 조명
        light_color = [1.0, 1.0, 0.95, 1]
        for x in range(-10, 11, 4):
            for y in range(-10, 11, 4):
                light = self.create_cylinder(0.3, 0.1, [x, y, 4.8], color=light_color)
                objects.append(light)

        # 8. 벤치 (관람객 휴식 공간)
        bench_color = [0.5, 0.4, 0.3, 1]
        benches = [
            [-10, 0, 0.3],
            [10, 0, 0.3],
            [0, -10, 0.3],
            [0, 10, 0.3],
        ]
        for pos in benches:
            bench = self.create_box([0.8, 0.3, 0.15], pos, color=bench_color)
            objects.append(bench)

        # 9. 안내판
        sign_color = [0.3, 0.3, 0.6, 1]
        signs = [
            [-9, -8, 1.5],
            [9, -8, 1.5],
            [-9, 8, 1.5],
            [9, 8, 1.5],
        ]
        for pos in signs:
            # 기둥
            pole = self.create_cylinder(0.05, 1.5, [pos[0], pos[1], 0.75], color=[0.5, 0.5, 0.5, 1])
            # 안내판
            sign = self.create_box([0.5, 0.03, 0.4], pos, color=sign_color)
            objects.extend([pole, sign])

        # 10. 화분과 식물 (장식)
        pot_color = [0.4, 0.3, 0.2, 1]
        plant_color = [0.2, 0.6, 0.2, 1]
        plants = [
            [-11, -6, 0.3],
            [-11, 6, 0.3],
            [11, -6, 0.3],
            [11, 6, 0.3],
        ]
        for pos in plants:
            # 화분
            pot = self.create_cylinder(0.25, 0.3, pos, color=pot_color)
            objects.append(pot)
            # 식물 (녹색 구)
            plant_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.3)
            plant_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.3, rgbaColor=plant_color)
            plant = p.createMultiBody(0, plant_shape, plant_vis, [pos[0], pos[1], pos[2] + 0.4])
            objects.append(plant)

        # 11. 소화기
        fire_color = [0.9, 0.1, 0.1, 1]
        fire_boxes = [
            [-11.5, -10, 0.5],
            [-11.5, 0, 0.5],
            [-11.5, 10, 0.5],
            [11.5, -10, 0.5],
            [11.5, 0, 0.5],
            [11.5, 10, 0.5],
        ]
        for pos in fire_boxes:
            box = self.create_cylinder(0.1, 0.5, pos, color=fire_color)
            objects.append(box)

        return objects
