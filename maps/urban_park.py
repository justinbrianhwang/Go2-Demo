"""Map 1: Urban Park - 도심 공원"""
import pybullet as p
from .map_loader import BaseMap


class UrbanParkMap(BaseMap):
    """
    도심 공원 맵
    - 평탄한 보행로
    - 잔디/보도블록 혼합 지형
    - 벤치, 가로등, 나무 장애물
    - 작은 언덕

    실험 포인트:
    - 보행 안정성
    - 지면 적응
    - 사람 회피 시뮬레이션
    - 저속 추종
    """

    def __init__(self):
        super().__init__(
            name="Map 1: Urban Park",
            description="평탄한 공원 환경 - 보행 안정성 테스트"
        )

    def build(self):
        """Build urban park environment"""
        objects = []

        # 1. Ground plane (회색 보도블록)
        ground = self.create_ground_plane()
        p.changeVisualShape(ground, -1, rgbaColor=[0.6, 0.6, 0.6, 1])
        objects.append(ground)

        # 2. 잔디 영역 (녹색 패치)
        grass_color = [0.2, 0.6, 0.2, 1]
        grass_areas = [
            ([3, 3, 0.01], [-2, -2, 0.01]),
            ([3, 3, 0.01], [2, 2, 0.01]),
            ([2, 4, 0.01], [4, -1, 0.01]),
        ]
        for size, pos in grass_areas:
            grass = self.create_box(size, pos, color=grass_color)
            objects.append(grass)

        # 3. 보도 경로 (밝은 회색)
        path_color = [0.8, 0.8, 0.8, 1]
        path1 = self.create_box([5, 0.5, 0.02], [0, 0, 0.02], color=path_color)
        path2 = self.create_box([0.5, 5, 0.02], [0, 0, 0.02], color=path_color)
        objects.extend([path1, path2])

        # 4. 벤치 (갈색 나무 벤치)
        bench_color = [0.4, 0.2, 0.1, 1]
        benches = [
            # 좌석
            ([0.4, 0.15, 0.05], [1.5, 1.5, 0.3]),
            ([0.4, 0.15, 0.05], [-1.5, -1.5, 0.3]),
            # 등받이
            ([0.4, 0.05, 0.2], [1.5, 1.7, 0.5]),
            ([0.4, 0.05, 0.2], [-1.5, -1.3, 0.5]),
        ]
        for size, pos in benches:
            bench = self.create_box(size, pos, color=bench_color)
            objects.append(bench)

        # 5. 가로등 (회색 기둥 + 노란 조명)
        pole_color = [0.3, 0.3, 0.3, 1]
        light_color = [0.9, 0.9, 0.5, 1]
        lamp_posts = [
            [3, 3, 0],
            [-3, 3, 0],
            [3, -3, 0],
            [-3, -3, 0],
        ]
        for pos in lamp_posts:
            # 기둥
            pole_pos = [pos[0], pos[1], 0.8]
            pole = self.create_cylinder(0.05, 1.6, pole_pos, color=pole_color)
            objects.append(pole)
            # 조명
            light_pos = [pos[0], pos[1], 1.7]
            light = self.create_cylinder(0.15, 0.1, light_pos, color=light_color)
            objects.append(light)

        # 6. 나무 (갈색 나무줄기 + 녹색 잎)
        trunk_color = [0.3, 0.15, 0.05, 1]
        leaves_color = [0.1, 0.5, 0.1, 1]
        trees = [
            [4, 4, 0],
            [-4, 4, 0],
            [4, -4, 0],
            [-4, -4, 0],
            [0, 5, 0],
            [5, 0, 0],
        ]
        for pos in trees:
            # 나무줄기
            trunk_pos = [pos[0], pos[1], 0.4]
            trunk = self.create_cylinder(0.1, 0.8, trunk_pos, color=trunk_color)
            objects.append(trunk)
            # 나뭇잎 (구형)
            leaves_pos = [pos[0], pos[1], 1.0]
            leaves_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.4)
            leaves_vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.4, rgbaColor=leaves_color)
            leaves = p.createMultiBody(0, leaves_shape, leaves_vis, leaves_pos)
            objects.append(leaves)

        # 7. 작은 언덕 (경사 테스트용)
        hill_color = [0.3, 0.5, 0.3, 1]
        hill = self.create_box([1.5, 1.5, 0.2], [-5, -5, 0.2], color=hill_color)
        objects.append(hill)

        # 8. 쓰레기통 (회색 원기둥)
        bin_color = [0.4, 0.4, 0.4, 1]
        bins = [
            [2, 0, 0.3],
            [0, 2, 0.3],
            [-2, 0, 0.3],
        ]
        for pos in bins:
            bin_obj = self.create_cylinder(0.15, 0.6, pos, color=bin_color)
            objects.append(bin_obj)

        return objects
