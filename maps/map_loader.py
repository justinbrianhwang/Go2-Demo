"""Map loader and manager"""
import pybullet as p


class MapLoader:
    """Manages different map environments"""

    def __init__(self):
        self.current_map = None
        self.current_map_index = 0
        self.maps = []
        self.map_objects = []  # Store loaded object IDs

    def load_map(self, map_instance):
        """Load a specific map"""
        # Clear existing map
        self.clear_map()

        # Load new map
        self.current_map = map_instance
        self.map_objects = map_instance.build()

        return self.map_objects

    def clear_map(self):
        """Clear all map objects"""
        for obj_id in self.map_objects:
            try:
                p.removeBody(obj_id)
            except:
                pass
        self.map_objects = []

    def register_maps(self, maps_list):
        """Register available maps"""
        self.maps = maps_list
        if len(maps_list) > 0:
            self.load_map(maps_list[0])

    def next_map(self):
        """Switch to next map"""
        if len(self.maps) == 0:
            return None

        self.current_map_index = (self.current_map_index + 1) % len(self.maps)
        self.load_map(self.maps[self.current_map_index])
        return self.maps[self.current_map_index]

    def get_map_name(self):
        """Get current map name"""
        if self.current_map:
            return self.current_map.name
        return "No map loaded"


class BaseMap:
    """Base class for all maps"""

    def __init__(self, name, description):
        self.name = name
        self.description = description

    def build(self):
        """Build the map and return list of object IDs"""
        raise NotImplementedError("Subclasses must implement build()")

    def create_ground_plane(self):
        """Create basic ground plane"""
        plane_id = p.createCollisionShape(p.GEOM_PLANE)
        ground_id = p.createMultiBody(0, plane_id)
        p.changeVisualShape(ground_id, -1, rgbaColor=[0.7, 0.7, 0.7, 1])
        return ground_id

    def create_box(self, half_extents, position, orientation=[0, 0, 0], mass=0, color=[0.5, 0.5, 0.5, 1]):
        """Create a box obstacle"""
        col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)

        orn_quat = p.getQuaternionFromEuler(orientation)
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orn_quat
        )
        return body_id

    def create_cylinder(self, radius, height, position, orientation=[0, 0, 0], mass=0, color=[0.5, 0.5, 0.5, 1]):
        """Create a cylindrical obstacle"""
        col_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
        vis_id = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=color)

        orn_quat = p.getQuaternionFromEuler(orientation)
        body_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orn_quat
        )
        return body_id
