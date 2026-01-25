"""Maps module for Go2 simulator"""
from .map_loader import MapLoader
from .urban_park import UrbanParkMap
from .campus import CampusMap
from .warehouse import WarehouseMap
from .parking import ParkingLotMap
from .exhibition import ExhibitionHallMap

__all__ = [
    'MapLoader',
    'UrbanParkMap',
    'CampusMap',
    'WarehouseMap',
    'ParkingLotMap',
    'ExhibitionHallMap'
]
