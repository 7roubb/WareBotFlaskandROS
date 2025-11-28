"""
Robot repository with specialized queries
"""
from typing import List, Dict, Any, Optional
from . import BaseRepository


class RobotRepository(BaseRepository):
    """Repository for robot data access"""
    
    def __init__(self):
        super().__init__("robots")
    
    def find_by_robot_id(self, robot_id: str) -> Optional[Dict[str, Any]]:
        """Find robot by robot_id field"""
        return self.find_one({"robot_id": robot_id})
    
    def find_active_robots(self) -> List[Dict[str, Any]]:
        """Get all robots that are not offline"""
        query = {"deleted": False, "status": {"$ne": "OFFLINE"}}
        return list(self.collection.find(query).sort("created_at", -1))
    
    def find_robots_by_status(self, status: str) -> List[Dict[str, Any]]:
        """Get robots with specific status"""
        query = {"deleted": False, "status": status}
        return list(self.collection.find(query))
    
    def find_available_robots(self) -> List[Dict[str, Any]]:
        """Get robots available for task assignment"""
        query = {
            "deleted": False,
            "available": True,
            "status": {"$in": ["IDLE", "MOVING"]}
        }
        return list(self.collection.find(query))
    
    def find_by_current_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        """Find robot by current task"""
        return self.find_one({"current_task_id": task_id})
    
    def get_robot_stats(self) -> Dict[str, Any]:
        """Get statistics about all robots"""
        stats = list(self.collection.aggregate([
            {"$match": {"deleted": False}},
            {"$group": {
                "_id": None,
                "total": {"$sum": 1},
                "offline": {"$sum": {"$cond": [{"$eq": ["$status", "OFFLINE"]}, 1, 0]}},
                "idle": {"$sum": {"$cond": [{"$eq": ["$status", "IDLE"]}, 1, 0]}},
                "moving": {"$sum": {"$cond": [{"$eq": ["$status", "MOVING"]}, 1, 0]}},
                "avg_battery": {"$avg": "$battery_level"},
                "avg_cpu": {"$avg": "$cpu_usage"},
                "avg_ram": {"$avg": "$ram_usage"},
                "avg_temp": {"$avg": "$temperature"},
            }}
        ]))
        
        return stats[0] if stats else {
            "total": 0,
            "offline": 0,
            "idle": 0,
            "moving": 0,
            "avg_battery": None,
            "avg_cpu": None,
            "avg_ram": None,
            "avg_temp": None,
        }
    
    def find_robots_in_area(self, x_min: float, x_max: float, y_min: float, y_max: float) -> List[Dict[str, Any]]:
        """Find robots within a rectangular area"""
        query = {
            "deleted": False,
            "x": {"$gte": x_min, "$lte": x_max},
            "y": {"$gte": y_min, "$lte": y_max},
        }
        return list(self.collection.find(query))
    
    def get_low_battery_robots(self, threshold: float = 20.0) -> List[Dict[str, Any]]:
        """Get robots with low battery"""
        query = {
            "deleted": False,
            "battery_level": {"$lt": threshold}
        }
        return list(self.collection.find(query))
