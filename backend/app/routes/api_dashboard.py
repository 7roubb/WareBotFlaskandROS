from flask import Blueprint, jsonify
from datetime import datetime
from bson import ObjectId

from ..services import (
    dashboard_top_moving_products,
    dashboard_shelf_summary,
    dashboard_daily_movements
)
from ..extensions import get_db

dashboard_bp = Blueprint("dashboard", __name__)


# =========================================================
# TOP MOVING PRODUCTS
# =========================================================
@dashboard_bp.route("/top-moving", methods=["GET"])
def dashboard_top_route():
    """
    Returns top picked products based on product_transactions
    """
    return jsonify(dashboard_top_moving_products())


# =========================================================
# SHELF SUMMARY
# =========================================================
@dashboard_bp.route("/shelves", methods=["GET"])
def dashboard_shelves_route():
    """
    Returns shelves summary:
    - shelf coords
    - total items
    - number of products
    """
    return jsonify(dashboard_shelf_summary())


# =========================================================
# DAILY MOVEMENTS
# =========================================================
@dashboard_bp.route("/daily", methods=["GET"])
def dashboard_daily_route():
    """
    Returns PICK, RETURN, ADJUST aggregated for the current day
    """
    return jsonify(dashboard_daily_movements())


# =========================================================
# LIVE TASK STATISTICS (Real-time Dashboard)
# =========================================================
@dashboard_bp.route("/live/tasks", methods=["GET"])
def dashboard_live_tasks():
    """
    Returns real-time task statistics for live dashboard
    """
    db = get_db()
    
    try:
        # Task breakdown by status
        task_statuses = {
            "PENDING": db.tasks.count_documents({"status": "PENDING"}),
            "ASSIGNED": db.tasks.count_documents({"status": "ASSIGNED"}),
            "MOVING_TO_PICKUP": db.tasks.count_documents({"status": "MOVING_TO_PICKUP"}),
            "ATTACHED": db.tasks.count_documents({"status": "ATTACHED"}),
            "MOVING_TO_DROP": db.tasks.count_documents({"status": "MOVING_TO_DROP"}),
            "COMPLETED": db.tasks.count_documents({"status": "COMPLETED"}),
            "ERROR": db.tasks.count_documents({"status": "ERROR"}),
            "CANCELLED": db.tasks.count_documents({"status": "CANCELLED"}),
        }
        
        total = sum(task_statuses.values())
        in_progress = task_statuses["ASSIGNED"] + task_statuses["MOVING_TO_PICKUP"] + task_statuses["ATTACHED"] + task_statuses["MOVING_TO_DROP"]
        
        # Recent completed tasks
        recent_completed = list(db.tasks.find({
            "status": "COMPLETED",
            "completed_at": {"$exists": True}
        }).sort("completed_at", -1).limit(10))
        
        from ..services.utils_service import serialize
        recent_completed = [serialize(t) for t in recent_completed]
        
        return jsonify({
            "statuses": task_statuses,
            "summary": {
                "total_tasks": total,
                "in_progress": in_progress,
                "completed": task_statuses["COMPLETED"],
                "failed": task_statuses["ERROR"],
                "completion_rate": round((task_statuses["COMPLETED"] / max(total, 1)) * 100, 2)
            },
            "recent_completed": recent_completed[:5],  # Last 5 for brevity
            "timestamp": datetime.utcnow().isoformat()
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# =========================================================
# LIVE ROBOT STATISTICS (Real-time Dashboard)
# =========================================================
@dashboard_bp.route("/live/robots", methods=["GET"])
def dashboard_live_robots():
    """
    Returns real-time robot status and performance metrics
    """
    db = get_db()
    
    try:
        robots = list(db.robots.find({"deleted": False}))
        
        robot_statuses = {
            "IDLE": 0,
            "BUSY": 0,
            "ERROR": 0,
            "OFFLINE": 0,
            "CHARGING": 0
        }
        
        robot_list = []
        total_battery = 0
        battery_count = 0
        
        for r in robots:
            status = r.get("status", "UNKNOWN")
            if status in robot_statuses:
                robot_statuses[status] += 1
            
            battery = r.get("battery_level")
            if battery is not None:
                total_battery += battery
                battery_count += 1
            
            robot_list.append({
                "id": str(r.get("_id")),
                "name": r.get("name"),
                "robot_id": r.get("robot_id"),
                "status": status,
                "available": r.get("available", False),
                "battery_level": battery,
                "position": {
                    "x": r.get("x"),
                    "y": r.get("y"),
                    "yaw": r.get("yaw")
                },
                "current_task_id": r.get("current_task_id"),
                "last_seen": r.get("last_seen"),
                "cpu_usage": r.get("cpu_usage"),
                "ram_usage": r.get("ram_usage"),
                "temperature": r.get("temperature")
            })
        
        avg_battery = round(total_battery / max(battery_count, 1), 2) if battery_count > 0 else 0
        
        return jsonify({
            "statuses": robot_statuses,
            "summary": {
                "total_robots": len(robots),
                "available": db.robots.count_documents({"deleted": False, "available": True}),
                "busy": db.robots.count_documents({"deleted": False, "available": False}),
                "offline": robot_statuses["OFFLINE"],
                "average_battery_level": avg_battery
            },
            "robots": robot_list,
            "timestamp": datetime.utcnow().isoformat()
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# =========================================================
# LIVE OVERALL SYSTEM STATS
# =========================================================
@dashboard_bp.route("/live/system", methods=["GET"])
def dashboard_live_system():
    """
    Returns overall system health and performance metrics
    """
    db = get_db()
    
    try:
        # Task statistics
        total_tasks = db.tasks.count_documents({})
        completed_tasks = db.tasks.count_documents({"status": "COMPLETED"})
        in_progress_tasks = db.tasks.count_documents({"status": {"$in": [
            "ASSIGNED", "MOVING_TO_PICKUP", "ATTACHED", "MOVING_TO_DROP"
        ]}})
        failed_tasks = db.tasks.count_documents({"status": "ERROR"})
        
        # Robot statistics
        total_robots = db.robots.count_documents({"deleted": False})
        available_robots = db.robots.count_documents({"deleted": False, "available": True})
        offline_robots = db.robots.count_documents({"deleted": False, "status": "OFFLINE"})
        
        # Shelf statistics
        total_shelves = db.shelves.count_documents({"deleted": False})
        shelves_in_use = db.tasks.count_documents({"status": {"$in": [
            "ASSIGNED", "MOVING_TO_PICKUP", "ATTACHED", "MOVING_TO_DROP"
        ]}})
        
        # Zone statistics
        total_zones = db.zones.count_documents({"deleted": False})
        
        # Performance metrics
        completed = list(db.tasks.find({"status": "COMPLETED", "duration_seconds": {"$exists": True}}))
        avg_duration = sum(t.get("duration_seconds", 0) for t in completed) / max(len(completed), 1) if completed else 0
        
        # System health score (0-100)
        health_score = 100
        if offline_robots > 0:
            health_score -= 10 * min(5, offline_robots)
        if failed_tasks > 0:
            health_score -= min(20, failed_tasks * 2)
        avg_battery = sum(r.get("battery_level", 100) for r in db.robots.find({"deleted": False})) / max(total_robots, 1)
        if avg_battery < 30:
            health_score -= 15
        health_score = max(0, min(100, health_score))
        
        return jsonify({
            "tasks": {
                "total": total_tasks,
                "in_progress": in_progress_tasks,
                "completed": completed_tasks,
                "failed": failed_tasks,
                "average_duration_seconds": round(avg_duration, 2),
                "success_rate": round((completed_tasks / max(total_tasks, 1)) * 100, 2)
            },
            "robots": {
                "total": total_robots,
                "available": available_robots,
                "busy": total_robots - available_robots - offline_robots,
                "offline": offline_robots
            },
            "resources": {
                "total_shelves": total_shelves,
                "shelves_in_use": shelves_in_use,
                "total_zones": total_zones
            },
            "health": {
                "score": round(health_score, 2),
                "status": "HEALTHY" if health_score >= 80 else "WARNING" if health_score >= 50 else "CRITICAL",
                "average_robot_battery": round(avg_battery, 2)
            },
            "timestamp": datetime.utcnow().isoformat()
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500
