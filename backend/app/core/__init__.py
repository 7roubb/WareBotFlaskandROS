"""
Core enums and constants for the warehouse bot system
"""
from enum import Enum


class RobotStatus(str, Enum):
    """Robot operational status"""
    IDLE = "IDLE"
    MOVING = "MOVING"
    BUSY = "BUSY"
    CHARGING = "CHARGING"
    ERROR = "ERROR"
    OFFLINE = "OFFLINE"
    
    @staticmethod
    def to_code(status: str) -> int:
        """Convert status string to numeric code for InfluxDB"""
        mapping = {
            "IDLE": 0,
            "MOVING": 1,
            "BUSY": 2,
            "CHARGING": 3,
            "ERROR": 4,
            "OFFLINE": 5,
        }
        return mapping.get((status or "").upper(), -1)


class TaskStatus(str, Enum):
    """Task operational status"""
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class ShelfStatus(str, Enum):
    """Shelf availability status"""
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    MAINTENANCE = "MAINTENANCE"
    DAMAGED = "DAMAGED"


class WebSocketEvent(str, Enum):
    """WebSocket event types"""
    # Robot events
    TELEMETRY = "telemetry"
    ROBOT_STATUS = "robot_status"
    ROBOT_UPDATE = "robot_update"
    
    # Map events
    MAP_UPDATE = "map_update"
    
    # Task events
    TASK_STATUS = "task_status"
    TASK_UPDATE = "task_update"
    
    # System events
    CONNECT = "connect"
    DISCONNECT = "disconnect"
    ERROR = "error"


class MQTTTopic(str, Enum):
    """MQTT topic patterns"""
    ROBOT_STATUS = "robots/mp400/+/status"
    ROBOT_TASK = "robots/mp400/+/task_status"
    MAP_UPDATE = "warehouse/map"


class HttpMethod(str, Enum):
    """HTTP methods"""
    GET = "GET"
    POST = "POST"
    PUT = "PUT"
    PATCH = "PATCH"
    DELETE = "DELETE"


# Pagination defaults
DEFAULT_PAGE = 1
DEFAULT_PAGE_SIZE = 20
MAX_PAGE_SIZE = 100

# Timeouts (seconds)
MQTT_KEEPALIVE = 60
SOCKET_TIMEOUT = 30
API_TIMEOUT = 30

# Cache TTL (seconds)
CACHE_ROBOT_TTL = 60
CACHE_MAP_TTL = 300
CACHE_TASK_TTL = 30
