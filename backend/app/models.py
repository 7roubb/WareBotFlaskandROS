from typing import Optional, List, Dict
from pydantic import BaseModel, Field, validator
from datetime import datetime

# -------------------------------------
# Robot Status Enum
# -------------------------------------
ROBOT_STATUSES = {"IDLE", "BUSY", "ERROR", "OFFLINE"}


# =========================================================
# PRODUCT MODELS
# =========================================================
class ProductCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=200)
    sku: str = Field(..., min_length=1, max_length=100)
    quantity: int = Field(ge=0)

    category: Optional[str] = Field(default=None, max_length=100)
    brand: Optional[str] = Field(default=None, max_length=100)
    price: Optional[float] = Field(default=None, ge=0)
    weight_kg: Optional[float] = Field(default=None, ge=0)
    dimensions_cm: Optional[Dict[str, float]] = None
    barcode: Optional[str] = Field(default=None, max_length=100)

    main_image_url: Optional[str] = None
    image_urls: List[str] = Field(default_factory=list)
    shelf_id: Optional[str] = None
    description: Optional[str] = Field(default=None, max_length=1000)

    @validator("*", pre=True)
    def strip_all(cls, v):
        return v.strip() if isinstance(v, str) else v


class ProductUpdate(BaseModel):
    name: Optional[str] = Field(default=None, min_length=1, max_length=200)
    sku: Optional[str] = Field(default=None, min_length=1, max_length=100)
    quantity: Optional[int] = Field(default=None, ge=0)
    category: Optional[str] = None
    brand: Optional[str] = None
    price: Optional[float] = Field(default=None, ge=0)
    weight_kg: Optional[float] = Field(default=None, ge=0)
    dimensions_cm: Optional[Dict[str, float]] = None
    barcode: Optional[str] = None
    main_image_url: Optional[str] = None
    image_urls: Optional[List[str]] = None
    shelf_id: Optional[str] = None
    description: Optional[str] = Field(default=None, max_length=1000)


# =========================================================
# SHELF MODELS
# =========================================================
class ShelfCreate(BaseModel):
    warehouse_id: str = Field(..., min_length=1, max_length=100)
    # Current coordinates (map coordinates where the shelf currently is)
    x_coord: float
    y_coord: float
    level: int

    # Storage (immutable) coordinates. If provided at creation these will be used
    # as the shelf's storage/home location; otherwise the current coordinates
    # will be used to initialize storage fields by the service layer.
    storage_x: Optional[float] = None
    storage_y: Optional[float] = None
    storage_yaw: Optional[float] = None

    available: bool = True
    status: str = "IDLE"

    # NEW — to store the link of the generated AprilTag (backend will populate it)
    april_tag_url: Optional[str] = None

    @validator("warehouse_id", "status", pre=True)
    def strip_strings(cls, v):
        return v.strip() if isinstance(v, str) else v


class ShelfUpdate(BaseModel):
    warehouse_id: Optional[str] = Field(default=None, min_length=1, max_length=100)
    x_coord: Optional[int] = None
    y_coord: Optional[int] = None
    level: Optional[int] = None
    available: Optional[bool] = None
    status: Optional[str] = None
    
    # CRITICAL FIX: Allow updating storage location fields (for manual reset only)
    storage_x: Optional[float] = None
    storage_y: Optional[float] = None
    storage_yaw: Optional[float] = None
    
    # Location status tracking
    location_status: Optional[str] = None

    # NEW — allow updating tag URL if needed (rare, but supported)
    april_tag_url: Optional[str] = None

    @validator("warehouse_id", "status", "location_status", "april_tag_url", pre=True)
    def strip_strings(cls, v):
        return v.strip() if isinstance(v, str) else v

# =========================================================
# ROBOT MODELS
# =========================================================
class RobotCreate(BaseModel):
    name: str
    robot_id: str               # <-- user will type: robot1
    available: bool = True
    status: str = "IDLE"
    current_shelf_id: Optional[str] = None

    @validator("status")
    def validate_status(cls, v):
        v = v.strip().upper()
        if v not in ROBOT_STATUSES:
            raise ValueError(f"status must be one of {ROBOT_STATUSES}")
        return v

    @validator("name", "robot_id")   # <-- FIXED HERE (no 'topic')
    def strip_text(cls, v):
        return v.strip()


class RobotUpdate(BaseModel):
    name: Optional[str] = None
    robot_id: Optional[str] = None   # <-- instead of topic
    available: Optional[bool] = None
    status: Optional[str] = None
    current_shelf_id: Optional[str] = None

    @validator("status")
    def validate_status(cls, v):
        if v is None:
            return v
        v = v.strip().upper()
        if v not in ROBOT_STATUSES:
            raise ValueError(f"status must be one of {ROBOT_STATUSES}")
        return v

# =========================================================
# ROBOT TELEMETRY (USED WITH INFLUXDB)
# =========================================================
class RobotTelemetry(BaseModel):
    cpu_usage: float = Field(..., ge=0, le=100)
    ram_usage: float = Field(..., ge=0, le=100)
    battery_level: float = Field(..., ge=0, le=100)
    temperature: float = Field(..., ge=-40, le=120)
    x: float
    y: float
    status: Optional[str] = None

    timestamp: datetime = Field(default_factory=datetime.utcnow)

    @validator("status")
    def validate_status(cls, v):
        if v is None:
            return v
        v = v.strip().upper()
        if v not in ROBOT_STATUSES:
            raise ValueError(f"status must be one of {ROBOT_STATUSES}")
        return v


# =========================================================
# AUTH MODELS
# =========================================================
class AdminCreate(BaseModel):
    username: str = Field(..., min_length=3, max_length=50)
    password: str = Field(..., min_length=6, max_length=128)

    @validator("username")
    def strip_username(cls, v):
        return v.strip()


class AdminLogin(BaseModel):
    username: str
    password: str

    @validator("username")
    def strip_username(cls, v):
        return v.strip()


# =========================================================
# TASK MODELS
# =========================================================
class TaskCreate(BaseModel):
    shelf_id: str
    priority: int = Field(default=1, ge=1, le=10)
    description: Optional[str] = None
    zone_id: Optional[str] = None
    task_type: str = Field(default="PICKUP_AND_DELIVER")
    # task_type values:
    # - PICKUP_AND_DELIVER: pick shelf from current location, deliver to zone
    # - MOVE_SHELF: move shelf to a new target location
    # - RETURN_SHELF: return shelf from zone back to storage
    # - REPOSITION: reposition shelf within warehouse
    target_shelf_id: Optional[str] = None  # for MOVE_SHELF: destination shelf/location
    target_zone_id: Optional[str] = None   # for REPOSITION: destination zone
    # Optional: origin/storage snapshots (populated by backend at creation time when available)
    origin_storage_x: Optional[float] = None
    origin_storage_y: Optional[float] = None
    origin_storage_yaw: Optional[float] = None
    origin_pickup_x: Optional[float] = None
    origin_pickup_y: Optional[float] = None
    origin_pickup_yaw: Optional[float] = None


class TaskStatusEnum(str):
    """Task status tracking with complete state machine"""
    PENDING = "PENDING"
    ASSIGNED = "ASSIGNED"
    MOVING_TO_PICKUP = "MOVING_TO_PICKUP"
    ARRIVED_AT_PICKUP = "ARRIVED_AT_PICKUP"
    ATTACHED = "ATTACHED"
    MOVING_TO_DROP = "MOVING_TO_DROP"
    ARRIVED_AT_DROP = "ARRIVED_AT_DROP"
    RELEASED = "RELEASED"
    MOVING_TO_REFERENCE = "MOVING_TO_REFERENCE"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    CANCELLED = "CANCELLED"


# =========================================================
# ZONE MODELS
# =========================================================
class ZoneCreate(BaseModel):
    zone_id: str = Field(..., min_length=1, max_length=100)
    name: Optional[str] = Field(default=None, max_length=200)
    x: float
    y: float
    yaw: Optional[float] = 0.0

    @validator("zone_id", "name", pre=True)
    def strip_strings(cls, v):
        return v.strip() if isinstance(v, str) else v


class ZoneUpdate(BaseModel):
    name: Optional[str] = Field(default=None, max_length=200)
    x: Optional[float] = None
    y: Optional[float] = None
    yaw: Optional[float] = None

    @validator("name", pre=True)
    def strip_name(cls, v):
        return v.strip() if isinstance(v, str) else v


# =========================================================
# STOCK MODELS
# =========================================================
class ProductTransactionCreate(BaseModel):
    product_id: str
    quantity: int = Field(..., gt=0)
    action: str = Field(..., pattern="^(PICK|RETURN|ADJUST)$")
    description: Optional[str] = None


class StockReturn(BaseModel):
    product_id: str
    quantity: int = Field(..., gt=0)
    description: Optional[str] = None


class StockAdjust(BaseModel):
    product_id: str
    new_quantity: int = Field(..., ge=0)
    reason: Optional[str] = None


# =========================================================
# IMAGE MODELS
# =========================================================
class SetMainImage(BaseModel):
    image_url: str


class DeleteImage(BaseModel):
    index: int = Field(..., ge=0)
