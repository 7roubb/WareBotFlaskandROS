from typing import Optional, List, Dict
from pydantic import BaseModel, Field, validator
from datetime import datetime

# ---------- Enum-like ----------
ROBOT_STATUSES = {"IDLE", "BUSY", "ERROR", "OFFLINE"}


# ---------- Product ----------
class ProductCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=200)
    sku: str = Field(..., min_length=1, max_length=100)
    quantity: int = Field(ge=0)

    # Extra details
    category: Optional[str] = Field(default=None, max_length=100)
    brand: Optional[str] = Field(default=None, max_length=100)
    price: Optional[float] = Field(default=None, ge=0)
    weight_kg: Optional[float] = Field(default=None, ge=0)
    dimensions_cm: Optional[Dict[str, float]] = Field(
        default=None,
        description="e.g. {\"width\": 10, \"height\": 20, \"depth\": 5}",
    )
    barcode: Optional[str] = Field(default=None, max_length=100)

    # Images: URLs (you can store S3 / local server / CDN URLs)
    main_image_url: Optional[str] = None
    image_urls: List[str] = Field(default_factory=list)

    # Shelf relation
    shelf_id: Optional[str] = None

    description: Optional[str] = Field(default=None, max_length=1000)

    @validator("name", "sku", "category", "brand", "barcode", pre=True, always=True)
    def strip_strings(cls, v):
        if v is None:
            return v
        return v.strip()


class ProductUpdate(BaseModel):
    name: Optional[str] = Field(default=None, min_length=1, max_length=200)
    sku: Optional[str] = Field(default=None, min_length=1, max_length=100)
    quantity: Optional[int] = Field(default=None, ge=0)

    category: Optional[str] = Field(default=None, max_length=100)
    brand: Optional[str] = Field(default=None, max_length=100)
    price: Optional[float] = Field(default=None, ge=0)
    weight_kg: Optional[float] = Field(default=None, ge=0)
    dimensions_cm: Optional[Dict[str, float]] = None
    barcode: Optional[str] = Field(default=None, max_length=100)

    main_image_url: Optional[str] = None
    image_urls: Optional[List[str]] = None

    shelf_id: Optional[str] = None
    description: Optional[str] = Field(default=None, max_length=1000)


# ---------- Shelf ----------
class ShelfCreate(BaseModel):
    warehouse_id: str
    x_coord: int
    y_coord: int
    level: int
    available: bool = True
    status: str = "IDLE"  # IDLE, IN_USE, BLOCKED...

    @validator("warehouse_id", "status")
    def strip_strings(cls, v):
        return v.strip()


class ShelfUpdate(BaseModel):
    warehouse_id: Optional[str] = None
    x_coord: Optional[int] = None
    y_coord: Optional[int] = None
    level: Optional[int] = None
    available: Optional[bool] = None
    status: Optional[str] = None


# ---------- Robot ----------
class RobotCreate(BaseModel):
    name: str
    topic: str
    available: bool = True
    status: str = "IDLE"
    current_shelf_id: Optional[str] = None

    @validator("status")
    def validate_status(cls, v):
        v = v.strip().upper()
        if v not in ROBOT_STATUSES:
            raise ValueError(f"status must be one of {ROBOT_STATUSES}")
        return v

    @validator("name")
    def strip_name(cls, v):
        return v.strip()


class RobotUpdate(BaseModel):
    name: Optional[str] = None
    topic: Optional[str] = None
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


# ---------- Robot Telemetry (MQTT) ----------
class RobotTelemetry(BaseModel):
    cpu_usage: float = Field(..., ge=0, le=100)
    ram_usage: float = Field(..., ge=0, le=100)
    battery_level: float = Field(..., ge=0, le=100)
    temperature: float = Field(..., ge=-40, le=120)
    x: float
    y: float
    status: Optional[str] = None

    @validator("status")
    def validate_status(cls, v):
        if v is None:
            return v
        v = v.strip().upper()
        if v not in ROBOT_STATUSES:
            raise ValueError(f"status must be one of {ROBOT_STATUSES}")
        return v


# ---------- Admin / Auth ----------
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

class TaskCreate(BaseModel):
    shelf_id: str = Field(..., description="Target shelf to pick/place")
    priority: int = Field(default=1, ge=1, le=10)
    description: Optional[str] = None

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
