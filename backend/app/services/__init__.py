# Make all services available at "from app.services import ..."

from .product_service import (
    create_product, list_products, get_product,
    update_product, soft_delete_product, search_products_by_name,
    get_products_for_shelf
)

from .stock_service import (
    subtract_from_product_stock,
    return_product_stock,
    adjust_product_stock
)

from .shelf_service import (
    create_shelf, list_shelves, get_shelf,
    update_shelf, soft_delete_shelf
)

from .robot_service import (
    create_robot, list_robots, get_robot,
    update_robot, soft_delete_robot,
    update_robot_telemetry, write_robot_telemetry_influx
)

from .task_service import (
    create_task_and_assign, list_tasks
)

from .dashboard_service import (
    dashboard_top_moving_products,
    dashboard_shelf_summary,
    dashboard_daily_movements
)

from .apriltag_service import (
    generate_apriltag,
    upload_apriltag
)

from .minio_service import (
    upload_image_to_minio,
    delete_image_from_minio,
    update_product_images
)
from .admin_service import (
    create_admin,
    get_admin_by_username,
    verify_admin_password
)
from .task_service import (
    create_task_and_assign,
    list_tasks,
    update_task_status,   # ← Missing, ADD THIS!
)
