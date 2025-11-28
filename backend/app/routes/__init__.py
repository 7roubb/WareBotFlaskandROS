from flask import Blueprint

from .api_products import products_bp
from .api_shelves import shelves_bp
from .api_robots import robots_bp
from .api_tasks import tasks_bp
from .api_task_assignment import tasks_bp as task_assignment_bp
from .api_stock import stock_bp
from .api_images import images_bp
from .api_dashboard import dashboard_bp
from .api_maps import maps_bp
from .api_reference_points import reference_points_bp

api_bp = Blueprint("api", __name__)

# Register individual modules
api_bp.register_blueprint(products_bp, url_prefix="/products")
api_bp.register_blueprint(stock_bp, url_prefix="/products")
api_bp.register_blueprint(images_bp, url_prefix="/products")
api_bp.register_blueprint(shelves_bp, url_prefix="/shelves")
api_bp.register_blueprint(robots_bp, url_prefix="/robots")
api_bp.register_blueprint(reference_points_bp, url_prefix="/robots")
api_bp.register_blueprint(tasks_bp, url_prefix="/tasks")
api_bp.register_blueprint(task_assignment_bp, url_prefix="/task-assignment")
api_bp.register_blueprint(dashboard_bp, url_prefix="/dashboard")
api_bp.register_blueprint(maps_bp, url_prefix="/maps")
