from flask import Blueprint, render_template_string
from .services import list_robots, list_shelves, list_products

dashboard_bp = Blueprint("dashboard", __name__)


TEMPLATE = """
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>Warebot Dashboard</title>
  <style>
    body { font-family: sans-serif; margin: 20px; }
    h1, h2 { margin-bottom: 0.2rem; }
    table { border-collapse: collapse; width: 100%; margin-bottom: 1.5rem; }
    th, td { border: 1px solid #ccc; padding: 0.4rem; text-align: left; font-size: 0.9rem; }
    th { background: #f0f0f0; }
    .chip { padding: 2px 6px; border-radius: 4px; font-size: 0.8rem; }
    .status-IDLE { background: #e0ffe0; }
    .status-BUSY { background: #fff0c2; }
    .status-ERROR { background: #ffd4d4; }
    .status-OFFLINE { background: #eeeeee; }
  </style>
</head>
<body>
  <h1>Warebot Dashboard</h1>

  <h2>Robots</h2>
  <table>
    <thead>
      <tr>
        <th>Name</th>
        <th>Available</th>
        <th>Status</th>
        <th>Current Shelf</th>
        <th>CPU %</th>
        <th>RAM %</th>
        <th>Battery %</th>
        <th>Temp °C</th>
        <th>Location (x, y)</th>
      </tr>
    </thead>
    <tbody>
      {% for r in robots %}
      <tr>
        <td>{{ r.name }}</td>
        <td>{{ 'Yes' if r.available else 'No' }}</td>
        <td><span class="chip status-{{ r.status }}">{{ r.status }}</span></td>
        <td>{{ r.current_shelf_id or '-' }}</td>
        <td>{{ r.cpu_usage if r.cpu_usage is not none else '-' }}</td>
        <td>{{ r.ram_usage if r.ram_usage is not none else '-' }}</td>
        <td>{{ r.battery_level if r.battery_level is not none else '-' }}</td>
        <td>{{ r.temperature if r.temperature is not none else '-' }}</td>
        <td>
          {% if r.x is not none and r.y is not none %}
            ({{ "%.2f"|format(r.x) }}, {{ "%.2f"|format(r.y) }})
          {% else %}
            -
          {% endif %}
        </td>
      </tr>
      {% endfor %}
    </tbody>
  </table>

  <h2>Shelves</h2>
  <table>
    <thead>
      <tr>
        <th>ID</th>
        <th>Warehouse</th>
        <th>(x, y, level)</th>
        <th>Available</th>
        <th>Status</th>
      </tr>
    </thead>
    <tbody>
      {% for s in shelves %}
      <tr>
        <td>{{ s.id }}</td>
        <td>{{ s.warehouse_id }}</td>
        <td>({{ s.x_coord }}, {{ s.y_coord }}, {{ s.level }})</td>
        <td>{{ 'Yes' if s.available else 'No' }}</td>
        <td>{{ s.status }}</td>
      </tr>
      {% endfor %}
    </tbody>
  </table>

  <h2>Products</h2>
  <table>
    <thead>
      <tr>
        <th>Name</th>
        <th>SKU</th>
        <th>Qty</th>
        <th>Shelf ID</th>
      </tr>
    </thead>
    <tbody>
      {% for p in products %}
      <tr>
        <td>{{ p.name }}</td>
        <td>{{ p.sku }}</td>
        <td>{{ p.quantity }}</td>
        <td>{{ p.shelf_id or '-' }}</td>
      </tr>
      {% endfor %}
    </tbody>
  </table>
</body>
</html>
"""


@dashboard_bp.route("/")
def dashboard():
    robots = list_robots()
    shelves = list_shelves()
    products = list_products()
    return render_template_string(
        TEMPLATE, robots=robots, shelves=shelves, products=products
    )
