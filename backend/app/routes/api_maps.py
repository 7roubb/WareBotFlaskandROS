from flask import Blueprint, jsonify
from ..extensions import get_db

maps_bp = Blueprint("maps", __name__)


# =========================================================
# GET MERGED MAP
# =========================================================
@maps_bp.route("/merged", methods=["GET"])
def get_merged_map_route():
    """
    Returns the merged occupancy grid map stored in MongoDB.
    Document structure expected:
    {
        "_id": ...,
        "name": "merged_map",
        "data": [...],
        "width": ...,
        "height": ...,
        "resolution": ...,
        "origin": [...],
        ...
    }
    """
    db = get_db()
    doc = db.maps.find_one({"name": "merged_map"})

    if not doc:
        return {"error": "not_found"}, 404

    # Convert _id to string
    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)

    return jsonify(doc)
