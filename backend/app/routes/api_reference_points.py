"""
Reference Points API - Manage robot home/dock locations
"""

from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required
from app import extensions

reference_points_bp = Blueprint('reference_points', __name__)


def get_db():
    """Get MongoDB database instance"""
    return extensions.mongo_db


@reference_points_bp.route('/reference-points', methods=['GET'])
@jwt_required()
def get_all_reference_points():
    """Get reference points for all robots"""
    db = get_db()
    robots = db.robots.find({}, {'_id': 1, 'robot_id': 1, 'reference_point': 1})
    
    ref_points = {}
    for robot in robots:
        if 'reference_point' in robot:
            ref_points[robot['robot_id']] = robot['reference_point']
    
    return jsonify(ref_points)


@reference_points_bp.route('/<robot_id>/reference-point', methods=['GET'])
@jwt_required()
def get_reference_point(robot_id):
    """Get reference point for a specific robot"""
    db = get_db()
    robot = db.robots.find_one({'robot_id': robot_id})
    
    if not robot:
        return jsonify({'error': 'Robot not found'}), 404
    
    return jsonify(robot.get('reference_point', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}))


@reference_points_bp.route('/<robot_id>/reference-point', methods=['POST'])
@jwt_required()
def update_reference_point(robot_id):
    """Update reference point for a robot"""
    db = get_db()
    data = request.get_json()
    
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'error': 'Missing x or y coordinate'}), 400
    
    robot = db.robots.find_one({'robot_id': robot_id})
    if not robot:
        return jsonify({'error': 'Robot not found'}), 404
    
    ref_point = {
        'x': float(data['x']),
        'y': float(data['y']),
        'yaw': float(data.get('yaw', 0.0))
    }
    
    db.robots.update_one(
        {'robot_id': robot_id},
        {'$set': {'reference_point': ref_point}}
    )
    
    return jsonify({'message': 'Reference point updated', 'reference_point': ref_point})


@reference_points_bp.route('/<robot_id>/reference-point', methods=['DELETE'])
@jwt_required()
def delete_reference_point(robot_id):
    """Delete reference point for a robot (reset to origin)"""
    db = get_db()
    default_point = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    
    db.robots.update_one(
        {'robot_id': robot_id},
        {'$set': {'reference_point': default_point}}
    )
    
    return jsonify({'message': 'Reference point reset to origin'})
