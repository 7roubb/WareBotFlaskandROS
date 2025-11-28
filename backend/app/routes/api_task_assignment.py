"""
Task Assignment Routes - API endpoints for robot task management
"""

from flask import Blueprint, request, jsonify, current_app
from flask_jwt_extended import jwt_required, get_jwt
from pydantic import ValidationError as PydanticValidationError, BaseModel
from typing import Optional
import logging
import time

from ..services.robot_service import get_robot_service
from ..services.ros2_integration import get_ros2_service
from ..core.exceptions import ValidationError, NotFoundError

tasks_bp = Blueprint('task_assignment', __name__)
logger = logging.getLogger('TaskRoutes')


class TaskAssignmentRequest(BaseModel):
    """Task assignment request model"""
    robot_id: str
    target_x: float
    target_y: float
    target_yaw: float = 0.0
    priority: int = 0
    reference_point_x: Optional[float] = None
    reference_point_y: Optional[float] = None
    reference_point_yaw: Optional[float] = None


@tasks_bp.route('/assign', methods=['POST'])
@jwt_required()
def assign_task():
    """
    Assign a task to a robot
    
    Request body:
    {
        "robot_id": "robot_1",
        "target_x": 5.2,
        "target_y": 3.8,
        "target_yaw": 0.0,
        "priority": 1,
        "reference_point_x": 0.0,
        "reference_point_y": 0.0,
        "reference_point_yaw": 0.0
    }
    """
    try:
        # Validate request
        try:
            data = TaskAssignmentRequest(**request.json)
        except PydanticValidationError as e:
            return jsonify({'error': 'validation_error', 'details': e.errors()}), 400

        robot_id = data.robot_id
        target_x = data.target_x
        target_y = data.target_y
        target_yaw = data.target_yaw
        priority = data.priority

        # Check if robot exists
        robot_service = get_robot_service()
        try:
            robot = robot_service.get_robot(robot_id)
            if not robot:
                return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404
        except NotFoundError:
            return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404

        # Set reference point if provided
        if data.reference_point_x is not None and data.reference_point_y is not None:
            ros2_service = get_ros2_service()
            ros2_service.set_robot_reference_point(
                robot_id=robot_id,
                x=data.reference_point_x,
                y=data.reference_point_y,
                yaw=data.reference_point_yaw or 0.0,
                description=f'Reference point for task assignment'
            )

        # Assign task via ROS2
        ros2_service = get_ros2_service()
        task_id = f"task_{robot_id}_{int(time.time() * 1000)}"
        
        success = ros2_service.assign_task_to_robot(
            task_id=task_id,
            robot_id=robot_id,
            target_x=target_x,
            target_y=target_y,
            target_yaw=target_yaw,
            priority=priority
        )

        if not success:
            return jsonify({'error': 'service_error', 'message': 'Failed to assign task'}), 500

        # Also update robot in database
        robot_service.assign_task(robot_id, task_id)

        logger.info(f'Task {task_id} assigned to {robot_id}')

        return jsonify({
            'task_id': task_id,
            'robot_id': robot_id,
            'target_x': target_x,
            'target_y': target_y,
            'target_yaw': target_yaw,
            'priority': priority,
            'status': 'ASSIGNED',
            'message': 'Task assigned to robot'
        }), 201

    except Exception as e:
        logger.error(f'Error assigning task: {e}')
        return jsonify({'error': 'server_error', 'message': str(e)}), 500


@tasks_bp.route('/<robot_id>/reference-point', methods=['POST'])
@jwt_required()
def set_reference_point(robot_id: str):
    """
    Set reference point (home/dock) for a robot
    
    Request body:
    {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
        "description": "Main Dock"
    }
    """
    try:
        data = request.json
        
        # Validate robot exists
        robot_service = get_robot_service()
        try:
            robot = robot_service.get_robot(robot_id)
            if not robot:
                return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404
        except NotFoundError:
            return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404

        # Set reference point
        ros2_service = get_ros2_service()
        success = ros2_service.set_robot_reference_point(
            robot_id=robot_id,
            x=float(data.get('x', 0)),
            y=float(data.get('y', 0)),
            yaw=float(data.get('yaw', 0)),
            description=data.get('description', f'Reference point for {robot_id}')
        )

        if not success:
            return jsonify({'error': 'service_error', 'message': 'Failed to set reference point'}), 500

        logger.info(f'Reference point set for {robot_id}')

        return jsonify({
            'robot_id': robot_id,
            'x': float(data.get('x', 0)),
            'y': float(data.get('y', 0)),
            'yaw': float(data.get('yaw', 0)),
            'description': data.get('description'),
            'status': 'SUCCESS',
            'message': 'Reference point updated'
        }), 200

    except Exception as e:
        logger.error(f'Error setting reference point: {e}')
        return jsonify({'error': 'server_error', 'message': str(e)}), 500


@tasks_bp.route('/<robot_id>/start', methods=['POST'])
@jwt_required()
def start_task_execution(robot_id: str):
    """Start task execution for a robot"""
    try:
        # Validate robot exists
        robot_service = get_robot_service()
        try:
            robot = robot_service.get_robot(robot_id)
            if not robot:
                return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404
        except NotFoundError:
            return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404

        # Start task execution
        ros2_service = get_ros2_service()
        success = ros2_service.start_robot_task_execution(robot_id)

        if not success:
            return jsonify({'error': 'service_error', 'message': 'Failed to start task execution'}), 500

        logger.info(f'Task execution started for {robot_id}')

        return jsonify({
            'robot_id': robot_id,
            'status': 'EXECUTING',
            'message': 'Task execution started'
        }), 200

    except Exception as e:
        logger.error(f'Error starting task execution: {e}')
        return jsonify({'error': 'server_error', 'message': str(e)}), 500


@tasks_bp.route('/<robot_id>/stop', methods=['POST'])
@jwt_required()
def stop_task_execution(robot_id: str):
    """Stop task execution for a robot"""
    try:
        # Validate robot exists
        robot_service = get_robot_service()
        try:
            robot = robot_service.get_robot(robot_id)
            if not robot:
                return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404
        except NotFoundError:
            return jsonify({'error': 'not_found', 'message': f'Robot {robot_id} not found'}), 404

        # Stop task execution
        ros2_service = get_ros2_service()
        success = ros2_service.stop_robot_task_execution(robot_id)

        if not success:
            return jsonify({'error': 'service_error', 'message': 'Failed to stop task execution'}), 500

        logger.info(f'Task execution stopped for {robot_id}')

        return jsonify({
            'robot_id': robot_id,
            'status': 'STOPPED',
            'message': 'Task execution stopped'
        }), 200

    except Exception as e:
        logger.error(f'Error stopping task execution: {e}')
        return jsonify({'error': 'server_error', 'message': str(e)}), 500
