
import sys
import unittest
from datetime import datetime
from bson import ObjectId
from unittest.mock import MagicMock, patch

# Mock modules
sys.modules['app.extensions'] = MagicMock()
sys.modules['app.services.product_service'] = MagicMock()
sys.modules['app.services.utils_service'] = MagicMock()
sys.modules['app.services.shelf_location_service'] = MagicMock()
sys.modules['flask'] = MagicMock()
sys.modules['flasgger'] = MagicMock()
sys.modules['flask_jwt_extended'] = MagicMock()
sys.modules['pydantic'] = MagicMock()
sys.modules['flask_cors'] = MagicMock()
sys.modules['flask_socketio'] = MagicMock()
sys.modules['eventlet'] = MagicMock()
sys.modules['minio'] = MagicMock()
sys.modules['paho'] = MagicMock()
sys.modules['paho.mqtt'] = MagicMock()
sys.modules['paho.mqtt.client'] = MagicMock()
sys.modules['moms_apriltag'] = MagicMock()
sys.modules['cv2'] = MagicMock()
sys.modules['PIL'] = MagicMock()
sys.modules['numpy'] = MagicMock()

# Import files under test
from app.services.robot_service import update_robot_telemetry
from app.services.task_service import create_task_and_assign

class TestRaceCondition(unittest.TestCase):

    def setUp(self):
        self.mock_db = MagicMock()
        self.patcher = patch('app.services.robot_service.get_db', return_value=self.mock_db)
        self.patcher_task = patch('app.services.task_service.get_db', return_value=self.mock_db)
        self.patcher.start()
        self.patcher_task.start()
        
        # Configure app.logger
        self.app_patcher = patch('app.services.task_service.app')
        self.mock_app = self.app_patcher.start()
        
        # Configure update_one return
        update_result = MagicMock()
        update_result.modified_count = 1
        self.mock_db.robots.update_one.return_value = update_result

        # Fix serialize
        from app.services.utils_service import serialize
        def side_effect(doc):
            if doc and '_id' in doc:
                doc['id'] = str(doc['_id'])
            return doc
        serialize.side_effect = side_effect

    def tearDown(self):
        self.patcher.stop()
        self.patcher_task.stop()
        self.app_patcher.stop()

    def test_telemetry_does_not_overwrite_busy(self):
        # Setup: Robot is BUSY in DB
        robot_id = "robot1"
        self.mock_db.robots.find_one.return_value = {"robot_id": robot_id, "status": "BUSY"}
        
        telemetry = {
            "status": "IDLE", # Robot thinks it is IDLE
            "x": 10, "y": 10, "yaw": 0
        }
        
        # Action
        update_robot_telemetry(robot_id, telemetry)
        
        # Assert
        # The update call should have status="BUSY" (preserved), not "IDLE"
        call_args = self.mock_db.robots.update_one.call_args
        self.assertIsNotNone(call_args)
        
        update_doc = call_args[0][1]['$set']
        self.assertEqual(update_doc['status'], 'BUSY')
        print("Verified: Telemetry IDLE was ignored because DB status is BUSY")

    def test_assignment_skips_robot_with_active_task(self):
        # Setup: Robot1 has "IDLE" status but HAS an active task in DB
        # Robot2 is IDLE and free
        
        robot1_oid = ObjectId()
        robot2_oid = ObjectId()
        
        robot1 = {
            "_id": robot1_oid, "robot_id": "robot1", "status": "IDLE", 
            "current_x": 0, "current_y": 0, "battery_level": 100, "deleted": False
        }
        robot2 = {
            "_id": robot2_oid, "robot_id": "robot2", "status": "IDLE", 
            "current_x": 100, "current_y": 100, "battery_level": 100, "deleted": False
        }
        
        # find returns both
        self.mock_db.robots.find.return_value = [robot1, robot2]
        
        # task find_one returns a task for robot1, but None for robot2
        def task_find_side_effect(query):
            if query.get('assigned_robot_id') == str(robot1_oid):
                return {"_id": "existing_task"}
            return None
            
        self.mock_db.tasks.find_one.side_effect = task_find_side_effect
        
        # Mock coords
        with patch('app.services.task_service._resolve_shelf_coords') as mock_coords:
            mock_coords.return_value = (0.0, 0.0, 0.0)
            
            # Action
            create_task_and_assign(shelf_id="s1", priority=1)
            
            # Assert
            # Should have assigned to robot2 (because robot1 was skipped)
            call_args = self.mock_db.tasks.insert_one.call_args
            inserted_doc = call_args[0][0]
            self.assertEqual(inserted_doc['assigned_robot_id'], str(robot2_oid))
            print("Verified: Assignments skipped Robot1 due to active task existence")

if __name__ == '__main__':
    unittest.main()
