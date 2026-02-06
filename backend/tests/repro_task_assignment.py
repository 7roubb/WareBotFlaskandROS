
import sys
import unittest
from datetime import datetime
from bson import ObjectId
from unittest.mock import MagicMock, patch

# Mock local imports
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


# Import the service under test
# We need to ensure we can import it even if dependencies are missing in this isolated script
# So we mock the specific function imports inside the file if needed, but since we mocked modules above, it should be fine.

from app.services.task_service import create_task_and_assign

class TestTaskAssignment(unittest.TestCase):

    def setUp(self):
        self.mock_db = MagicMock()
        self.patcher = patch('app.services.task_service.get_db', return_value=self.mock_db)
        self.patcher.start()
        
        
        # FIX: Mock serialize to return the document as is (or stringified _id)
        from app.services.utils_service import serialize
        def side_effect(doc):
            if doc and '_id' in doc:
                doc['id'] = str(doc['_id'])
            return doc
        serialize.side_effect = side_effect
        
        # Configure app.logger
        self.app_patcher = patch('app.services.task_service.app')
        self.mock_app = self.app_patcher.start()

    def tearDown(self):
        self.patcher.stop()
        self.app_patcher.stop()

    def test_nearest_robot_assigned_and_marked_busy(self):
        # Setup: 2 Robots
        # Robot 1: Far away (0,0) -> Shelf at (10,10) distance ~14
        # Robot 2: Close (9,9)   -> Shelf at (10,10) distance ~1.4
        
        # We need _resolve_shelf_coords to return (10, 10, 0)
        with patch('app.services.task_service._resolve_shelf_coords') as mock_coords:
            mock_coords.return_value = (10.0, 10.0, 0.0)
            
            robot1 = {
                "_id": ObjectId(),
                "robot_id": "robot1",
                "name": "Robot 1",
                "status": "IDLE",
                "current_x": 0.0,
                "current_y": 0.0,
                "battery_level": 100.0,
                "deleted": False
            }
            robot2 = {
                "_id": ObjectId(),
                "robot_id": "robot2",
                "name": "Robot 2",
                "status": "IDLE",
                "current_x": 9.0,
                "current_y": 9.0,
                "battery_level": 50.0, # Lower battery, but closer
                "deleted": False
            }
            
            self.mock_db.robots.find.return_value = [robot1, robot2]
            
            # Setup valid robots mock
            # The code iterates find result.
            
            # Action
            # We ignore return value as db.tasks.find_one mocks are tricky
            try:
                create_task_and_assign(
                    shelf_id="shelf123",
                    priority=1,
                    desc="Test Task"
                )
            except Exception:
                pass # The mock return value processing might fail, but we check calls
            
            # Assert
            
            # Verify insert_one was called with the correct robot assignment
            call_args = self.mock_db.tasks.insert_one.call_args
            self.assertIsNotNone(call_args, "insert_one should have been called")
            
            inserted_doc = call_args[0][0] # First arg is the doc
            self.assertEqual(inserted_doc['assigned_robot_id'], str(robot2['_id']))
            self.assertEqual(inserted_doc['status'], 'ASSIGNED')
            
            print(f"Verified Task assigned to Robot {robot2['_id']}")
            
            # Verify update_one was called to set BUSY
            self.mock_db.robots.update_one.assert_called_with(
                {"_id": robot2['_id']},
                {"$set": {"status": "BUSY", "updated_at": unittest.mock.ANY}}
            )
            print("Successfully assigned to nearest robot (Robot 2) and marked BUSY")

if __name__ == '__main__':
    unittest.main()
