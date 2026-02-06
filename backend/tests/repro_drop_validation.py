
import sys
import unittest
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
from app.services.task_service import create_task_and_assign

class TestDropValidation(unittest.TestCase):

    def setUp(self):
        self.mock_db = MagicMock()
        self.patcher_task = patch('app.services.task_service.get_db', return_value=self.mock_db)
        self.patcher_task.start()
        
        self.app_patcher = patch('app.services.task_service.app')
        self.mock_app = self.app_patcher.start()

        # Fix serialize
        from app.services.utils_service import serialize
        def side_effect(doc):
            if doc and '_id' in doc:
                doc['id'] = str(doc['_id'])
            return doc
        serialize.side_effect = side_effect

    def tearDown(self):
        self.patcher_task.stop()
        self.app_patcher.stop()

    def test_reject_if_drop_location_occupied(self):
        # Setup: Shelf A at (5,5)
        # We try to move Shelf B to (5.1, 5.1) -> Dist ~0.14 < 0.5 -> Collision
        
        shelf_b_id = str(ObjectId())
        
        # Mock RESOLVED coords: Pickup (0,0), Drop (5.1, 5.1)
        with patch('app.services.task_service._resolve_shelf_coords', return_value=(0,0,0)), \
             patch('app.services.task_service._resolve_zone_coords', return_value=(5.1,5.1,0)):
            
            # DB returns an occupant at (5,5)
            occupant = {
                "_id": ObjectId(), 
                "current_x": 5.0, "current_y": 5.0,
                "storage_x": 0, "storage_y": 0
            }
            # The search excludes shelf_b, so it finds 'occupant'
            self.mock_db.shelves.find.return_value = [occupant]
            
            # Action & Assert
            with self.assertRaises(ValueError) as cm:
                create_task_and_assign(
                    shelf_id=shelf_b_id,
                    priority=1,
                    task_type="PICKUP_AND_DELIVER",
                    zone_id="zone_occupied"
                )
            self.assertIn("drop_location_occupied", str(cm.exception))
            print("Verified: Task rejected because drop location is occupied")

    def test_reject_if_drop_location_is_storage_of_another(self):
        # Setup: Shelf A has STORAGE at (2,2)
        # We try to move Shelf B to (2,2)
        
        shelf_b_id = str(ObjectId())
        
        # Mock RESOLVED coords: Pickup (0,0), Drop (2,2)
        with patch('app.services.task_service._resolve_shelf_coords', return_value=(0,0,0)), \
             patch('app.services.task_service._resolve_zone_coords', return_value=(2.0,2.0,0)):
            
            # DB returns a shelf with STORAGE at (2,2) (but maybe current is elsewhere)
            occupant = {
                "_id": ObjectId(), 
                "current_x": 100, "current_y": 100, 
                "storage_x": 2.0, "storage_y": 2.0
            }
            self.mock_db.shelves.find.return_value = [occupant]
            
            # Action & Assert
            with self.assertRaises(ValueError) as cm:
                create_task_and_assign(
                    shelf_id=shelf_b_id,
                    priority=1,
                    task_type="PICKUP_AND_DELIVER",
                    zone_id="zone_storage"
                )
            self.assertIn("drop_location_is_storage", str(cm.exception))
            print("Verified: Task rejected because drop location is someone's storage")

if __name__ == '__main__':
    unittest.main()
