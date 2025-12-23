import types
from datetime import datetime

import pytest


class FakeCollection:
    def __init__(self):
        self._docs = {}

    def find_one(self, query):
        # support lookup by shelf_id or _id
        if not query:
            return None
        if "_id" in query:
            return self._docs.get(query["_id"])  # may be None
        if "shelf_id" in query:
            for d in self._docs.values():
                if d.get("shelf_id") == query["shelf_id"] and not d.get("deleted", False):
                    return d
        return None

    def update_one(self, query, update):
        # very small subset: handle {$set: {...}}
        target = None
        if "_id" in query:
            target = self._docs.get(query["_id"])
        elif "shelf_id" in query:
            for k, d in self._docs.items():
                if d.get("shelf_id") == query["shelf_id"] and not d.get("deleted", False):
                    target = d
                    break

        class Res:
            modified_count = 0

        res = Res()
        if not target:
            return res

        if "$set" in update:
            for k, v in update["$set"].items():
                target[k] = v
            target["updated_at"] = datetime.utcnow()
            res.modified_count = 1

        return res

    def insert_one(self, doc):
        _id = doc.get("_id") or doc.get("shelf_id")
        self._docs[_id] = doc
        class Res:
            inserted_id = _id
        return Res()


class FakeDB:
    def __init__(self):
        self.shelves = FakeCollection()
        self.shelf_location_history = FakeCollection()


def test_update_shelf_current_location_does_not_overwrite_storage(monkeypatch):
    from backend.app.services import shelf_location_service as sls

    fake_db = FakeDB()
    # initial shelf
    shelf = {
        "shelf_id": "s1",
        "x_coord": 20.0,
        "y_coord": 30.0,
        "yaw": 0.1,
        "storage_x": 10.0,
        "storage_y": 15.0,
        "storage_yaw": 0.0,
        "deleted": False,
    }
    fake_db.shelves._docs["s1"] = shelf

    monkeypatch.setattr(sls, "get_db", lambda: fake_db)

    # update current location
    ok = sls.update_shelf_current_location("s1", 99.0, 100.0, 0.5, location_status="AT_DROP_ZONE", task_id="t1")
    assert ok is True
    # storage values remain unchanged
    updated = fake_db.shelves._docs["s1"]
    assert updated["storage_x"] == 10.0
    assert updated["storage_y"] == 15.0
    assert updated["x_coord"] == 99.0


def test_restore_shelf_to_storage_location(monkeypatch):
    from backend.app.services import shelf_location_service as sls

    fake_db = FakeDB()
    shelf = {
        "shelf_id": "s2",
        "x_coord": 50.0,
        "y_coord": 60.0,
        "yaw": 0.2,
        "storage_x": 11.0,
        "storage_y": 12.0,
        "storage_yaw": 0.0,
        "deleted": False,
    }
    fake_db.shelves._docs["s2"] = shelf

    monkeypatch.setattr(sls, "get_db", lambda: fake_db)

    ok = sls.restore_shelf_to_storage_location("s2", task_id="t99")
    assert ok is True
    updated = fake_db.shelves._docs["s2"]
    assert updated["x_coord"] == 11.0
    assert updated["y_coord"] == 12.0
    assert updated["location_status"] == "STORED"
