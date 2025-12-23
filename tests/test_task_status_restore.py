from datetime import datetime, timedelta

import pytest


class FakeCollection:
    def __init__(self):
        self._docs = {}

    def find_one(self, query):
        if "_id" in query:
            return self._docs.get(query["_id"]) or None
        return None

    def update_one(self, query, update):
        class Res:
            modified_count = 0
        res = Res()
        if "_id" in query and query["_id"] in self._docs:
            doc = self._docs[query["_id"]]
            if "$set" in update:
                for k, v in update["$set"].items():
                    doc[k] = v
                res.modified_count = 1
        return res


class FakeDB:
    def __init__(self):
        self.tasks = FakeCollection()
        self.robots = FakeCollection()


def test_update_task_status_return_triggers_restore(monkeypatch):
    from backend.app.services import task_service as ts

    fake_db = FakeDB()

    # create a fake task
    task = {
        "_id": "t123",
        "task_type": "RETURN_SHELF",
        "shelf_id": "shelf123",
        "status": "IN_PROGRESS",
        "created_at": datetime.utcnow() - timedelta(seconds=30),
    }
    fake_db.tasks._docs["t123"] = task

    monkeypatch.setattr(ts, "get_db", lambda: fake_db)

    called = {"restored": False}

    def fake_restore(shelf_id, task_id=None):
        called["restored"] = True
        return True

    # Patch the restore function used by task_service
    monkeypatch.setattr(ts, "restore_shelf_to_storage_location", fake_restore)

    ok = ts.update_task_status("t123", "COMPLETED")
    assert ok is True
    assert called["restored"] is True
