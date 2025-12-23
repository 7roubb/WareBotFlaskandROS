"""
Backfill storage_x/storage_y/storage_yaw for legacy shelves.

Usage:
    python scripts/backfill_storage_coords.py --commit
    python scripts/backfill_storage_coords.py --dry-run

This script can be imported and used programmatically by passing a pymongo `db` object
and `commit` flag to `backfill_storage_coords(db, commit=False)`.
"""
from typing import Tuple
import argparse
from pprint import pprint


def backfill_storage_coords(db, commit: bool = False) -> Tuple[int, int]:
    """
    Backfill shelves missing storage_x/storage_y/storage_yaw from current coordinates.
    Returns tuple (checked_count, updated_count)
    """
    shelves = list(db.shelves.find({"deleted": False}))
    checked = 0
    updated = 0

    for s in shelves:
        checked += 1
        sid = s.get("_id")
        # If storage_x is missing or None
        if s.get("storage_x") is None:
            # Determine fallback values
            x = s.get("x_coord", s.get("x", 0.0))
            y = s.get("y_coord", s.get("y", 0.0))
            yaw = s.get("storage_yaw", s.get("yaw", 0.0))

            print(f"Will set storage for shelf {sid}: x={x}, y={y}, yaw={yaw}")
            if commit:
                db.shelves.update_one({"_id": sid}, {"$set": {"storage_x": float(x), "storage_y": float(y), "storage_yaw": float(yaw)}})
                updated += 1

    print(f"Checked {checked} shelves, updated {updated} (commit={commit})")
    return checked, updated


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Backfill storage coords for shelves")
    parser.add_argument("--commit", action="store_true", help="Apply changes to DB")
    args = parser.parse_args()

    # Lazy import so script can be imported in tests while not needing a running DB
    try:
        from backend.app.extensions import get_db
        db = get_db()
    except Exception as e:
        print("Failed to get DB from application context:", e)
        print("Run this script from the project root with the backend app context available, or import and call backfill_storage_coords(db)")
        raise

    backfill_storage_coords(db, commit=args.commit)
