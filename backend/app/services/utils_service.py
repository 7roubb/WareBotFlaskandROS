from typing import Dict, Any

def serialize(doc: Dict[str, Any]) -> Dict[str, Any]:
    if not doc:
        return None
    doc["id"] = str(doc["_id"])
    doc.pop("_id", None)
    return doc


