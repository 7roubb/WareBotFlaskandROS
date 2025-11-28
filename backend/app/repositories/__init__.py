"""
Base repository class with common CRUD operations
"""
from typing import List, Dict, Any, Optional
from bson import ObjectId
from ..extensions import get_db
from ..core.exceptions import NotFoundError


class BaseRepository:
    """Base class for all repositories with common CRUD operations"""
    
    def __init__(self, collection_name: str):
        self.collection_name = collection_name
    
    @property
    def collection(self):
        """Get the MongoDB collection"""
        db = get_db()
        return db[self.collection_name]
    
    def find_by_id(self, id: str) -> Optional[Dict[str, Any]]:
        """Find document by MongoDB ObjectId"""
        try:
            oid = ObjectId(id)
            return self.collection.find_one({"_id": oid, "deleted": False})
        except:
            return None
    
    def find_one(self, query: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Find single document by query"""
        query["deleted"] = False
        return self.collection.find_one(query)
    
    def find_all(self, query: Dict[str, Any] = None, skip: int = 0, limit: int = 20) -> List[Dict[str, Any]]:
        """Find multiple documents with pagination"""
        if query is None:
            query = {}
        query["deleted"] = False
        
        return list(
            self.collection
            .find(query)
            .skip(skip)
            .limit(limit)
            .sort("created_at", -1)
        )
    
    def count(self, query: Dict[str, Any] = None) -> int:
        """Count documents matching query"""
        if query is None:
            query = {}
        query["deleted"] = False
        return self.collection.count_documents(query)
    
    def create(self, data: Dict[str, Any]) -> str:
        """Create new document"""
        result = self.collection.insert_one(data)
        return str(result.inserted_id)
    
    def update_by_id(self, id: str, data: Dict[str, Any]) -> bool:
        """Update document by id"""
        try:
            oid = ObjectId(id)
            result = self.collection.update_one(
                {"_id": oid, "deleted": False},
                {"$set": data}
            )
            return result.modified_count > 0
        except:
            return False
    
    def update(self, query: Dict[str, Any], data: Dict[str, Any]) -> int:
        """Update documents matching query"""
        query["deleted"] = False
        result = self.collection.update_many(query, {"$set": data})
        return result.modified_count
    
    def delete_by_id(self, id: str) -> bool:
        """Soft delete document by id"""
        try:
            oid = ObjectId(id)
            result = self.collection.update_one(
                {"_id": oid},
                {"$set": {"deleted": True}}
            )
            return result.modified_count > 0
        except:
            return False
    
    def hard_delete_by_id(self, id: str) -> bool:
        """Permanently delete document"""
        try:
            oid = ObjectId(id)
            result = self.collection.delete_one({"_id": oid})
            return result.deleted_count > 0
        except:
            return False
    
    def exists(self, query: Dict[str, Any]) -> bool:
        """Check if document exists"""
        query["deleted"] = False
        return self.collection.count_documents(query) > 0
    
    def bulk_create(self, documents: List[Dict[str, Any]]) -> List[str]:
        """Insert multiple documents"""
        result = self.collection.insert_many(documents)
        return [str(id) for id in result.inserted_ids]
    
    def bulk_update(self, updates: List[tuple]) -> int:
        """Perform bulk updates. updates: [(query, data), ...]"""
        count = 0
        for query, data in updates:
            count += self.update(query, data)
        return count
