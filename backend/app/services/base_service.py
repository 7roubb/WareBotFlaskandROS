"""
Base service class with common business logic
"""
from typing import Dict, Any, List, Optional, Tuple
from datetime import datetime
from flask import current_app
from ..core.exceptions import NotFoundError, ValidationError
from ..repositories import BaseRepository


class BaseService:
    """Base service class with common methods"""
    
    def __init__(self, repository: BaseRepository, resource_name: str):
        self.repository = repository
        self.resource_name = resource_name
    
    def log_info(self, message: str):
        """Log info message"""
        current_app.logger.info(f"[{self.resource_name}] {message}")
    
    def log_error(self, message: str, error: Exception = None):
        """Log error message"""
        if error:
            current_app.logger.error(f"[{self.resource_name}] {message}: {error}")
        else:
            current_app.logger.error(f"[{self.resource_name}] {message}")
    
    def log_debug(self, message: str):
        """Log debug message"""
        current_app.logger.debug(f"[{self.resource_name}] {message}")
    
    def get_or_raise(self, id: str) -> Dict[str, Any]:
        """Get resource by id or raise NotFoundError"""
        resource = self.repository.find_by_id(id)
        if not resource:
            raise NotFoundError(self.resource_name, id)
        return resource
    
    def paginate(self, query: Dict[str, Any], page: int = 1, page_size: int = 20) -> Tuple[List, int]:
        """Get paginated results"""
        if page < 1:
            page = 1
        if page_size < 1 or page_size > 100:
            page_size = 20
        
        skip = (page - 1) * page_size
        items = self.repository.find_all(query, skip=skip, limit=page_size)
        total = self.repository.count(query)
        
        return items, total
    
    def get_pagination_info(self, page: int, page_size: int, total: int) -> Dict[str, Any]:
        """Get pagination metadata"""
        total_pages = (total + page_size - 1) // page_size
        
        return {
            "current_page": page,
            "page_size": page_size,
            "total_items": total,
            "total_pages": total_pages,
            "has_next": page < total_pages,
            "has_prev": page > 1,
        }
    
    def validate_required_fields(self, data: Dict[str, Any], required_fields: List[str]):
        """Validate that all required fields are present"""
        missing = [field for field in required_fields if not data.get(field)]
        if missing:
            raise ValidationError(f"Missing required fields: {', '.join(missing)}")
    
    def validate_field_type(self, data: Dict[str, Any], field: str, expected_type: type):
        """Validate field type"""
        if field in data and not isinstance(data[field], expected_type):
            raise ValidationError(f"Field '{field}' must be {expected_type.__name__}")
    
    def add_timestamps(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Add created_at and updated_at timestamps"""
        now = datetime.utcnow()
        data["created_at"] = data.get("created_at", now)
        data["updated_at"] = now
        data["deleted"] = data.get("deleted", False)
        return data
    
    def format_response(self, data: Dict[str, Any], include_fields: List[str] = None, exclude_fields: List[str] = None) -> Dict[str, Any]:
        """Format response data"""
        if include_fields:
            data = {k: v for k, v in data.items() if k in include_fields}
        
        if exclude_fields:
            data = {k: v for k, v in data.items() if k not in exclude_fields}
        
        return data
    
    def serialize_id(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Convert MongoDB ObjectId to string"""
        if "_id" in data:
            data["id"] = str(data["_id"])
            del data["_id"]
        return data
    
    def handle_error(self, error: Exception) -> Dict[str, Any]:
        """Convert exception to error response"""
        if hasattr(error, 'code') and hasattr(error, 'message'):
            return {
                "error": error.message,
                "code": error.code
            }
        
        self.log_error("Unexpected error", error)
        return {
            "error": "Internal server error",
            "code": 500
        }
