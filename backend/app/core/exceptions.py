"""
Custom exceptions for the warehouse bot application
"""


class WareBotException(Exception):
    """Base exception for all warehouse bot errors"""
    def __init__(self, message: str, code: int = 500):
        self.message = message
        self.code = code
        super().__init__(self.message)


class ValidationError(WareBotException):
    """Raised when input validation fails"""
    def __init__(self, message: str):
        super().__init__(message, 400)


class NotFoundError(WareBotException):
    """Raised when resource is not found"""
    def __init__(self, resource: str, identifier: str):
        message = f"{resource} not found: {identifier}"
        super().__init__(message, 404)


class DuplicateError(WareBotException):
    """Raised when trying to create a duplicate resource"""
    def __init__(self, resource: str, field: str, value: str):
        message = f"{resource} with {field}='{value}' already exists"
        super().__init__(message, 409)


class UnauthorizedError(WareBotException):
    """Raised when user is not authenticated"""
    def __init__(self, message: str = "Unauthorized"):
        super().__init__(message, 401)


class ForbiddenError(WareBotException):
    """Raised when user doesn't have permission"""
    def __init__(self, message: str = "Forbidden"):
        super().__init__(message, 403)


class ConflictError(WareBotException):
    """Raised when there's a conflict (e.g., robot already assigned to task)"""
    def __init__(self, message: str):
        super().__init__(message, 409)


class ServiceUnavailableError(WareBotException):
    """Raised when a service is unavailable (MQTT, DB, etc)"""
    def __init__(self, service: str):
        message = f"{service} is currently unavailable"
        super().__init__(message, 503)


class InvalidOperationError(WareBotException):
    """Raised when operation cannot be performed in current state"""
    def __init__(self, message: str):
        super().__init__(message, 400)
