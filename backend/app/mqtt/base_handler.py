"""
Base MQTT handler
"""
from flask import current_app


class BaseMQTTHandler:
    """Base class for MQTT message handlers"""
    
    def __init__(self, handler_name: str):
        self.handler_name = handler_name
    
    def log_info(self, message: str):
        """Log info message"""
        current_app.logger.info(f"[MQTT:{self.handler_name}] {message}")
    
    def log_error(self, message: str, error: Exception = None):
        """Log error message"""
        if error:
            current_app.logger.error(f"[MQTT:{self.handler_name}] {message}: {error}")
        else:
            current_app.logger.error(f"[MQTT:{self.handler_name}] {message}")
    
    def log_debug(self, message: str):
        """Log debug message"""
        current_app.logger.debug(f"[MQTT:{self.handler_name}] {message}")
    
    def handle(self, topic: str, payload: str) -> None:
        """
        Handle MQTT message
        Must be implemented by subclasses
        """
        raise NotImplementedError("Handler must implement handle() method")
