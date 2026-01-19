"""
Task Management and Queue System
"""
import threading
from collections import deque
from typing import Optional, Dict, Any, Set
from warebot_task_runner.enums import TaskState


class TaskManager:
    """Manages task queue and deduplication"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # Task queue
        self.task_queue = deque()
        self.queue_lock = threading.Lock()
        
        # Task deduplication
        self.completed_tasks: Set[str] = set()
        self.in_progress_task_id: Optional[str] = None
        self.tasks_lock = threading.Lock()
        
        # Current task state
        self.current_task: Optional[Dict[str, Any]] = None
        self.task_state = TaskState.IDLE
    
    def is_task_duplicate(self, task_id: str) -> bool:
        """Check if task is already completed or in progress"""
        with self.tasks_lock:
            if task_id in self.completed_tasks:
                return True
            if self.in_progress_task_id == task_id:
                return True
            return False
    
    def mark_task_in_progress(self, task_id: str):
        """Mark task as currently in progress"""
        with self.tasks_lock:
            self.in_progress_task_id = task_id
            self.logger.info(f"📝 Task marked IN PROGRESS: {task_id}")
    
    def mark_task_completed(self, task_id: str):
        """Mark task as completed to prevent re-execution"""
        with self.tasks_lock:
            self.completed_tasks.add(task_id)
            if self.in_progress_task_id == task_id:
                self.in_progress_task_id = None
            self.logger.info(f"✅ Task marked COMPLETED: {task_id}")
    
    def queue_task(self, task_data: Dict[str, Any]) -> bool:
        """Add task to queue for processing with deduplication"""
        task_id = task_data.get("task_id")
        
        # Check for duplicates
        if self.is_task_duplicate(task_id):
            self.logger.warn(f"⚠️  DUPLICATE task ignored: {task_id}")
            return False
        
        with self.queue_lock:
            # Check if already in queue
            for queued_task in self.task_queue:
                if queued_task.get("task_id") == task_id:
                    self.logger.warn(f"⚠️  Task already in queue: {task_id}")
                    return False
            
            self.task_queue.append(task_data)
            self.logger.info(f"📋 Task queued: {task_id} (queue size: {len(self.task_queue)})")
            return True
    
    def get_next_task(self) -> Optional[Dict[str, Any]]:
        """Get next task from queue"""
        with self.queue_lock:
            if not self.task_queue:
                return None
            return self.task_queue.popleft()
    
    def has_pending_tasks(self) -> bool:
        """Check if there are pending tasks in queue"""
        with self.queue_lock:
            return len(self.task_queue) > 0
    
    def get_queue_size(self) -> int:
        """Get current queue size"""
        with self.queue_lock:
            return len(self.task_queue)