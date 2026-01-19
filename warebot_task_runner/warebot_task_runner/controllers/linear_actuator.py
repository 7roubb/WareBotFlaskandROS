"""
Linear Actuator Controller for WareBot
"""
import time
import threading
import serial


class LinearActuatorController:
    """Controller for 4 linear actuators via serial"""
    
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200, logger=None):
        """
        Initialize serial connection to Arduino controlling actuators
        
        Args:
            port: Serial port for actuator controller (e.g., /dev/ttyUSB1)
            baudrate: Communication speed (default: 115200)
            logger: ROS logger for output
        """
        self.logger = logger
        self.ser = None
        self.connected = False
        self.port = port
        self.baudrate = baudrate
        self._serial_lock = threading.Lock()
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2.5)  # Wait for Arduino to reset
            self.connected = True
            if self.logger:
                self.logger.info(f"🔌 Linear actuators connected on {port}")
            self._read_response()  # Clear startup messages
        except serial.SerialException as e:
            self.connected = False
            if self.logger:
                self.logger.error(f"❌ Failed to connect to actuators on {port}: {e}")
                self.logger.warn("⚠️  Linear actuators NOT available - continuing without them")
            else:
                print(f"❌ Failed to connect to actuators on {port}: {e}")
    
    def _send_command(self, command: str):
        """Send command to Arduino"""
        if not self.connected or not self.ser:
            if self.logger:
                self.logger.warn(f"⚠️  Actuators not connected, skipping command: {command}")
            return False
        
        try:
            with self._serial_lock:
                self.ser.write(f"{command}\n".encode())
                time.sleep(0.1)
                self._read_response()
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"❌ Actuator command failed: {e}")
            return False
    
    def _read_response(self):
        """Read and log response from Arduino"""
        if not self.connected or not self.ser:
            return
        
        try:
            while self.ser.in_waiting > 0:
                response = self.ser.readline().decode('utf-8').strip()
                if response and self.logger:
                    self.logger.debug(f"🔧 Actuator: {response}")
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Failed to read actuator response: {e}")
    
    def extend(self):
        """Extend all actuators"""
        if self.logger:
            self.logger.info("⬆️ EXTENDING linear actuators")
        return self._send_command("EXTEND")
    
    def retract(self):
        """Retract all actuators"""
        if self.logger:
            self.logger.info("⬇️ RETRACTING linear actuators")
        return self._send_command("RETRACT")
    
    def stop(self):
        """Stop all actuators"""
        if self.logger:
            self.logger.info("⏹️ STOPPING linear actuators")
        return self._send_command("STOP")
    
    def close(self):
        """Close serial connection"""
        if self.connected and self.ser:
            try:
                self.stop()  # Ensure actuators are stopped
                time.sleep(0.2)
                self.ser.close()
                if self.logger:
                    self.logger.info("🔌 Linear actuator connection closed")
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"Error closing actuator connection: {e}")