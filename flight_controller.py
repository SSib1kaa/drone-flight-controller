#!/usr/bin/env python3
"""
Drone Flight Controller
Manages attitude control, GPS navigation, and telemetry for UAV systems.
Optimized for Raspberry Pi and STM32 microcontrollers.
"""

import time
import math
import threading
from dataclasses import dataclass
from typing import Optional


@dataclass
class IMUData:
    """Inertial Measurement Unit data"""
    roll: float
    pitch: float
    yaw: float
    ax: float
    ay: float
    az: float


@dataclass
class GPSData:
    """Global Positioning System data"""
    latitude: float
    longitude: float
    altitude: float
    velocity: float
    heading: float


class PIDController:
    """PID Controller for attitude stabilization"""
    
    def __init__(self, kp: float, ki: float, kd: float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
    
    def update(self, current_value: float) -> float:
        """Calculate PID output"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        self.last_error = error
        
        output = (self.kp * error + self.ki * self.integral + self.kd * derivative)
        return output


class AttitudeController:
    """Manages drone attitude (roll, pitch, yaw) stabilization"""
    
    def __init__(self):
        self.roll_pid = PIDController(kp=1.2, ki=0.1, kd=0.8)
        self.pitch_pid = PIDController(kp=1.2, ki=0.1, kd=0.8)
        self.yaw_pid = PIDController(kp=1.0, ki=0.05, kd=0.6)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
    
    def set_target_attitude(self, roll: float, pitch: float, yaw: float):
        """Set target attitude in degrees"""
        self.target_roll = roll
        self.target_pitch = pitch
        self.target_yaw = yaw
        self.roll_pid.setpoint = roll
        self.pitch_pid.setpoint = pitch
        self.yaw_pid.setpoint = yaw
    
    def calculate_motor_commands(self, imu_data: IMUData) -> dict:
        """Calculate motor PWM values based on current attitude"""
        roll_correction = self.roll_pid.update(imu_data.roll)
        pitch_correction = self.pitch_pid.update(imu_data.pitch)
        yaw_correction = self.yaw_pid.update(imu_data.yaw)
        
        base_throttle = 500
        m1 = base_throttle + pitch_correction + roll_correction - yaw_correction
        m2 = base_throttle + pitch_correction - roll_correction + yaw_correction
        m3 = base_throttle - pitch_correction - roll_correction - yaw_correction
        m4 = base_throttle - pitch_correction + roll_correction + yaw_correction
        
        m1 = max(0, min(1000, int(m1)))
        m2 = max(0, min(1000, int(m2)))
        m3 = max(0, min(1000, int(m3)))
        m4 = max(0, min(1000, int(m4)))
        
        return {"M1": m1, "M2": m2, "M3": m3, "M4": m4}


class NavigationController:
    """Manages GPS-based navigation and waypoint following"""
    
    def __init__(self):
        self.current_position: Optional[GPSData] = None
        self.target_waypoint: Optional[GPSData] = None
        self.waypoint_list = []
    
    def add_waypoint(self, lat: float, lon: float, alt: float):
        """Add a waypoint to the flight plan"""
        self.waypoint_list.append(GPSData(
            latitude=lat, longitude=lon, altitude=alt,
            velocity=0.0, heading=0.0
        ))
    
    def distance_to_waypoint(self) -> float:
        """Calculate distance to waypoint using Haversine formula"""
        if not self.current_position or not self.target_waypoint:
            return 0.0
        
        R = 6371000
        lat1 = math.radians(self.current_position.latitude)
        lat2 = math.radians(self.target_waypoint.latitude)
        dlat = math.radians(self.target_waypoint.latitude - self.current_position.latitude)
        dlon = math.radians(self.target_waypoint.longitude - self.current_position.longitude)
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        return R * c


class DroneFlightController:
    """Main flight controller integrating all subsystems"""
    
    def __init__(self, loop_rate: float = 200.0):
        self.loop_rate = loop_rate
        self.loop_time = 1.0 / loop_rate
        self.running = False
        self.attitude_controller = AttitudeController()
        self.navigation_controller = NavigationController()
        self.flight_mode = "STABILIZE"
    
    def control_loop(self):
        """Main control loop"""
        print(f"Flight Controller started at {self.loop_rate} Hz")
        while self.running:
            loop_start = time.time()
            imu_data = IMUData(roll=0.0, pitch=0.0, yaw=0.0, ax=0.0, ay=0.0, az=-9.81)
            motor_commands = self.attitude_controller.calculate_motor_commands(imu_data)
            
            if self.flight_mode == "AUTO" and self.navigation_controller.target_waypoint:
                distance = self.navigation_controller.distance_to_waypoint()
                print(f"Distance to waypoint: {distance:.1f}m")
            
            elapsed = time.time() - loop_start
            if elapsed < self.loop_time:
                time.sleep(self.loop_time - elapsed)
    
    def start(self):
        """Start the flight controller"""
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
    
    def stop(self):
        """Stop the flight controller"""
        self.running = False


def main():
    """Example usage"""
    fc = DroneFlightController(loop_rate=200)
    fc.navigation_controller.add_waypoint(lat=55.7558, lon=37.6173, alt=100)
    fc.attitude_controller.set_target_attitude(roll=0, pitch=0, yaw=0)
    fc.start()
    
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        fc.stop()
        print("Flight controller stopped")


if __name__ == "__main__":
    main()
