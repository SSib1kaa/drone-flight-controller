"""
Drone Flight Controller Configuration
All settings for flight controller, sensors, and communication
"""

# ===== HARDWARE PLATFORM =====
# Options: 'raspberry_pi', 'stm32f4', 'stm32f7'
PLATFORM = 'raspberry_pi'

# ===== CONTROL LOOP FREQUENCY =====
CONTROL_LOOP_RATE = 200  # Hz (200Hz = 5ms loop)

# ===== GPIO PIN CONFIGURATION (Raspberry Pi) =====
GPIO_MODE = 'BCM'  # Use Broadcom pin numbering

# Motor PWM pins
MOTOR_PINS = {
    'front_left': 12,   # GPIO 12 (PWM0)
    'front_right': 13,  # GPIO 13 (PWM1)
    'rear_left': 18,    # GPIO 18 (PWM0)
    'rear_right': 19,   # GPIO 19 (PWM1)
}

# PWM Frequency and resolution
PWM_FREQUENCY = 50  # Hz (standard servo frequency)
PWM_MIN = 1000  # Minimum pulse width (microseconds)
PWM_MAX = 2000  # Maximum pulse width (microseconds)

# ===== I2C CONFIGURATION =====
I2C_BUS = 1  # Raspberry Pi I2C bus number (1 for Pi 3/4)
I2C_FREQUENCY = 400000  # 400 kHz standard mode

# I2C Device Addresses
I2C_DEVICES = {
    'imu_bno055': 0x28,      # BNO055 IMU (0x28 or 0x29)
    'pressure_bmp280': 0x76,  # BMP280 (0x76 or 0x77)
    'mag_hmc5883l': 0x1E,     # HMC5883L Magnetometer
}

# ===== UART CONFIGURATION (GPS) =====
UART_PORT = '/dev/ttyAMA0'  # Raspberry Pi serial port
UART_BAUDRATE = 9600  # u-blox NEO-6M default baud rate

# ===== IMU SENSOR CONFIGURATION =====
IMU_TYPE = 'BNO055'  # Sensor type
IMU_OPERATION_MODE = 0x0C  # NDOF mode (9-DOF)
IMU_POWER_MODE = 0x00  # Normal power mode

# Calibration offsets (tune these for your specific unit)
IMU_CALIBRATION = {
    'accel_offset': [0, 0, 0],
    'gyro_offset': [0, 0, 0],
    'mag_offset': [0, 0, 0],
}

# ===== PID CONTROLLER GAINS =====
PID_GAINS = {
    'roll': {'kp': 1.2, 'ki': 0.1, 'kd': 0.8},
    'pitch': {'kp': 1.2, 'ki': 0.1, 'kd': 0.8},
    'yaw': {'kp': 1.0, 'ki': 0.05, 'kd': 0.6},
    'altitude': {'kp': 1.5, 'ki': 0.2, 'kd': 0.5},
}

# PID integral limits (anti-windup)
PID_INTEGRAL_LIMIT = 100.0

# ===== FLIGHT PARAMETERS =====
MAX_ROLL_ANGLE = 45.0  # degrees
MAX_PITCH_ANGLE = 45.0  # degrees
MAX_CLIMB_RATE = 5.0  # m/s

# ===== BATTERY CONFIGURATION =====
BATTERY_VOLTAGE_ADC_PIN = 0  # ADC pin for battery voltage
BATTERY_WARNING_VOLTAGE = 10.8  # 3S LiPo minimum safe voltage
BATTERY_CRITICAL_VOLTAGE = 9.6  # Immediate landing voltage

# ===== TELEMETRY CONFIGURATION =====
TELEMETRY_ENABLED = True
TELEMETRY_RATE = 10  # Hz (10 updates per second)
TELEMETRY_PROTOCOL = 'mavlink'  # mavlink or custom

# MQTT Configuration (optional)
MQTT_ENABLED = False
MQTT_BROKER = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC = 'drone/telemetry'

# ===== LOGGING =====
LOGGING_ENABLED = True
LOG_FILE = '/var/log/drone_controller.log'
LOG_LEVEL = 'INFO'  # DEBUG, INFO, WARNING, ERROR, CRITICAL

# ===== SAFETY PARAMETERS =====
ARM_DISARM_TIME = 1.0  # seconds
MIN_THROTTLE = 0  # percent
MAX_THROTTLE = 100  # percent
FAILSAFE_ENABLED = True
FAILSAFE_ACTION = 'land'  # land, hover, rtb (return to base)

# ===== GPS NAVIGATION =====
GPS_ENABLED = True
WAYPOINT_RADIUS = 5.0  # meters (acceptance radius)
HOME_LATITUDE = 55.7558
HOME_LONGITUDE = 37.6173
HOME_ALTITUDE = 100.0  # meters above sea level

# ===== DEBUG OPTIONS =====
DEBUG_MODE = False
SIMULATE_SENSORS = False  # Use simulated sensor data instead of real
VERBOSE_OUTPUT = False
