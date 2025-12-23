# Drone Flight Controller - Installation Guide

## System Requirements

### Hardware
- **Raspberry Pi 4+** (recommended) or compatible board
- **STM32F405** or **STM32F722** microcontroller
- **IMU Sensor**: BNO055 9-DOF IMU
- **Pressure Sensor**: BMP280
- **Magnetometer**: HMC5883L
- **GPS Module**: u-blox NEO-6M or compatible
- **Brushless Motors**: 2212 1000KV or compatible
- **ESCs (Electronic Speed Controllers)**: 30A
- **Propellers**: 10-inch

### Software
- **Python 3.7+**
- **pip** (Python package manager)
- **Git**

## Installation Steps

### 1. Clone the Repository
```bash
git clone https://github.com/SSib1kaa/drone-flight-controller.git
cd drone-flight-controller
```

### 2. Install Dependencies

#### For Raspberry Pi:
```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev
pip install --upgrade pip
pip install -r requirements.txt
```

#### For STM32 Development:
```bash
pip install stm32loader
pip install -r requirements.txt
```

### 3. Hardware Configuration

#### GPIO Pin Configuration (Raspberry Pi)
- **Motor PWM**: GPIO 12, 13, 18, 19 (PWM pins)
- **I2C (IMU/Magnetometer)**: GPIO 2, 3 (SDA, SCL)
- **UART (GPS)**: GPIO 14, 15 (RX, TX)

#### I2C Device Addresses
- BNO055 IMU: 0x28 or 0x29
- BMP280: 0x76 or 0x77
- HMC5883L: 0x1E

### 4. Run the Flight Controller

```bash
python3 flight_controller.py
```

## Troubleshooting

### I2C Connection Issues
```bash
i2cdetect -y 1  # List I2C devices on Raspberry Pi
```

### GPIO Permission Errors
```bash
sudo usermod -aG gpio pi  # Add user to gpio group
```

### Motor Calibration
Motors require ESC calibration:
1. Power on the system
2. Apply full throttle
3. Connect battery to ESC
4. ESC will beep sequence
5. Reduce throttle to minimum

## Safety Precautions

⚠️ **IMPORTANT**: Always remove propellers before testing!

1. Never arm the drone indoors
2. Keep a safe distance during tests
3. Check all connections before flight
4. Monitor battery voltage
5. Use safety goggles

## Next Steps

Refer to [README.md](README.md) for usage examples and API documentation.
