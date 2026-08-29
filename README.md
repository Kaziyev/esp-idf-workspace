# ESP32-S3 + BMI270 I2C Project

This project demonstrates how to interface an **ESP32-S3** with a **Bosch BMI270 IMU** using I2C and ESP-IDF.

The project reads accelerometer data from the BMI270 and outputs the X, Y, and Z acceleration values through the serial monitor.

## Hardware

- ESP32-S3
- Bosch BMI270 IMU
- I2C interface

### I2C Configuration

| Signal | ESP32-S3 |
|---|---|
| SDA | GPIO 39 |
| SCL | GPIO 40 |

Detected BMI270 I2C address:

```text
0x69
```

Expected BMI270 CHIP ID:

```text
0x24
```

---

# Build

The project is built inside the ESP-IDF Docker container.

Open the terminal inside the development container and go to the project directory:

```bash
cd /workspace/apps/i2c_imu
```

Optional: clean the previous build:

```bash
idf.py fullclean
```

Build the project:

```bash
idf.py build
```

A successful build should finish with:

```text
Project build complete.
```

The generated application binary will be located at:

```text
build/i2c_imu.bin
```

---

# Flashing from Windows PowerShell

Open PowerShell in the project directory:

```powershell
cd "D:\Загрузки\course-iot-with-esp-idf-main\course-iot-with-esp-idf-main\workspace\apps\i2c_imu"
```

Check available COM ports if necessary:

```powershell
python -m serial.tools.list_ports -v
```

In the current setup, the ESP32-S3 uses:

```text
COM6
```

Flash the firmware:

```powershell
python -m esptool --port COM6 --chip esp32s3 --baud 460800 write-flash --flash-mode dio --flash-size 2MB --flash-freq 80m 0x0 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\i2c_imu.bin
```

A successful flash should finish with messages similar to:

```text
Hash of data verified.
Hard resetting via RTS pin...
```

---

# Serial Monitor

Start the serial monitor:

```powershell
python -m serial.tools.miniterm COM6 115200
```

The expected output is similar to:

```text
I2C bus created
FOUND device at 0x69
DIRECT CHIP_ID = 0x24
BMI270 initialized successfully!
Accelerometer configured
Accelerometer enabled!

RAW X=120 Y=-54 Z=16280 | g X=0.007 Y=-0.003 Z=0.994
```

To exit the serial monitor, press:

```text
Ctrl + ]
```

> Note: COM port numbers may be different on another computer. Use `python -m serial.tools.list_ports -v` to find the correct port.

---

# Typical Development Workflow

After changing the source code:

### 1. Build inside the Docker/ESP-IDF container

```bash
cd /workspace/apps/i2c_imu
idf.py build
```

### 2. Flash from Windows PowerShell

```powershell
python -m esptool --port COM6 --chip esp32s3 --baud 460800 write-flash --flash-mode dio --flash-size 2MB --flash-freq 80m 0x0 .\build\bootloader\bootloader.bin 0x8000 .\build\partition_table\partition-table.bin 0x10000 .\build\i2c_imu.bin
```

### 3. Monitor the sensor data

```powershell
python -m serial.tools.miniterm COM6 115200
```

---

# BMI270 Initialization Process

The program performs the following steps:

1. Creates the ESP32-S3 I2C bus.
2. Scans for the BMI270.
3. Detects the sensor at address `0x69`.
4. Reads the `CHIP_ID` register.
5. Verifies that the CHIP ID is `0x24`.
6. Initializes the BMI270 using the Bosch Sensor API.
7. Configures the accelerometer.
8. Enables the accelerometer.
9. Continuously reads X, Y, and Z acceleration data.
10. Converts raw accelerometer values into `g`.

---

# Project Structure

```text
i2c_imu/
├── CMakeLists.txt
└── main/
    ├── CMakeLists.txt
    ├── main.c
    ├── bmi2.c
    ├── bmi2.h
    ├── bmi2_defs.h
    ├── bmi270.c
    └── bmi270.h
```
