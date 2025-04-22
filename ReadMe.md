# BNO08x ROS2 Driver

ROS2 driver for the BNO08x sensor family based on the SH-2 protocol.

The BNO08x family (BNO085/BNO086) is a compact System in Package (SiP) with integrated accelerometer, gyroscope, magnetometer, and a 32-bit ARM® Cortex™-M0+ running CEVA's SH-2 firmware. It delivers real-time 3D orientation, heading, calibrated acceleration, and angular velocity, with on-board sensor fusion algorithms and calibration. It supports I2C, SPI, and UART interfaces for sensor data output.
For more information, refer to the [datasheet](./docs/BNO080_085-Datasheet.pdf).

### Supported Features:
#### Communication Interfaces:
- I2C
- UART (Under development)
- SPI (Not yet implemented)

#### Data Rates:
- IMU data up to `400Hz`
- Magnetic field data up to `100Hz`

> [!IMPORTANT]
> This driver is **actively maintained**, with new features and bug fixes regularly added. 
>Please feel free to open an issue if you encounter any problems or have feature requests.   
>If you're using this driver, consider giving the repository a **star** to show your support.  

> [!NOTE]
> If you need SPI support, kindly open an issue, and it will be added to the driver in a 
> future update.

## Parameters
| Parameter |	Type	| Default	| Description |
|-----------|---------|-----------|-------------|
| frame_id      |string	|"bno085"	|Frame ID for sensor data messages.|
| publish.magnetic_field.enabled |bool	|true	|Enable publishing of magnetic field data.|
| publish.magnetic_field.rate	| int	|100	|Rate at which to publish magnetic field data (Hz).|
| publish.imu.enabled	|bool	|true	|Enable publishing of IMU data.|
| publish.imu.rate	|int	|100	|Rate at which to publish IMU data (Hz).|
| i2c.enabled	|bool	|true	|Enable I2C communication.|
| i2c.device	|string	|"/dev/i2c-7"	|I2C device path.|
| i2c.address	|string	|"0x4A"	|I2C address of the BNO08x sensor.|
| uart.enabled	|bool	|false	|Enable UART communication.|
| uart.device	|string	|"/dev/ttyACM0"	|UART device path.|

> [!NOTE]
> UART communication is currently under development and will be fully supported by November 15, 2024.
> In the meantime, please use the I2C communication interface.

## Installation
Clone the repository:
```bash
cd ~/ros_ws/src
git clone https://github.com/bnbhat/bno08x-ros2-driver.git
```
Build the package:
```bash
cd ~/ros_ws
colcon build --packages-select bno08x_ros2_driver
```
Install any missing dependencies using rosdep:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Usage
Sensor is configured to I2C by default. To use other communication interfaces , 
you need to do a hardware change. Please refer to the [datasheet](./docs/BNO080_085-Datasheet.pdf) 
for more information.
Once the hardware change is done, you can configure the driver to use UART/SPI communication 
interface.

> [!CAUTION]
> If you want to use UART, make sure to select UART-SHTP mode in the sensor configuration.  
> UART-RVC is a simplified interface mode with minimal functionalities and is not supported 
> by this driver.


### I2C
To run the driver with I2C communication interface, you need to provide the I2C bus number and 
the I2C address of the BNO08x sensor.

example config file:
```yaml
bno08x_driver:
  ros__parameters:

    frame_id: "bno085"  # The frame_id to use for the sensor data

    # Communication Interface
    # Select the communication interface to use 
    # (only one interface can be enabled at a time)
    # This requires a hardware change to the board
    # (see documentation for more information)
    i2c:
      enabled: true
      device: "/dev/i2c-18"
      address: "0x4A"

    uart:
      enabled: false
      device: "/dev/ttyACM0"  

    publish:
      all: false
      magnetic_field: 
        enabled: true
        rate: 100   # max 100 Hz
      imu:
        enabled: true
        rate: 100   # max 400 Hz
```

Launch the driver:
```bash
ros2 launch bno08x_ros2_driver bno085_i2c.launch.py
```

## Code Structure
 
At the core of this driver is a C++ class, `BNO08x`, which represents the BNO080/BNO085/BNO086 sensor and 
provides required methods to interact with the sensor using the [SH-2 protocol](./docs/SH-2-Reference-Manual.pdf).
 
The communication interface is abstracted using the `CommInterface` class, which can be implemented for any communication method (I2C, UART, SPI).

Currently, `I2C` and `UART` communication interfaces are implemted and tested on Linux systems.

The Core `BNO08x` implementation can be used in any C++ project on any platform by implementing the `CommInterface` for the respective platform.

`BNO08xROS` is a wrapper class that uses the `BNO08x` class to provide ROS2 specific functionality.

The directory structure is as follows:
```plaintext
.
├── CMakeLists.txt
├── config
│   ├── bno085_i2c.yaml
│   └── bno085_uart.yaml
├── docs
│   ├── BNO080_085-Datasheet.pdf
│   └── SH-2-Reference-Manual.pdf
├── include
│   ├── bno08x_ros2_driver
│   │   ├── bno08x.hpp
│   │   ├── bno08x_ros.hpp
│   │   ├── comm_interface.hpp
│   │   ├── i2c_interface.hpp
│   │   ├── logger.h
│   │   └── uart_interface.hpp
│   └── sh2
│       ├── CMakeLists.txt
│       ├── NOTICE.txt
│       ├── README.md
│       ├── sh2.c
│       ├── sh2_err.h
│       ├── sh2.h
│       ├── sh2_hal.h
│       ├── sh2_SensorValue.c
│       ├── sh2_SensorValue.h
│       ├── sh2_util.c
│       ├── sh2_util.h
│       ├── shtp.c
│       └── shtp.h
├── launch
│   ├── bno085_i2c.launch.py
│   └── bno085_uart.launch.py
├── LICENSE
├── package.xml
├── ReadMe.md
└──  src
    ├── bno08x.cpp
    ├── bno08x_ros.cpp
    └── ros_node.cpp
```

## Acknowledgements
This driver uses the SH-2 protocol library provided by Hillcrest Labs.
It can be found in the `include/sh2` directory.
Visit the official repository here: [SH-2 Protocol Library](https://github.com/ceva-dsp/sh2.git)

A special thanks to **Lemvos GmbH** for providing the hardware for testing and development.

## License
This project is licensed under the Apache License 2.0. You can find the full license text in the [LICENSE](./LICENSE) file of the repository.