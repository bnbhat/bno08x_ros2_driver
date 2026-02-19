^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bno08x_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2026-02-17)
------------------
* Calibration Status Monitoring
* Dynamic Covariance Scaling
* IMU Data Bundling improvements
* Memory Management Improvements
* Watchdog Timer Improvements
* Configuration & Parameter Updates
* Code readability improvements
* Contributors: Sergei Grichine

0.1.0 (2025-07-13)
------------------
* **Initial Release**: ROS2 driver for BNO08x series IMU
  - Supports BNO080, BNO085, and BNO08x series IMUs
  - Implements communication interfaces for I2C, can work with SPI and UART
  - WDT self recovering feature
  - Tested upto 200Hz data rate for IMU msg 
* Contributors: Balachandra Bhat, Lachlan Hurst
