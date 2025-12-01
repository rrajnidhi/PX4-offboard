# PX4 Offboard Simulation with ROS-Noetic, MAVROS and Python Scripting

This repository provides a complete setup for running PX4 SITL (Software-In-The-Loop) simulation with ROS Noetic and MAVROS. It demonstrates autonomous mode switching, controlled offboard navigation, and automated landing using Python scripts.

---

## Features

* Fully tested PX4 SITL environment for Ubuntu 20.04 using multirotor configurations
* Automatic switching between Mission mode and Offboard mode and back
* Automatic landing after reaching the final waypoint or home position

---

## Demonstration Video

Click on the thumbnail below to watch the SITL simulation video.

[![Watch the video](https://img.youtube.com/vi/txMo8NnxLfE/0.jpg)](https://youtu.be/txMo8NnxLfE)

---

## Requirements

* Ubuntu 20.04
* ROS Noetic
* PX4 Firmware (SITL)
* MAVROS
* GeographicLib datasets
* Python 3 with required dependencies

---

## Running the Simulation

1. Start PX4 SITL
2. Launch MAVROS with the appropriate configuration
3. Run the Python offboard script
4. Observe the vehicle switching modes and executing the mission

---

## Notes

* Ensure MAVROS topics are correctly mapped to PX4 SITL
* Offboard mode requires continuous command publication to remain active
* Some parameters may vary depending on the vehicle type and firmware version

---

## License

This project is released under an open-source license. Refer to the LICENSE file for details.
