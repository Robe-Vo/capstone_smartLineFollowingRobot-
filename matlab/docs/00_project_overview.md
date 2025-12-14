# docs/00_project_overview.md

## Project Name
capstone_smartLineFollowingRobot-

## Purpose
A network-controlled line-following robot where:
- **ESP32** executes low-level actuation (DC motor + steering servo) and sensor acquisition.
- **MATLAB** supervises the run, performs higher-level logic/state switching, performs tuning, and plots/logs data.

## Key Requirements (as specified)
1. Network control (currently Bluetooth; extensible to other transports).
2. Motion-profile execution with **segment-based tuning** (controller type and gains can change per known distance/segment).
3. Line following using steering servo with **PID** and **ON/OFF** options.
4. A MATLAB app for tuning parameters and plotting/logging.

## High-Level Design (as specified)
- **Drive (speed loop):** PID on ESP32 at **10 ms**.
- **Drive (position loop):** PID on host (MATLAB) at **100 ms**.
- **Steering:** servo-based line tracking, adaptable per road segment.

## Repository References (unverified by tool access)
- MATLAB side: `matlab/+capstone`
- Low-level firmware: `low_level`
