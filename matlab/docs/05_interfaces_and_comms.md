# docs/05_interfaces_and_comms.md

## Roles
### ESP32 (low-level)
- Executes motor speed PID (10 ms)
- Reads sensors and encoder
- Applies steering commands to servo
- Packages sensor data into frames for host

### MATLAB (host)
- Supervisory control
- Road state machine / segment tuning
- Position loop PID (100 ms)
- Logging and plotting
- Sends mode switches and control commands

## Transport
- Current: Bluetooth (as specified)
- Extensibility to other network interfaces is a stated requirement, but no protocol abstraction was provided.

## Frame Definition
- A “frame” exists as a MATLAB-side struct storing everything related to one exchange cycle.
- **Không đủ dữ liệu để xác minh** the exact byte-level protocol (headers, payload fields, CRC/ACK policy) from the prompt alone.
