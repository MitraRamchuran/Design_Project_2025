# EEE4113F 2025
# Penguin Nest Protection System

*Concept image of the artificial nest system*

This project involves designing and implementing an intelligent artificial nest system to protect endangered African penguin eggs from predators. The system combines three integrated subsystems: threat detection, penguin identification, and automated nest protection.

## Project Structure

### Threat Detection Subsystem
- Utilizes ESP32-CAM module with motion detection and image recognition
- Implements computer vision to identify predators
- Triggers protective measures when threats are detected

### Penguin Identification Subsystem
- Uses RFID technology to recognize tagged penguins
- Manages nest access through automated door control
- Maintains whitelist of approved penguin tags

### Nest Protection Subsystem
- Implements weather-resistant mechanical door system
- Powered by low-speed, high-torque DC motor
- Provides insulation and camouflage matching natural habitat

## Subsystem Requirements

### Threat Detection Subsystem
- Detect movement within 5-meter radius of nest
- Classify predators with >75% accuracy
- Transmit alerts within 2 seconds of detection
- Operate in various lighting conditions (0-100,000 lux)

### Penguin Identification Subsystem
- Read RFID tags within 10cm range
- Process identification within 1 second
- Maintain database of 100+ penguin IDs
- Operate in wet conditions (IP65 rating)

### Nest Protection Subsystem
- Open/close door within 5 seconds
- Withstand coastal wind speeds up to 60km/h
- Maintain interior temperature between 5°C-30°C
- Operate on battery power for 72+ hours

## How to Use

### Installation
1. Assemble nest structure at penguin colony site
2. Install RFID reader at nest entrance
3. Mount ESP32-CAM with clear field of view
4. Connect all subsystems to central microcontroller

### Testing
1. Verify RFID recognition with test tags
2. Validate predator detection with sample images
3. Test door mechanism under various weather conditions
4. Monitor system power consumption

### Maintenance
- Monthly camera lens cleaning
- Bi-monthly mechanical component inspection
- Quarterly battery replacement
- Annual full system diagnostic

## Contributors
- Alex Massyn (MSSALE009) - Hardware and Mechanical Subsystem
- Kival Maharaj (MHRKIV001) - Penguin Presence Detection Subsystem
- Mitra Ramchuran (RMCMIT001) - Threat Detection and Monitoring Subsystem

## License
This conservation technology project is licensed under the [GNU GPLv3 License](LICENSE.md) to encourage open collaboration in wildlife protection efforts.

---

**Conservation Impact:** Each deployed nest has the potential to increase hatchling survival rates by 40-60%, contributing significantly to African penguin population recovery efforts.
