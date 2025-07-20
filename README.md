# Cooling-Link Controller

**Project:** Cooling-Link Controller
**Platform:** ESP32 (Heltec LoRa version)
**Environment:** ESP-IDF (ESP32 development framework)
**Project type:** Final Degree Project (Brazilian TCC — Trabalho de Conclusão de Curso)

---

## Overview

Cooling-Link Controller is the core embedded system designed for my final engineering project (TCC). It is a cold storage chamber controller that integrates real-time monitoring, control, and remote communication for temperature, humidity, and CO₂ levels.

This system leverages the ESP32 Heltec LoRa board and the ESP-IDF development environment. It was built to explore fine-grained task and thread management using FreeRTOS, with careful use of semaphores, mutexes, and hardware interrupts to optimize interface flow and system responsiveness.

---

## Main Features

* **Real-time environmental monitoring**
  Measures internal and external temperature, humidity, and CO₂ using sensors like HTU31D.

* **User interaction**
  Local adjustment of sensor thresholds (min/max) through physical buttons (ADD, SUB, ENTER) with debounce handling, short/long press detection, and an OLED display for feedback.

* **Communication protocols**

  * LoRa: Local radio communication between distributed nodes.
  * MQTT over Wi-Fi: Remote telemetry to brokers for supervision and control.

* **Concurrency and flow control**
  Fine-tuned FreeRTOS tasks, semaphores, and interrupts ensure robust, responsive operations even under concurrent events (e.g., button presses, sensor updates, communication).

* **Custom utilities and wrappers**
  Includes custom-made utility libraries (e.g., `Utils`) for logging, I²C handling, and structured hardware abstraction.

---

## Technical Highlights

* Written entirely in C, with modular design and clear separation between drivers, tasks, and communication layers.
* Uses HTU31D wrapper for simplified sensor integration.
* Relies on ESP32 hardware features like hardware interrupts (`IRAM_ATTR`) and dual-core tasking.
* Incorporates a state machine (FSM) for configuration modes and user interaction flow.
* Sends structured status words (e.g., "011010") representing environmental conditions.

---

## Project Context

This project was developed as part of my final undergraduate thesis in Computer Engineering, aiming to combine embedded systems, IoT protocols, and automation principles into a practical, dependable solution.

While designed for a family-operated fruit storage business, the architecture is flexible and can be adapted to other environmental control systems requiring wireless monitoring and threshold-based automation.

---

## How It Works (Brief)

1. **Startup:** Initializes drivers, sensors, display, LoRa, Wi-Fi, MQTT, and sets up tasks and interrupts.
2. **Monitoring loop:** Periodically updates sensor readings, calculates dew point, displays data, and sends updates via MQTT.
3. **User configuration:** Via button interface, users can adjust sensor limits, stored temporarily, then committed.
4. **Event handling:** Button presses and door state changes trigger interrupt handlers, notifying tasks via FreeRTOS mechanisms.
5. **Communication:** LoRa packets handle local data exchange; MQTT handles cloud communication.

---

## Future Improvements

* Add persistent storage (e.g., NVS) to save settings across reboots.
* Implement TLS for secure MQTT communication.
* Extend CO₂ integration with active sensing hardware.
* Add watchdog timers for enhanced fault tolerance.

---
