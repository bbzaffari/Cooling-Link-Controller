# Cooling-Link Controller

**Project:** Cooling-Link Controller \
**Platform:** ESP32 (Heltec LoRa version) \
**Environment:** ESP-IDF (ESP32 development framework) \
**Project type:** Final Degree Project (aka in Brazil as TCC — Trabalho de Conclusão de Curso) \

---

## Overview

Cooling-Link Controller is the core embedded system designed for my final engineering project (TCC). It is a cold storage chamber controller that integrates real-time monitoring, control, and remote communication for temperature, humidity, and CO₂ levels.

This system leverages the ESP32 Heltec LoRa board and the ESP-IDF development environment. It was built to explore fine-grained task and thread management using FreeRTOS, with careful use of semaphores, mutexes, and hardware interrupts to optimize interface flow and system responsiveness.

---

## Main Features

* **Real-time environmental monitoring**
  Measures internal (temperature, humidity, and CO₂) and external (temperature and humidity) using sensors like HTU31D.

* **User interaction**
  Local adjustment of sensor thresholds (min/max) through physical buttons (ADD, SUB, ENTER) with debounce handling, short/long press detection, and an OLED display for feedback.

* **Communication protocols**

  * LoRa: Local radio communication between distributed nodes.
  * MQTT over Wi-Fi: Remote telemetry to brokers for supervision and control.

* **Concurrency and flow control**
  FreeRTOS tasks, semaphores, and interrupts ensure robust, responsive operations even under concurrent events (e.g., button presses, sensor updates, communication).

* **Custom utilities and wrappers**
  Includes custom-made utility libraries (e.g., `Utils`) for logging, I²C handling, and structured hardware abstraction.

---

#In the near future, I plan to provide a more detailed explanation of this project, including deeper insights into its architecture, implementation challenges, and potential applications.
