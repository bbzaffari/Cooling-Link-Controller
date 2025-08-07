# Cooling-Link Controller

**Status:** [![status: stable | refactoring in progress](https://img.shields.io/badge/status-stable%20%7C%20refactoring%20in%20progress-green.svg)](https://github.com/bbzaffari/Cooling-Link-Controller)

**Project:** Cooling-Link Controller \
**Platform:** ESP32 (Heltec LoRa version) \
**Environment:** ESP-IDF (ESP32 development framework) \
**Project type:** [Final Project Thesis(TCC – Trabalho de Conclusão de Curso, in Brazil)](https://github.com/bbzaffari/TCC-Final-Project-Thesis)

---

## Overview

Cooling-Link Controller is the core embedded system designed for my final engineering project (TCC). It is a cold storage chamber controller that integrates real-time monitoring, control, and remote communication for temperature, humidity, and CO2 levels.

This system leverages the ESP32 Heltec LoRa board and the ESP-IDF development environment. It was built to explore fine-grained task and thread management using FreeRTOS, with careful use of semaphores, mutexes, and hardware interrupts to optimize interface flow and system responsiveness.

​ 
---


- [Core Structure](#core-structure)
- [Repository Structure](#repository-structure)
- [Clone](#clone)
---
## Main Features

* **Real-time environmental monitoring**
Receives internal (temperature, humidity, and CO2), through a `LoRa–Proto`, and external (temperature and humidity) using sensors like HTU31D.

* **User interaction**
  Local adjustment of sensor thresholds (min/max) through physical buttons (ADD, SUB, ENTER) with debounce handling, short/long press detection, and an OLED display for feedback.

* **Communication protocols**

  * LoRa–Proto: Protocol developed on top of LoRa(PHY).
  * MQTT over Wi-Fi: Remote telemetry to brokers for supervision and control.

* **Concurrency and flow control**
  FreeRTOS tasks, semaphores, and interrupts ensure robust, responsive operations even under concurrent events (e.g., button presses, sensor updates, communication).

* **Custom utilities and wrappers**
  Includes custom-made utility libraries (e.g., `Utils`) for logging, I2C handling, and structured hardware abstraction.

---

## Core Structure

This module defines the **core data structures** for configuring and managing sensor parameters (like TEMP, HUM, CO2) in a clean and scalable way.
- ***`sConfigActivation`***
- ***`sConfigElement`***

```c
typedef struct {//--- configs[]
    const char* name;                // Sensor group name (e.g., "TEMP")
    sConfigElement elements[4];      // Parameters to configure (e.g., MAX, MIN, relays)
    uint32_t repeat;                // Delay (ms) between auto-increments during long press
    uint8_t elements_count;         // Number of valid elements in the array
} sConfigActivation;

typedef struct {//-- ConfigElements
    const char* name;     // Human-readable label (e.g., "MAX", "RELAY MIN")
    float value;          // Current value of this element (modifiable)
    float* MAX;           // Optional upper bound (enforced at runtime)
    float* MIN;           // Optional lower bound
    float pacing;         // Step used when adjusting via physical buttons
} sConfigElement;

```
### Sensor:

Each entry in ***`configs`*** corresponds to a sensor, such as:
- ***`TEMP`*** for temperature 
- ***`HUM`*** for humidity 
- ***`CO2`*** for carbon dioxide concentration 

```c
// === Structural Comp. =============================================----------
static sConfigActivation configs[SENSORS_COUNT] = {
    {
        .name = "TEMP",
        .elements = { ... },
        .repeat = ...,
        .elements_count = ...
    },
    {
        .name = "HUM",
        .elements = { ... },
        .repeat = ...,
        .elements_count = ...
    },
    {
        .name = "CO2",
        .elements = { ... },
        .repeat = ...,
        .elements_count = ...
    }
};
```

### Inside each sensor:

We define four elements:
1. **MAX**: maximum threshold
2. **RELAY MAX**: A relay control flag for the max limit
3. **MIN**: A minimum threshold
4. **RELAY MIN**: A relay control flag for the min limit

Each element is defined using:

```c
.elements = {
            // NAME:| value        |MAX pointer |MIN pointer| pacing|
            { "MAX", MAX_LIMIT_TEMP, &MAX_LIMIT_TEMP, NULL, 0.5f },
            { "RELAY MAX", 0, &ONE, &ZERO, 1},
            { "MIN", MIN_LIMIT_TEMP, NULL, &MIN_LIMIT_TEMP, 0.5f },
            { "RELAY MIN", 0, &ONE, &ZERO, 1}
````

### Why this Matters:
1. `"MAX"`/`"MIN"` entries define when alerts or actions (like ventilation or dehumidifiers) should trigger. 
2. `"RELAY {MAX, MIN}"` act as switches — enabling or disabling control actions when those thresholds are crossed.
3. Pacing defines how finely the user can adjust the value (e.g., +0.5°C, +50 ppm).
4. The system traverses these elements when the user enters "configuration mode", using buttons for ADD/SUB and ENTER.

> The configuration FSM (tConfig) knows only how to:
> 1. Select one sConfigElement at a time
> 2. Modify its .value using .pacing
> 3. Save it after user confirmation
> 4. Use its .MAX and .MIN *pointers* to enforce safe bounds

### Data-Driven Design
Instead of hardcoding logic for each sensor and each variable, this array allows the control logic to be generalized.
  
> This abstraction enables:
> 1. Code reusability
> 2. Safe configuration
> 3. Dynamic display
> 4. External interface expansion (e.g., MQTT-driven updates)

## Example: TEMP

```c
// === Structural Comp. =============================================----------
static sConfigActivation configs[SENSORS_COUNT] = {
    {
        .name = "TEMP",
        .elements = {
            // NAME:| value         |MAX            | MIN| pacing|
            { "MAX", MAX_LIMIT_TEMP, &MAX_LIMIT_TEMP, NULL, 0.5f },
            { "RELAY MAX", 0, &ONE, &ZERO, 1},
            { "MIN", MIN_LIMIT_TEMP, NULL, &MIN_LIMIT_TEMP, 0.5f },
            { "RELAY MIN", 0, &ONE, &ZERO, 1}
        },
        .repeat = REPEAT_MS_TEMP,
        .elements_count = 4
    },
    { ... },
    { ... }
};
```

This tells the system:
- The temperature MAX limit starts at 80°C and cannot go above that
- The MIN limit is 2°C and cannot go below that
- Relay activation is enabled/disabled via binary flags (RELAY {MAX, MIN})
- Values are adjusted in 0.5°C increments using the physical interface

## Repository Structure
Below is an overview of the key folders and components in this project:

>Cooling-Link-Controller/ \
>├── components/ \
>│   ├── [`htu31`](https://github.com/bbzaffari/HTU-31D-ESP-IDF-C)/          // HTU31D sensor driver (fully implemented for ESP-IDF) \
>│   ├── [`lora`](https://github.com/bbzaffari/lora-phy)/           // Base LoRa PHY communication layer (adapted) \
>│   ├── [`lora_proto`](https://github.com/bbzaffari/LoRa-Protocol-ESP-IDF)/     // Custom protocol over LoRa PHY (developed for this project) \
>│   ├── [`ssd1306`](https://github.com/bbzaffari/esp-idf-ssd1306-Minimal-Version)/        // SSD1306 OLED driver (adapted from nopnop2002) \
>│ \
>├── main/ \
>│   ├── main.c         // Entry point: system control loop, tasks and config \
>│   ├── Utils.c/.h      // Utility functions (Wrappers, common tools) \
>│   ├── CMakeLists.txt \
>│ \
>├── .gitmodules         // Git submodules configuration \
>├── README.md \
>├── sdkconfig           // ESP-IDF config file \
>└── CMakeLists.txt      // Project build definition \

## Clone
All the modules listed above are required for proper system operation, as they implement core sensor drivers, communication protocols, and peripheral interfaces. To clone the complete functional project with all its dependencies, including submodules, use the following command:
```bash
git clone --recursive https://github.com/bbzaffari/Cooling-Link-Controller
```
This will ensure that all necessary components are pulled together for seamless compilation and deployment using ESP-IDF.

---
