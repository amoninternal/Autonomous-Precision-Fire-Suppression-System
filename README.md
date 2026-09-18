# Sentry Flare — Autonomous Precision Fire Suppression System

> **Compact. Visible. Reliable.** An IoT-enabled, closed-loop fire suppression turret designed to neutralize fires at their source while eliminating collateral water damage in high-value environments.

---

## Overview

Traditional fire sprinkler systems are open-loop systems: when a thermal element bursts, water sprays indiscriminately across the entire room. In environments housing high-value assets—such as **data centers, server rooms, laboratories, and archives**—water damage often results in far greater financial loss and operational downtime than the fire itself.

**Sentry Flare** addresses this challenge through an autonomous, dual-axis turret mechanism driven by an **ESP32 microcontroller**. By integrating multi-sensor directional flame detection with closed-loop PD motor control, Sentry Flare continuously monitors for fire, calculates the precise azimuth of the source, aims its nozzle, and fires a targeted stream of water only when locked on target.

---

## Key Features

* **360° Real-Time Fire Detection:** Uses a 4-quadrant flame sensor array (Front, Right, Back, Left) to continuously monitor the surrounding environment.
* **Vector-Based Azimuth Calculation:** Computes continuous target headings using weighted trigonometric vector summation, going beyond simple discrete quadrant targeting.
* **360° Field-of-View via Flip Kinematics:** Leverages dynamic Pan ($0^\circ - 180^\circ$) and Tilt ($55^\circ / 125^\circ$) modes to cover a full $360^\circ$ physical range without mechanical entanglements
* **PD Control Motion Smoothing:** Exponential easing and deadband thresholds prevent servo jitter, overshoot, and mechanical wear
* **Aim-Verification Pump Safety:** The water pump activates *only* after the turret maintains lock on the target within defined angular tolerances ($\pm 5^\circ$ Pan, $\pm 6^\circ$ Tilt) for a set duration ($300\text{ ms}$)
* **Hysteresis Protection:** Dual-threshold detection prevents rapid cycling near sensor boundary states.

---
