# 🚁 NIDAR — Survivor Detection & Autonomous Geotagging System

> **National-Level Competition Project | Drone Federation of India (DFI)**

NIDAR is an autonomous drone system designed to detect survivors in disaster zones, geotag their GPS coordinates, and transmit real-time telemetry — all without human intervention.

---

## 🎯 What It Does

| Feature | Description |
|---|---|
| 👤 Survivor Detection | YOLOv8/v11-based real-time human detection onboard |
| 📍 Geotagging | Timestamped GPS coordinates logged for each detection |
| 📡 Telemetry | Live data transmission including heading, altitude, and location |
| 🤖 Autonomous Navigation | Pixhawk-based waypoint execution and altitude control |
| 🔭 LiDAR Sensing | Stabilized LiDAR pipeline for obstacle awareness and terrain mapping |

---

## 🛠️ Tech Stack

- **Hardware:** Raspberry Pi 5, Pixhawk Flight Controller, LiDAR sensor
- **Computer Vision:** YOLOv8 / YOLOv11 (Ultralytics)
- **Communication:** MAVLink protocol (custom implementation)
- **Middleware:** DroneKit, Mission Planner
- **Language:** Python

---

## 🏗️ System Architecture

```
┌─────────────────┐       MAVLink        ┌──────────────────┐
│   Raspberry Pi 5│◄────────────────────►│  Pixhawk FC      │
│                 │                       │                  │
│  ┌───────────┐  │                       │  - Waypoints     │
│  │  YOLOv8   │  │                       │  - Altitude ctrl │
│  │ Detection │  │                       │  - GPS fusion    │
│  └───────────┘  │                       └──────────────────┘
│  ┌───────────┐  │
│  │  LiDAR    │  │       Telemetry
│  │ Pipeline  │  │──────────────────────► Ground Station
│  └───────────┘  │
└─────────────────┘
```

---

## 🚀 Key Implementations

- **YOLOv8 onboard inference** — optimized for Raspberry Pi with real-time detection pipeline
- **Custom MAVLink handler** — timestamped GPS log generation, heading estimation, packet parsing
- **LiDAR stabilization** — continuous packet parsing with noise filtering for reliable indoor/outdoor sensing
- **Multi-sensor fusion** — combining camera, LiDAR, and GPS data for robust situational awareness
- **Autonomous mission flow** — full waypoint execution, loiter, and return-to-home without manual input

---

## 📸 Demo

> *(Photos/videos from testing will be added here)*

---

## 🏆 Competition

This project was part of an  national-level drone competition organized by the **Drone Federation of India (DFI)**. The system is designed to operate in real-world post-disaster scenarios with multi-environment robustness requirements.

---

## 👤 Author

**Tanish Nagarkar**
[LinkedIn](https://linkedin.com/in/tanish-nagarkar-768384251) | [Email](mailto:tanishnagarkar@gmail.com)
