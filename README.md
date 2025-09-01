# Visual-Inertial Odometry with Event-Based Camera

This repository contains the implementation of my **Master’s Thesis** on **Visual-Inertial Odometry (VIO) using an event-based camera (DAVIS346) and an Inertial Measurement Unit (IMU)**.  
The work focuses on the development of a complete pipeline for event-driven visual odometry and its integration with inertial sensing through **GTSAM** optimization.

---

## 📖 Project Overview

Event-based cameras asynchronously capture changes in brightness with microsecond latency, offering high temporal resolution and robustness to motion blur and lighting conditions.  
In this project, I combined event-driven vision with IMU preintegration to estimate the camera trajectory in real-time.

The pipeline includes:
- **Event corner detection** using **Arc\***.
- **Feature tracking** with optical flow on time surfaces.
- **Keyframe selection** and **baseline estimation**.
- **Essential matrix decomposition** and relative pose recovery.
- **3D landmark triangulation**.
- **Pose propagation** via `estimateWorldCameraPose` for successive keyframes.
- **Visual-Inertial fusion** with **GTSAM** using:
  - IMU preintegration factors,
  - Visual odometry factors,
  - Bundle adjustment on a sliding window.

The trajectory is optimized in the **ENU world frame** and expressed with respect to the **camera center**.

---

> ⚠️ Note: Large raw datasets are not stored in this repository.  
> Please refer to the [iniVation DAVIS346 datasheet](https://inivation.com/davis346/) and follow the acquisition guidelines described in the thesis.

---

## 🚀 Requirements

- **MATLAB** R2023a or newer  
  Toolboxes:
  - Computer Vision Toolbox  
  - Optimization Toolbox  
- **GTSAM** (tested with v4.2)  
- **Git LFS** (if working with large `.mat` datasets)

---

## ▶️ Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/FrancescoGriguol/master-thesis.git
   cd master-thesis

## 📂 Repository Structure

