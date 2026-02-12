# Sensor Placement Interpolation for Parameter Variation
MATLAB implementation of a Sensor Placement Interpolation (SPI) scheme for robust triaxial accelerometer optimal sensor placement under parametric variation. Includes EFI3+, redundancy filtering, and performance evaluation via det(FIM), MAC, and condition number.

<p align="center">
  <img src="docs/fig_overview.jpg" width="800">
</p>

## 📌 Related Publication

This repository provides MATLAB codes for the following paper:

**Chanwoo Lee**, Youjin Kim, and Hyung-Jo Jung (2025)  
*Sensor placement interpolation scheme for modal identification under parametric variation in infrastructure*  
Structure and Infrastructure Engineering.  
DOI: 10.1080/15732479.2025.2579824

## 🎯 Motivation

Optimal sensor placement (OSP) is essential for efficient structural health monitoring (SHM).
However, conventional deterministic methods often fail to remain optimal when structural parameters vary, such as:

- stiffness degradation (damage scenarios)
- failure-mode evolution
- uncertainty in boundary conditions

This work proposes a Sensor Placement Interpolation (SPI) scheme that provides robust triaxial accelerometer layouts under expected parameter variations.
The proposed approach combines discrete Effective Independence (EFI)-based sensor placement results across multiple parameter points and interpolates them to achieve a robust Forward Sequential Sensor Placement (FSSP) strategy.

<p align="left">
  <img src="figures/overview.png" width="700">
</p>

## ⚙️ Algorithm Overview

The proposed SPI framework extends:

- **EFI3+** (Effective Independence for forward sequential placement of triaxial accelerometers)
- **Redundancy of Information (RoI)** for eliminating redundant sensor nodes

together with scenario-weighted interpolation of Fisher Information across parameter variations.


Workflow:

- **Input** → Target modes (Φ_k), scenario weights (w_k), redundancy threshold (R_th), and final sensor number (N_sensor).  
- **1st sensor placement** → Compute and interpolate scenario-dependent importance metrics to select the first optimal node.  
- **Initial placement** → Sequentially add sensors until the reduced FIM (Q_0) achieves full rank across all scenarios.  
- **Final placement** → Complete **FSSP** by interpolating EFI3+ scores and removing redundant nodes using RoI filtering.  
- **Output** → Robust triaxial accelerometer layout optimal under expected parametric variations.

<p align="left">
  <img src="figures/flow chart.png" width="500">
</p>

