# sensor_placement_interpolation_for_parameter_variation
MATLAB implementation of a Sensor Placement Interpolation (SPI) scheme for robust triaxial accelerometer optimal sensor placement under parametric variation. Includes EFI3+, redundancy filtering, and performance evaluation via det(FIM), MAC, and condition number.

# Sensor Placement Interpolation for Parameter Variation

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

This work proposes a **Sensor Placement Interpolation (SPI)** scheme that provides robust triaxial accelerometer layouts under expected parameter variations.
<p align="center">
  <img src="docs/fig_param_variation.jpg" width="700">
</p>

## ⚙️ Algorithm Overview

The proposed SPI framework extends:

- EFI3+ (Effective Independence)
- Redundancy of Information (RoI)
- Scenario-weighted interpolation of Fisher Information

<p align="center">
  <img src="docs/fig_flowchart.jpg" width="850">
</p>

Main steps:

1. Compute target modes under multiple parameter scenarios  
2. Interpolate EFI metrics across scenarios  
3. Remove redundant nodes using RoI  
4. Select robust triaxial sensor locations
