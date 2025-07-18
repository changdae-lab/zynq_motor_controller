# Zynq Motor Controller  

**Real-time Motor Control System using Xilinx Zynq SoC (ARM + FPGA Heterogeneous Computing)**  

## Overview

This repository implements a **real-time motor controller** for robotic systems using the **Xilinx Zynq-7000 SoC (e.g., ZYBO Z7-20)**.  

- **ARM Cortex-A9 (PS):** High-level decision, trajectory generation, UART CLI  
- **FPGA fabric (PL):** Low-latency, hard real-time PID/Fuzzy motor control  
- **AXI4-Lite:** PS-PL data exchange  
- **Target:** Maxon DC motors with incremental encoders  

This architecture enables scalable, modular motor control for research and prototyping in robotic systems.

## Motivation

Modern robotic systems require **hard real-time motor control with minimal latency**, while maintaining **high-level flexibility for trajectory planning and runtime gain tuning**.

This project was developed to:  
- Reduce control loop latency using FPGA hardware acceleration  
- Enable **runtime PID gain tuning via UART CLI**  
- **Automatically log experiment data to CSV for post-experiment analysis**  
- Evaluate trajectory generation and fuzzy-based overshoot reduction  

## Architecture
#### FPGA Motor Controller Overall System Diagram 
![image](https://github.com/user-attachments/assets/83a3e31f-21b4-401d-a933-dd0cc1a0d481)

#### PS/PL Data Flow Diagram 
![image](https://github.com/user-attachments/assets/9d40ab3c-9921-4561-a376-80ff315d8f0d)
