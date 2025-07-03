# Zynq Motor Controller
Real-time Motor Control System using Xilinx Zynq SoC (ARM + FPGA Heterogeneous Computing)

## Overview
This repository implements a real-time motor controller for robotic systems using the Xilinx Zynq-7000 SoC (e.g., ZYBO Z7-20), leveraging:

- ARM Cortex-A9 (PS) for high-level decision, trajectory generation, and communication (UART/ROS)
- FPGA fabric (PL) for low-level, hard real-time motor control (PI/PID control, PWM generation, encoder decoding)
- AXI4-Lite bus for PS-PL communication

The system is designed to control Maxon DC motors with incremental encoders, supporting modular, scalable motor control

## Features
✅ Hybrid ARM(PS)-FPGA(PL) (Zynq) architecture
✅ Low-latency, hard real-time control in FPGA
✅ AXI4-Lite register interface for PS-PL data exchange
✅ UART CLI for runtime gain tuning and desired position control
✅ Fixed-point PID controller (Q format) implemented in Verilog
✅ Overshoot detection using FSM with fuzzy gain adjustment (optional)
✅ Quintic trajectory generation on ARM (optional)
✅ SD card logging for experiment data
