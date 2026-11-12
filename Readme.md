# Automated hBN Wet Exfoliation via Dobot Magician

This repository contains the Python control software for automating the wet exfoliation of hexagonal Boron Nitride (hBN) using a Dobot Magician robotic arm. 
## Sources
**pydobot documentation**  https://github.com/luismesas/pydobot

## Prerequisites
Currently the set up instructions are for Ubuntu, but workflow should be similar on other operating systems.

## Environment Setup

### 1. Grant Serial Port Permissions
By default, Ubuntu restricts access to serial ports. To allow the software to communicate with the Dobot via USB-to-serial, you must add your user account to the `dialout` group.

Open your terminal and run:
```bash
sudo usermod -a -G dialout $USER
```
**Important:** You must log out of Ubuntu and log back in (or restart your machine) for these permission changes to take effect.

### 2. Identify the Robot's Port
Connect the Dobot Magician via USB and turn it on. Find its assigned port by running:
```bash
ls /dev/ttyUSB*
```
Note the output (typically `/dev/ttyUSB0`). You will need to specify this port in the Python scripts.

### 3. Create the Virtual Environment
We use a standard Python `venv` to isolate the project's dependencies. Clone this repository or create your own directory and set up a python environment

```bash
# Clone this repository
git clone *current link to the repo*
cd repo_directory

# Create the virtual environment
python3 -m venv .venv

# Activate the environment
source .venv/bin/activate
```

### 4. Install Dependencies
With the virtual environment active, install the required Python libraries. This project relies on `pydobot` for native Python serial communication with the arm.

```bash
pip install pydobot pyserial
```

## Usage

Ensure your virtual environment is active (`source .venv/bin/activate`), then launch the web control interface:
```bash
python exfoliation_ui.py
```
This opens the local dashboard at `http://127.0.0.1:8080`, where you can:
- Connect to `/dev/ttyUSB0` (or enable **Simulation Mode** to preview the sequence without hardware).
- Set the **Dipping Speed**, **Arm Angle During Dipping ($\Delta R$)**, **Liquid Surface Height ($Z_{\text{surface}}$)**, and **Immersion Depth ($\Delta Z_{\text{dip}}$)**.
- Configure the **Sample Pickup Position** next to the dipping station and capture coordinates directly from the robot's current pose.

To run the standalone command-line demo script directly:
```bash
python test_demo.py
```

## Process Overview: hBN Wet Exfoliation

* To be included 

## Roadmap
- [x] Initial hardware communication and environment setup
- [x] Web UI for wet exfoliation parameter control and sample pickup/return routine
