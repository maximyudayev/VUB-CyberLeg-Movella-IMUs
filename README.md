# CyberLeg External IMU Sensing 

## Installation

1. Clone the repo on the host PC to which Movella DOTs will be connected.
2. Install Anaconda or miniconda on the host PC.
3. Install [Movella SDK](https://www.movella.com/support/software-documentation) on the host PC.
4. Create Conda environment for the project:
```bash
conda create -n cyberleg-external-imu python=3.10 numpy=1.26
```
5. Activate the environment
```bash
conda activate cyberleg-external-imu
```
6. Install Movella SDK into the created Conda environment. 
```bash
cd <PATH-TO-INSTALL-FOLDER-LIKE: C:\Program Files\Movella\DOT PC SDK 2023.6\SDK Files\Python\x64>

pip install movelladot_pc_sdk-2023.6.0-cp310-none-win_amd64.whl
```

Next:
To log data on the sbRIO, make another loop in LabView that sends packets to 'localhost'/127.0.0.1, and have a Python script run on the Linux shell of the sbRIO, that writes the captured UDP data to a file that's then downloaded from it.
