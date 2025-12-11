# EECS149FinalProject

## Main setup

To install the libraries necessary. 

1. Create a virtual environment: `python -m venv venv`

or if another python version is needed, find the path of your python installation. For example, on Marcus's computer:
`C:\Users\clubb\AppData\Local\Programs\Python\Python312\python.exe -m venv venv`

2. Activate venv. On Windows: `venv\scripts\activate`

3. Install required libraries (only need to do this once): `pip install -r requirements.txt`

4. Set the camera ID to the camera id of the webcam inside `autonomous_navigation.py`

5. `python autonomous_navigation.py -f [WAYPOINT YAML] -l [LOG FILE]` or `python autonomous_navigation.py` for just mouse GUI navigation

## Setup for deploying code onto polulu

1. Make sure you are on WSL outside the repo. OR use a linux machine/VM

2. install necessary C libraries (if you haven't already)
```
sudo apt update
sudo apt install -y git cmake build-essential libusb-1.0-0-dev
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi
```

3. Go/clone into the repo and Get the pico-sdk (if you haven't already): `git submodule update --init --recursive`

4. Get picotool. Replace PATHTOPICO_SDK with the file path to the pico-sdk inside the repo. It should look something like `/home/marcusuniverse/EECS149FinalProject/pico-sdk`. Do these commands one at a time
```
cd ~
git clone https://github.com/raspberrypi/picotool.git
cd picotool
mkdir build
cd build
cmake .. -DPICO_SDK_PATH=PATHTOPICO_SDK
make
sudo make install
```

5. WINDOWS/WSL only: In a separate **administrator** windows terminal, reroute usb connections to wsl. Do these commands one at a time
```
winget install usbipd
usbipd list
```

You should see something like 
`4-3    2e8a:0003  USB Mass Storage Device, RP2 Boot                             Not shared`

Note down the first part (4-3), and do (REPLACE 4-3 with whatever you see):
```
usbipd bind --busid 4-3
usbipd attach --wsl --busid 4-3
```

Note: You may have to redo the command:
`usbipd attach --wsl --busid 4-3`

6. Verify you see the robot: `lsusb`

## Deploying lingua franca code

1. Compiling:
```
lfc src/FILENAME.lf
```

2. Load into robot (BOOTSEL first):
```
sudo picotool load -x bin/FILENAME.elf
```

## How to send commands to the robot

Look at test_command.py for an example of how to send a command. This file sends the command "drive forward" through bluetooth.
DO NOT change or delete the MAC or UUID constants.