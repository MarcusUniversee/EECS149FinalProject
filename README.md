# EECS149FinalProject

To install the libraries necessary

1. Create a virtual environment: `python -m venv venv`

or if another python version is needed, find the path of your python installation. For example, on Marcus's computer:
`C:\Users\clubb\AppData\Local\Programs\Python\Python312\python.exe -m venv venv`

2. Activate venv. On Windows: `venv\scripts\activate`

3. Install required libraries (only need to do this once): `pip install -r requirements.txt`

## How to send commands to the robot

Look at test_command.py for an example of how to send a command. This file sends the command "drive forward" through bluetooth.
DO NOT change or delete the MAC or UUID constants.