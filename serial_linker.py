import serial
import os

#require sudoers configuration
#<username> ALL=(root) NOPASSWD: /usr/bin/ln, /usr/bin/unlink
#plus adding <username> to dialout group

if(not os.path.exists("/dev/ttyUSB0")):
    print("No USB devices connected. Aborting.")

#read list of connected serials
serials = os.popen("ls /dev/ttyUSB*")

#remove existing symlinks
os.system("if [ -L /dev/AHRS ]; then sudo unlink /dev/AHRS; fi")
os.system("if [ -L /dev/UWBfront ]; then sudo unlink /dev/UWBfront; fi")

for device in serials:
    serialPath = device.strip()
    #get portion of output
    serialTmp= serial.Serial(serialPath, 115200)
    line = str(serialTmp.readline(), encoding="ASCII")
    print(line) #debug
    #make symlinks of recognized devices
    if "AHRS" in line:
        os.system("sudo ln -s {} /dev/AHRS".format(serialPath))
    elif "DIST" in line:
        os.system("sudo ln -s {} /dev/UWBfront".format(serialPath))

