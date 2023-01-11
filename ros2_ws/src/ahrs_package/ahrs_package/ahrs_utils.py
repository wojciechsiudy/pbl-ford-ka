import math
from serial import Serial

class MeasureValue:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        
class Measure:
	def __init__(self, magn, gyro, accel):
		self.magn = magn
		self.gyro = gyro
		self.accel = accel

def ahrs_data_to_point(data):
	return Measure(MeasureValue(data[1],data[2],data[3]), MeasureValue(data[4],data[5],data[6]), MeasureValue(data[7],data[8],data[9])) #todo

def get_data():
    ahrs_serial = Serial("/dev/ttyS1", 115200) #
    while (True):
        try:
            line = str(ahrs_serial.readline(), encoding="ASCII")
            if "AHRS" in line:
                data = line.split(';')
                return ahrs_data_to_point(data)
        except(UnicodeDecodeError):
            pass
