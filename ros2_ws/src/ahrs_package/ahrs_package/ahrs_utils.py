import math
from serial import Serial
from multiprocessing import Queue
from threading import Thread
from time import sleep
from std_msgs.msg import Float32
from ahrs_interface.msg import MeasureValue, Measure
      
        
class AhrsConnection:
    def __init__(self, serial_path="/dev/AHRS"):
        self.ahrs_serial = Serial(serial_path, 115200)
        self.measures_queue = Queue()
        self.last_value = Measure()
        self._set_threads()
        
    def begin(self):
        self.process.start()
        
    def _set_threads(self):
        self.process = Thread(target=self._thread_process, args=(self.measures_queue,))
        
    def _thread_process(self, queue):
        while True:
            try:
                line = str(self.ahrs_serial.readline(), encoding="ASCII")
                if "AHRS" in line:
                    data = line.split(';')
                    queue.put(self.ahrs_data_to_point(data))
            except:
                print("readErr")

    def end(self):
        self._set_threads()
        
        
    def getLastValue(self):
        if self.measures_queue.qsize() > 0:
            self.last_value = self.measures_queue.get()
        return self.last_value
        
    def ahrs_data_to_point(self, data):
        a = MeasureValue()
        g = MeasureValue()
        m = MeasureValue()
        measure = Measure()
        try:
            m.x = float(data[1])
            m.y = float(data[2])
            m.z = float(data[3])
            g.x = float(data[4])
            g.y = float(data[5])
            g.z = float(data[6])
            a.x = float(data[7])
            a.y = float(data[8])
            a.z = float(data[9])
            measure.magn = m
            measure.gyro = g
            measure.accel = a
        except:
            pass    
        return measure         



