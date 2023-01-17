from multiprocessing import Process, Queue, Pipe
from serial import Serial
import time

baudrate = 115200
MAX_FAILS = 10
FRAMES_AMOUNT = 10

class UwbSerialConnection:
    def __init__(self, serial_path="/dev/UWBt") -> None:
        self.serial_path = serial_path
        self.serial = Serial()
        self.connected = False
        self.distancesQueue = Queue()
        self.lastValue = 0
        self.fails = 0
        self.address_parent, self.address_child = Pipe()
        self.process = Process(target=self._thread_process, args=(self.distancesQueue, self.address_child,))

    def begin(self):
        self._connect()
        self.process.start()

    def end(self):
        self.process.join()
        self._disconnect()

    def _thread_process(self, queue, address):
        while True:
            message = str(address.recv()) + str(FRAMES_AMOUNT)
            self.serial.write(bytes(message, "ASCII"))
            for i in range (0, FRAMES_AMOUNT):
                line = str(self.serial.readline(), encoding="ASCII")
                try:
                    distance = float(line[8:].strip())
                    queue.put(distance)
                except(ValueError): #in case of reciver failure
                    pass

            
                


    def _connect(self):
        self.serial = Serial(self.serial_path, baudrate)
        self.connected = True
        
    def _disconnect(self):
        self.serial.close()
        self.connected = False

    def reconnect(self):
        self.fails = 0
        self._disconnect()
        self._connect()

    def _test_connection(self):
        if self.serial.isOpen():
            self.connected = True
        else:
            self.connected = False

    def _dummy_thread_function(self):
        MAX_PUT = 3
        while (MAX_PUT > 0):
            self.distancesQueue.put("HI")
            time.sleep(1)
            MAX_PUT -= 1

    # this method creates interface between ROS2 and serial connection
    def get_distance(self):
        if self.distancesQueue.qsize() > 0:
            self.lastValue = self.distancesQueue.get()
        return self.lastValue
    
    def set_address(self, _address):
        self.address_parent.send(_address)

    

    
#depreciated
def getDistanceArray(serialPath, address, size):
    uwb_serial = Serial(serialPath, baudrate)
    distances = []
    message = address + str(size)
    uwb_serial.write(bytes(message, "ASCII"))
    while (size > 0):
        size -= 1
        line = str(uwb_serial.readline(), encoding="ASCII")
        try:
            distance = float(line[8:].strip())
            distances.append(distance)
        except(ValueError): #in case of reciver failure
            pass
    uwb_serial.close()
    return distances

#depreciated
def getAverage(distances):
    if len(distances) == 1:
        return
    sum = 0
    n = 0
    for distance in distances:
        sum += distance
        n += 1
    if (n == 0):
        return 0 #error
    else:
        return (sum / n)


#uw_connection = UwbSerialConnection()


# MULTITHREAD QUEUE SANDBOX
# connection = UwbSerialConnection()
# print("VALS before start:")
# print(connection.distancesQueue.qsize())
# p = Process(target=connection._dummy_thread_function)
# p.start()
# for i in range (1, 10):
#     print(connection.get_distance())
#     time.sleep(1)
# p.join()
# print(connection.distancesQueue.qsize())