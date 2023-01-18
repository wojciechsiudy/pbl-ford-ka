from multiprocessing import Process, Queue, Pipe
from threading import Timer
from serial import Serial

baudrate = 115200
MAX_FAILS = 10
FRAMES_AMOUNT = 10
THREAD_TIMEOUT = 1.0

class UwbSerialConnection:
    def __init__(self, serial_path="/dev/UWBt") -> None:
        self.serial_path = serial_path
        self.serial = Serial()
        self.connected = False
        self.distances_queue = Queue()
        self.last_value = 0
        self.fails = 0
        self.address_parent, self.address_child = Pipe()
        self.address = Queue()
        self.last_address = "XX:YY"
        self.alive_ping = Queue()
        self.watchdog = Timer(THREAD_TIMEOUT, self._watchdog_process, args=(self.alive_ping,))
        self.process = Process(target=self._thread_process, args=(self.distances_queue, self.address, self.alive_ping,))

    def begin(self):
        self._connect()
        self.process.start()
        self.watchdog.start()

    def end(self):
        self.watchdog.cancel()
        self.process.join()
        self._disconnect()

    def restart(self):
        self.end()
        self.begin()
        print("RESTARTED")

    #it is not working, thread process talks too quick
    def _watchdog_process(self, ping_queue):
        if ping_queue.qsize() > 0:
            ping_queue.get()
        else:
            self.restart()


    def _thread_process(self, queue, address, ping_queue):
        current_address = "none"
        while True:
            print("THREAD ALIVE")
            if current_address != "none":
                ping_queue.put("X")
            #kaś się tu zawiesza
            if self.address.qsize() > 0:
                current_address = address.get()
                print("ADDRESS SET: ", current_address)
            message = str(current_address) + str(FRAMES_AMOUNT)
            if self.serial.isOpen() and current_address != "none":
                self.serial.write(bytes(message, "ASCII"))
                for i in range (0, FRAMES_AMOUNT):
                    line = str(self.serial.readline(), encoding="ASCII")
                    try:
                        distance = float(line[8:].strip())
                        print("I: ", i, "DISTANCE: ", distance)
                        queue.put(distance)
                    except(ValueError): #in case of reciver failure
                        pass
            else:
                self.reconnect()

            
                


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
            self.distances_queue.put("HI")
            time.sleep(1)
            MAX_PUT -= 1

    # this method creates interface between ROS2 and serial connection
    def get_distance(self):
        if self.distances_queue.qsize() > 0:
            self.last_value = self.distances_queue.get()
        return self.last_value
    
    def set_address(self, _address):
        self.address_parent.send(_address)
        if _address != self.last_address:
            self.last_address = _address
            self.address.put(self.last_address)
        print("SET_ADRESSS_CALLED: ", _address)

    

    
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