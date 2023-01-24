from multiprocessing import Queue
from threading import Thread
from serial import Serial, SerialException
from time import sleep

BAUDRATE = 115200
MAX_FAILS = 1
FRAMES_AMOUNT = 10
THREAD_TIMEOUT = 0.05

DEBUG = False


class UwbSerialConnection:
    def __init__(self, serial_path="/dev/UWBt") -> None:
        self.serial_path = serial_path
        self.serial = Serial()
        self.connected = False
        self.distances_queue = Queue()
        self.last_value = 0
        self.fails = 0
        self.address_queue = Queue()
        self.last_address = "XX:YY"
        self.alive_ping = Queue()
        self._set_threads()

    def begin(self):
        self._connect()
        self.process.start()
        self.watchdog.start()

    def _set_threads(self):
        self.watchdog = Thread(target=self._watchdog_process, args=(self.alive_ping,))
        self.process = Thread(target=self._thread_process,
                              args=(self.distances_queue, self.address_queue, self.alive_ping,))

    def end(self):
        self._disconnect()
        self._set_threads()

    def restart(self):
        self.end()
        self.begin()
        if DEBUG is True:
            print("RESTARTED")

    def _watchdog_process(self, ping_queue):
        while True:
            # print("WATCHDOG PING")
            if ping_queue.qsize() > MAX_FAILS:
                self.restart()
                return
            sleep(THREAD_TIMEOUT)

    def _thread_process(self, queue, address, ping_queue):
        current_address = "none"
        while True:
            if DEBUG is True:
                print("THREAD ALIVE")
            if ping_queue.qsize() > 0:
                ping_queue.get("X")
            # kajś się tu zawiesza
            if self.address_queue.qsize() > 0:
                current_address = address.get()
                if DEBUG:
                    print("ADDRESS SET: ", current_address)
            message = str(current_address) + str(FRAMES_AMOUNT)
            if self.serial.isOpen() and current_address != "none":
                try:
                    self.serial.write(bytes(message, "ASCII"))
                    for i in range(0, FRAMES_AMOUNT):
                        line = str(self.serial.readline(), encoding="ASCII")
                        try:
                            distance = float(line[8:].strip())
                            if DEBUG:
                                print("I: ", i, "DISTANCE: ", distance)
                            queue.put(distance)
                        except ValueError:  # in case of reciver failure
                            pass
                except(TypeError, SerialException, OSError):
                    if DEBUG:
                        print("SERIAL FAILURE")
                    return
            else:
                self.reconnect()
            sleep(THREAD_TIMEOUT / 2)

    def _connect(self):
        self.serial = Serial(self.serial_path, BAUDRATE)
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

    # this method creates interface between ROS2 and serial connection
    def get_distance(self):
        if self.distances_queue.qsize() > 0:
            self.last_value = self.distances_queue.get()
        return self.last_value

    def set_address(self, _address):
        if _address != self.last_address:
            self.last_address = _address
            self.address_queue.put(self.last_address)
            if DEBUG:
                print("SET_ADDRESSS_CALLED: ", _address)
        self.alive_ping.put("X")


# depreciated
def get_distance_array(serial_path, address, size):
    uwb_serial = Serial(serial_path, BAUDRATE)
    distances = []
    message = address + str(size)
    uwb_serial.write(bytes(message, "ASCII"))
    while size > 0:
        size -= 1
        line = str(uwb_serial.readline(), encoding="ASCII")
        try:
            distance = float(line[8:].strip())
            distances.append(distance)
        except ValueError:  # in case of reciver failure
            pass
    uwb_serial.close()
    return distances


# depreciated
def get_average(distances):
    if len(distances) == 1:
        return
    sum = 0
    n = 0
    for distance in distances:
        sum += distance
        n += 1
    if n == 0:
        return 0  # error
    else:
        return sum / n

# uw_connection = UwbSerialConnection()


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
