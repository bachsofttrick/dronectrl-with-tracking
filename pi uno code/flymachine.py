from time import time, sleep
from threading import Thread
import serial
import socket

class UDPMsg:
    def __init__(self, UDP_PORT):
        self.shutdown_thread = False
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", UDP_PORT))
        self.string = None
        self.in_contact = False
        self.received = False
        self._udp_timer = 0

    def start(self):
        self._thread = Thread(target=self.recvmes, args=(), daemon=True)
        self._thread.start()
        return self

    def recvmes(self):
        while not self.shutdown_thread:
            self.received = False
            data, addr = self.sock.recvfrom(1024)
            self.received = True
            self._udp_timer = time()
            self.string = data
            self.string_print = data.decode()

    def check_msg(self):
        if (not self.received and (time() - self._udp_timer >= 2)):
            self.in_contact = False
        else:
            self.in_contact = True
        return self.in_contact

    def close(self):
        self.shutdown_thread = True
        if self._thread.daemon == False:
            self._thread.join()
            self.sock.close()

get_text = UDPMsg(5005).start()
por = serial.Serial('/dev/ttyACM0',9600)
sleep(1)

try:
    while True:
        if get_text.check_msg():
            print(time(), get_text.string_print)
            strsend = get_text.string
            #strsend = bytes(get_text.string_print,'ascii')
        else:
            #print('send default. ')
            strsend = bytes('8888','ascii')
        por.write(strsend)
        por.flush()
        if por.in_waiting > 0:
            strrecv = por.readline().decode('ascii','ignore')
            print(strrecv)
        #print('sleep')
        sleep(0.05)
except KeyboardInterrupt:
    get_text.close()
    print('exit.')
