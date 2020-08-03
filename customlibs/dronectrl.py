import socket
from time import time, sleep
from threading import Thread

# Define drone
class dm107s():
    # Default control value
    def __init__(self):
        # 4 values for flight
        self.roll = 128
        self.pitch = 128
        self.throttle = 128
        self.yaw = 128
        # 0 - normal mode, 2 - emergency stop, 4 - gyroscope calibration
        self.commands = 0
        # Required for wifi control
        self.onoff = 1
        # Prevent multiple takeoff button presses
        self._takeoff_flag = False
        # Prevent multiple calibrate button presses
        self._calibrate_flag = False
        # Connect to UDP port
        self.sess = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        #self.sess.connect(('192.168.100.1', 19798))
        # Initialize timer value
        self._takeoff_timer = 0
        self._calibrate_timer = 0
        # Flag to stop thread
        self._stopped = False
    
    # Start separated thread for drone control
    def start(self):    
        self._thread = Thread(target=self.send_ctrl, args=(), daemon=True)
        self._thread.start()
        return self
    
    # Get command hex for drone
    def get_hex(self):
        # XOR is for checksum
        self.command_out=((26122<<144)|self.roll<<136|self.pitch<<128|self.throttle<<120|self.yaw<<112|self.commands<<104|self.onoff*2<<96|65535<<80|(self.roll^self.pitch^self.throttle^self.yaw^self.commands^(self.onoff*2))<<8|153)
        self.command_out = hex(self.command_out)[2::]
        return self.command_out
    
    # Turn hex to byte package
    def _get_packet(self):
        self._hex_code = self.get_hex()
        self.package = bytes.fromhex(self._hex_code)
        return self.package
    
    # Send control to drone
    def send_ctrl(self):
        while not self._stopped:
            self._package = self._get_packet()
            #self.sess.send(self._package)
            self.sess.sendto(self._package, ('192.168.100.1', 19798))
            self.Flag_off()
            sleep(0.02)
    
    # Close connection to drone
    def close_connection(self):
        self._stopped = True
        if self._thread.daemon == False:
            self._thread.join()
            self.sess.close()
    
    # Return to default
    def default(self):
        self.roll = 128
        self.pitch = 128
        self.throttle = 128
        self.yaw = 128
        self.commands = 0
        self.onoff = 1
        self._takeoff_flag = False
    
    # Increment control
    def incremt(self, rl, pt, th, yw):
        self._value_to_change = [128, 128, 128, 128]
        self._change_val = [rl, pt, th, yw]
        for x in range(len(self._value_to_change)):
            self._value_to_change[x] += self._change_val[x]
            if self._value_to_change[x] <= 0:
                self._value_to_change[x] = 0
            if self._value_to_change[x] >= 255:
                self._value_to_change[x] = 255
            [self.roll, self.pitch, self.throttle, self.yaw] = self._value_to_change
    
    # Roll right
    def roll_right(self):
        if self.roll < 255:
            self.roll += 127
    
    # Pitch forward
    def pitch_fwd(self):
        if self.pitch < 255:
            self.pitch += 127
    
    # Increase throttle
    def throttle_up(self):
        if self.throttle < 255:
            self.throttle += 127
    
    # Yaw right
    def yaw_right(self):
        if self.yaw < 255:
            self.yaw += 127
    
    # Roll left
    def roll_left(self):
        if self.roll > 1:
            self.roll -= 127
    
    # Pitch backward
    def pitch_bwd(self):
        if self.pitch > 1:
            self.pitch -= 127
    
    # Decrease throttle
    def throttle_dwn(self):
        if self.throttle > 1:
            self.throttle -= 127
    
    # Yaw left
    def yaw_left(self):
        if self.yaw > 1:
            self.yaw -= 127
    
    # Takeoff
    def takeoff(self):
        if self._takeoff_flag == False:
            self.commands = 1
            self._takeoff_flag = True
            self._takeoff_timer = time()
    
    # Landing
    def land(self):
        if self._takeoff_flag == False:
            self.commands = 1
            self._takeoff_flag = True
            self._takeoff_timer = time()
    
    # Flip takeoff flag
    def Flag_off(self):
        if (self._takeoff_flag == True and (time() - self._takeoff_timer >= 1)):
            self.commands = 0
            self._takeoff_flag = False
        if (self._calibrate_flag == True and (time() - self._calibrate_timer >= 3)):
            self.commands = 0
            self.onoff = 1
            self._calibrate_flag = False

    # Stop IMMEDIATELY
    def emergency_stop(self):
        self.roll = 128
        self.pitch = 128
        self.throttle = 128
        self.yaw = 128
        self.commands = 2
        self.onoff = 1
        self._takeoff_flag = False
    
    # Calibrate gyroscope
    def calib_gyro(self):
        if self._calibrate_flag == False:
            self.roll = 128
            self.pitch = 128
            self.throttle = 128
            self.yaw = 128
            self.commands = 4
            self.onoff = 0
            self._calibrate_flag = True
            self._calibrate_timer = time()

class naza():
    # Default control value
    def __init__(self, ip, port):
        # 4 values for flight
        self.roll = 8
        self.pitch = 8
        self.throttle = 8
        self.yaw = 8
        # Prevent multiple ignite button presses
        self._ignite_flag = False
        # Connect to UDP port
        self.sess = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        #self.sess.connect((ip, port))
        # Initialize timer value
        self._ignite_timer = 0
        # Flag to stop thread
        self._stopped = False
    
    # Start separated thread for drone control
    def start(self):    
        self._thread = Thread(target=self.send_ctrl, args=(), daemon=True)
        self._thread.start()
        return self
    
    # Get command hex for drone
    def get_hex(self):
        # XOR is for checksum
        self.command_out=(self.throttle<<12|self.yaw<<8|self.pitch<<4|self.roll)
        self.command_out = hex(self.command_out)[2::]
        return self.command_out
    
    # Send control to drone
    def send_ctrl(self):
        while not self._stopped:
            self._package = self.get_hex()
            #self.sess.send(self._package)
            self.sess.sendto(self._package, (ip, port))
            self.Flag_off()
            sleep(0.02)
    
    # Close connection to drone
    def close_connection(self):
        self._stopped = True
        if self._thread.daemon == False:
            self._thread.join()
            self.sess.close()
    
    # Return to default
    def default(self):
        self.roll = 8
        self.pitch = 8
        self.throttle = 8
        self.yaw = 8
        self._takeoff_flag = False
    
    # Increment control
    def incremt(self, rl, pt, th, yw):
        self._value_to_change = [8, 8, 8, 8]
        self._change_val = [rl, pt, th, yw]
        for x in range(len(self._value_to_change)):
            self._value_to_change[x] += self._change_val[x]
            if self._value_to_change[x] <= 0:
                self._value_to_change[x] = 0
            if self._value_to_change[x] >= 15:
                self._value_to_change[x] = 15
            [self.roll, self.pitch, self.throttle, self.yaw] = self._value_to_change
    
    # Roll right
    def roll_right(self):
        if self.roll < 14:
            self.roll += 1
    
    # Pitch forward
    def pitch_fwd(self):
        if self.pitch < 15:
            self.pitch += 1
    
    # Increase throttle
    def throttle_up(self):
        if self.throttle < 15:
            self.throttle += 1
    
    # Yaw right
    def yaw_right(self):
        if self.yaw < 15:
            self.yaw += 1
    
    # Roll left
    def roll_left(self):
        if self.roll > 0:
            self.roll -= 1
    
    # Pitch backward
    def pitch_bwd(self):
        if self.pitch > 0:
            self.pitch -= 1
    
    # Decrease throttle
    def throttle_dwn(self):
        if self.throttle > 0:
            self.throttle -= 1
    
    # Yaw left
    def yaw_left(self):
        if self.yaw > 0:
            self.yaw -= 1
    
    # Start engine
    def ignite(self):
        self.roll = 0
        self.pitch = 0
        self.throttle = 0
        self.yaw = 15
        self._takeoff_flag = True
    
    # Flip takeoff flag
    def Flag_off(self):
        if (self._takeoff_flag == True and (time() - self._takeoff_timer >= 1)):
            self.default()


    
