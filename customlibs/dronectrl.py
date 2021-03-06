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
        self.roll += 20
        if self.roll > 248:
            self.roll = 248
    
    # Pitch forward
    def pitch_fwd(self):
        self.pitch += 20
        if self.pitch > 248:
            self.pitch = 248
    
    # Increase throttle
    def throttle_up(self):
        self.throttle += 20
        if self.throttle > 248:
            self.throttle = 248
    
    # Yaw right
    def yaw_right(self):
        self.yaw -= 20
        if self.yaw < 18:
            self.yaw = 18
    
    # Roll left
    def roll_left(self):
        self.roll -= 20
        if self.roll < 18:
            self.roll = 18
    
    # Pitch backward
    def pitch_bwd(self):
        self.pitch -= 20
        if self.pitch < 18:
            self.pitch = 18
    
    # Decrease throttle
    def throttle_dwn(self):
        self.throttle -= 20
        if self.throttle < 18:
            self.throttle = 18
    
    # Yaw left
    def yaw_left(self):
        self.yaw += 20
        if self.yaw > 248:
            self.yaw = 248
    
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
        # Prevent multiple takeoff button presses
        self._takeoff_flag = False
        # Prevent multiple ignite button presses
        self._ignite_flag = False
        self._ignite_send = False
        # Connect to UDP port
        self.sess = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.ip = ip
        self.port = port
        #self.sess.connect((ip, port))
        # Initialize timer value
        self._ignite_timer = 0
        self._takeoff_timer = 0
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
            if self._ignite_send == True:
                ignite_msg = 'st'
                self._package = ignite_msg.encode()
            else:
                self._package = self.get_hex().encode()
            #self.sess.send(self._package)
            self.sess.sendto(self._package, (self.ip, self.port))
            self.Flag_off()
            sleep(0.05)
    
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
        self._ignite_flag = False
    
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
        if self.roll < 15:
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
        if self._ignite_flag == False:
            self._ignite_flag = True
            self._ignite_send = True
            self._ignite_timer = time()
        
    # Takeoff
    def takeoff(self):
        if self._takeoff_flag == False:
            self.throttle = 12
            self._takeoff_flag = True
            self._takeoff_timer = time()
    
    # Flip takeoff flag
    def Flag_off(self):
        if self._ignite_flag == True:
            if (time() - self._ignite_timer >= 1) and (time() - self._ignite_timer < 1.5):
                self._ignite_send = False
                self.roll = 8
                self.pitch = 8
                self.yaw = 8
                self.throttle = 0
            # Warming up engine
            if (time() - self._ignite_timer >= 1.5) and (time() - self._ignite_timer < 2):
                self.throttle = 2
            if (time() - self._ignite_timer >= 2) and (time() - self._ignite_timer < 2.5):
                self.throttle = 4
            if (time() - self._ignite_timer >= 2.5) and (time() - self._ignite_timer < 3):
                self.throttle = 6
            if (time() - self._ignite_timer >= 3) and (time() - self._ignite_timer < 4):
                self.throttle = 8
            # After starting engine, takeoff after 4s
            if (time() - self._ignite_timer >= 4):
                self._ignite_flag = False
                self.takeoff()
        if (self._takeoff_flag == True and (time() - self._takeoff_timer >= 4)):
            self.throttle = 8
            self._takeoff_flag = False
