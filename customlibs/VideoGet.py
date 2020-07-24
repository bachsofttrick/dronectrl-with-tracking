from threading import Thread
import cv2
import os
from time import sleep

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src):
        self.src = src
        '''
        if (src[0:20] == "rtsp://192.168.100.1"):
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
            self._stream = cv2.VideoCapture(src,cv2.CAP_FFMPEG)
        '''
        if src == '0':
            self._stream = cv2.VideoCapture(0)
        elif (src[0:4] == "rtsp"):
            src = "rtspsrc location=" + src + " latency=0 buffer-mode=auto ! decodebin ! videoconvert ! appsink sync=false"
            self._stream = cv2.VideoCapture(src)
        elif (src[0:3] == "udp"):
            src = "udpsrc port=" + src[4:] + " ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! appsink sync=false"
            self._stream = cv2.VideoCapture(src)
        elif (src[0:3] == "tcp"):
            src = "tcpclientsrc " + src + " ! gdpdepay ! rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false" #host=192.168.4.101 port=8081
            self._stream = cv2.VideoCapture(src)
        (self.grabbed, self.frame) = self._stream.read()
        self._stopped = False

    def start(self):
        '''
        Daemon is to make background thread, which shutdown with main thread
        regardless of conditions
        '''
        self._thread = Thread(target=self._get, args=(), daemon=True)
        self._thread.start()
        return self

    def _get(self):
        while not self._stopped:
            '''
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self._stream.read()
                if (self.src[0:20] == "rtsp://192.168.100.1"):
                    sleep(0.005)
            '''
            try:
                (self.grabbed, self.frame) = self._stream.read()
                (self.temp_grabbed, self.temp_frame) = (self.grabbed, self.frame)
            except:
                continue
                (self.grabbed, self.frame) = (self.temp_grabbed, self.temp_frame)
            if (self.src[0:20] == "rtsp://192.168.100.1"):
                sleep(0.005)
    
    def update(self):
        return (self.grabbed, self.frame)

    def print_addr(self):
        print(self.src[0:20])
    
    def stop(self):
        self._stopped = True
        if self._thread.daemon == False:
            print("Shutting down thread.")
            self._thread.join() # Wait till end of thread
            self._stream.release() # [only release when thread has stopped running]
