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
        if (src[0:20] == "rtsp://192.168.100.1"):
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;udp"
            self._stream = cv2.VideoCapture(src,cv2.CAP_FFMPEG)
        elif src == '0':
            self._stream = cv2.VideoCapture(0)
        else:
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
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self._stream.read()
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
