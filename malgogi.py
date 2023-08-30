import os
import cv2
import time
import Jetson.GPIO as gp
import threading
from queue import Queue
from Focuser import Focuser
class ArducamController:
    def __init__(self, exit_flag):
        self.exit_flag = exit_flag
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)
        gp.setup(7, gp.OUT)
        gp.setup(11, gp.OUT)
        gp.setup(12, gp.OUT)

        self.focuser1 = Focuser(1)
        self.focuser2 = Focuser(1)
        self.focuser3 = Focuser(1)
        self.focuser4 = Focuser(1)

    def set_focal_length(self, cam, value):
        i2c = "i2cset -y 1 0x70 0x00 0x0{}".format(cam + 3)
        os.system(i2c)
        if cam == 1:
            self.focuser1.set(Focuser.OPT_FOCUS, value)
        elif cam == 2:
            self.focuser2.set(Focuser.OPT_FOCUS, value)
        elif cam == 3:
            self.focuser3.set(Focuser.OPT_FOCUS, value)
        elif cam == 4:
            self.focuser4.set(Focuser.OPT_FOCUS, value)
        else:
            print("Invalid camera number")
    def laplacian(self, img):
        img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        img_sobel = cv2.Laplacian(img_gray,cv2.CV_16U)
        return cv2.mean(img_sobel)[0]
    def stream_video(self, cam):
        print(f"Start streaming video from camera {cam}")
        width = 1280
        height = 720

        def gstreamer_pipeline(capture_width=1280, capture_height=720, display_width=640, display_height=360, framerate=60, flip_method=0):
            return (
                "nvarguscamerasrc ! "
                "video/x-raw(memory:NVMM), "
                "width=(int)%d, height=(int)%d, "
                "format=(string)NV12, framerate=(fraction)%d/1 ! "
                "nvvidconv flip-method=%d ! "
                "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
                % (
                    capture_width,
                    capture_height,
                    framerate,
                    flip_method,
                    display_width,
                    display_height,
                )
            )

        def frame_reader(camera, name):
            while self.exit_flag.check():
                _, frame = camera.read()
                while queues:
                    queue = queues.pop()
                    queue.put(frame)

        def get_frame(camera, timeout=None):
            queue = Queue(1)
            queues.append(queue)
            return queue.get(timeout=timeout)

        def previewer(camera, name):
            window_name = "Arducam"
            while self.exit_flag.check():

                frame = get_frame(camera, 2000)
                frame0 = frame[:360, :640, :]
                frame1 = frame[360:, :640, :]
                frame2 = frame[:360, 640:, :]
                frame3 = frame[360:, 640:, :]

                self.auto_focuser1.adjust_focal_length(frame0)
                self.auto_focuser2.adjust_focal_length(frame1)
                self.auto_focuser3.adjust_focal_length(frame2)
                self.auto_focuser4.adjust_focal_length(frame3)

                cv2.imshow('0',frame0)
                cv2.imshow('1',frame1)
                cv2.imshow('2',frame2)
                cv2.imshow('3',frame3)                
                #cv2.imshow(window_name, frame)
                keyCode = cv2.waitKey(16) & 0xFF
            cv2.destroyWindow(window_name)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0, display_height=720, display_width = 1280), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            raise RuntimeError("Failed to open camera!")

        queues = []

        frame_reader_thread = threading.Thread(target=frame_reader, args=(cap, ""))
        frame_reader_thread.daemon = True
        frame_reader_thread.start()

        previewer_thread = threading.Thread(target=previewer, args=(cap, ""))
        previewer_thread.daemon = True
        previewer_thread.start()

        while self.exit_flag.check():
            time.sleep(10)

        cap.release()
        cv2.destroyAllWindows()

        gp.cleanup()

if __name__ == "__main__":

    class ExitFlag:
        def __init__(self):
            self.flag = True

        def on(self):
            self.flag = False
        def check(self):
            return self.flag

    ef = ExitFlag()
    import signal, sys

    def signal_handler(sig, frame):
        print('You pressed Ctrl+C!')
        ef.on()
        print('System shutdown in 3 secs')
        time.sleep(3)    
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    controller = ArducamController(ef)
    controller.stream_video(1)
