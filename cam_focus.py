import cv2
import os
import Jetson.GPIO as gp
import threading
from queue import Queue
from Focuser import Focuser
import time

class AutoFocuser:
    def __init__(self, focuser):
        self.focuser = focuser
        self.max_index = 10
        self.max_value = 0.0
        self.last_value = 0.0
        self.dec_count = 0
        self.focal_distance = 10
        self.focus_finished = False
        self.skip_frame = 6

    def laplacian(self, img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        img_sobel = cv2.Laplacian(img_gray, cv2.CV_16U)
        return cv2.mean(img_sobel)[0]

    def adjust_focal_length(self, frame):
        if self.skip_frame == 0:
            self.skip_frame = 6
            if self.dec_count < 6 and self.focal_distance < 1000:
                self.focuser.set(Focuser.OPT_FOCUS, self.focal_distance)
                val = self.laplacian(frame)
                if val > self.max_value:
                    self.max_index = self.focal_distance
                    self.max_value = val

                if val < self.last_value:
                    self.dec_count += 1
                else:
                    self.dec_count = 0

                if self.dec_count < 6:
                    self.last_value = val
                    self.focal_distance += 10
            elif not self.focus_finished:
                self.focuser.set(Focuser.OPT_FOCUS, self.max_index)
                self.focus_finished = True
        else:
            self.skip_frame = self.skip_frame - 1


class ArducamController:
    def __init__(self, exit_flag):
        self.exit_flag = exit_flag
        gp.setwarnings(False)
        gp.setmode(gp.BOARD)
        gp.setup(7, gp.OUT)
        gp.setup(11, gp.OUT)
        gp.setup(12, gp.OUT)

        self.focuser1 = Focuser(7)
        self.focuser2 = Focuser(7)
        self.focuser3 = Focuser(7)
        self.focuser4 = Focuser(7)

        self.auto_focuser1 = AutoFocuser(self.focuser1)
        self.auto_focuser2 = AutoFocuser(self.focuser2)
        self.auto_focuser3 = AutoFocuser(self.focuser3)
        self.auto_focuser4 = AutoFocuser(self.focuser4)

    def set_camera(self, cam):
        if cam == 1:
            i2c = "i2cset -y 7 0x70 0x00 0x04"
            gp.output(7, False)
            gp.output(11, False)
            gp.output(12, True)
        elif cam == 2:
            i2c = "i2cset -y 7 0x70 0x00 0x05"
            gp.output(7, True)
            gp.output(11, False)
            gp.output(12, True)
        elif cam == 3:
            i2c = "i2cset -y 7 0x70 0x00 0x06"
            gp.output(7, False)
            gp.output(11, True)
            gp.output(12, False)
        elif cam == 4:
            i2c = "i2cset -y 7 0x70 0x00 0x07"
            gp.output(7, True)
            gp.output(11, True)
            gp.output(12, False)
        else:
            print("Invalid camera number")
            return

        #os.system(i2c)
        print(f'success set camera {cam}')

    def set_focal_length(self, cam, value):
        self.set_camera(cam)
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
            f_length = 0
            while self.exit_flag.check():

                frame = get_frame(camera, 100)

                frame1 = frame[:360, :640, :]
                frame2 = frame[360:, :640, :]
                frame3 = frame[:360, 640:, :]
                frame4 = frame[360:, 640:, :]

                self.set_camera(1)
                self.focuser1.set(Focuser.OPT_FOCUS, f_length)
                self.set_camera(2)
                self.focuser2.set(Focuser.OPT_FOCUS, f_length)
                self.set_camera(3)
                self.focuser3.set(Focuser.OPT_FOCUS, f_length)
                self.set_camera(4)
                self.focuser4.set(Focuser.OPT_FOCUS, f_length)

                f_length += 10
                if f_length == 160:
                    f_length = 0   
                time.sleep(0.1)                             

                # self.auto_focuser1.adjust_focal_length(frame1)
                # self.auto_focuser2.adjust_focal_length(frame2)
                # self.auto_focuser3.adjust_focal_length(frame3)
                # self.auto_focuser4.adjust_focal_length(frame4)

                cv2.imshow('1', frame1)
                cv2.imshow('2', frame2)
                cv2.imshow('3', frame3)
                cv2.imshow('4', frame4)

                keyCode = cv2.waitKey(16) & 0xFF

            cv2.destroyWindow(window_name)

        cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0, display_height=720, display_width=1280), cv2.CAP_GSTREAMER)
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
