# Heavily based off of this:
# https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
import threading
import rospy
from bmb_msgs.msg import StateCommand
import sys, select, termios, tty
from math import pi


class StateCommandPublishThread(threading.Thread):
    def __init__(self, rate=0.0):
        super(StateCommandPublishThread, self).__init__()
        self.publisher = rospy.Publisher('/state_command', StateCommand, queue_size=1)
        self.roll = 0.0
        self.pitch = 0.0
        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print(f"Waiting for subscriber to connect to {self.publisher.name}")
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, pitch, roll, speed):
        self.condition.acquire()
        self.pitch = pitch
        self.roll = roll
        self.speed = speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 10)
        self.join()

    def run(self):
        command = StateCommand()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into command message.
            command.pitch = self.pitch * pi / 180
            command.roll = self.roll * pi / 180
            command.speed = self.speed

            self.condition.release()

            # Publish.
            self.publisher.publish(command)

        # Publish stop message when thread exits.
        command.pitch = 0
        command.roll = 0
        command.speed = 10
        self.publisher.publish(command)


class KeyboardStateCommander:
    MOVE_BINDINGS = {
        'w': (1, 0),
        's': (-1, 0),
        'd': (0, 1),
        'a': (0, -1)
    }

    SPEED_BINDINGS = {
        'o': 1.1,
        'p': 0.9
    }

    MSG = """
    Reading from the keyboard  and Publishing to /state_command!
    ---------------------------
    w/s : increase/decrease pitch by 1 degree
    d/a : increase/decrease roll by 1 degree
    o/p : increase/decrease speed by 10%
    CTRL-C to quit
    """

    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.pitch = 0
        self.roll = 0
        self.speed = 10
        self.status = 0
        self.key_timeout = None
        self.pub_thread = StateCommandPublishThread()

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, *_ = select.select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def print_command(self):
        # print self.MSG after every 15 lines
        if self.status == 0:
            print(self.MSG)
        self.status = (self.status + 1) % 15

        print(f"Currently:\tpitch {self.pitch}\troll {self.roll}\tspeed {self.speed} ")

    def spin(self):
        try:
            self.pub_thread.wait_for_subscribers()
            self.pub_thread.update(self.pitch, self.roll, self.speed)

            self.print_command()
            while True:
                key = self.get_key()
                if key == '\x03': # CTRL-C
                    break
                elif key in self.MOVE_BINDINGS.keys():
                    self.pitch += self.MOVE_BINDINGS[key][0]
                    self.roll += self.MOVE_BINDINGS[key][1]
                    self.print_command()
                    self.pub_thread.update(self.pitch, self.roll, self.speed)
                elif key in self.SPEED_BINDINGS.keys():
                    self.speed *= self.SPEED_BINDINGS[key]
                    self.print_command()
                    self.pub_thread.update(self.pitch, self.roll, self.speed)

        except Exception as e:
            print(e)

        finally:
            self.pub_thread.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # Program won't terminate if other nodes were also launched via roslaunch
            print("Closing node. Press Ctrl+C again if the program doesn't terminate.")
