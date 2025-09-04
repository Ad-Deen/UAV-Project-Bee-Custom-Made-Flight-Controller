#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import struct
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from pynput import keyboard
import threading

def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))

class BeeControlNode(Node):
    def __init__(self):
        super().__init__('bee_control_node')
        
        # self.esp_ip = '192.168.0.102'       #Explore
        self.esp_ip = '192.168.4.3'       #Hive
        self.esp_port = 10002
        self.pc_port = 10001

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.pc_port))
        self.sock.setblocking(False)

        self.publisher = self.create_publisher(Float32MultiArray, 'quadcopter_feedback', 10)
        self.create_subscription(Int32MultiArray, '/motion', self.motion_callback, 10)
        self.motion_fb = 1
        self.motion_lr = 1
        self.motion_ud = 1
 

        # Rotor and PID values
        self.r = 0     # 430 Flight point | 430+ elevation | 430- Depletion | 400- Drastic fall
        # self.r = 10  #teleop
        self.kpx = 200  #PID
        # self.kdx = 0  #PID
        # self.kpx = 10     #teleop-----------------------------------------------
        # self.kdx = 10     #teleop-----------------------------------------------
        self.kdx = 100       # brake tuning----------------------------------------
        self.kix = 100        # brake tuning----------------------------------------
        # self.kix = 0        #PID
        self.kpy = 200      #PID
        self.kdy = 100      #PID
        self.kiy = 120      #PID
        self.kpz = 100      #PID
        self.kdz = 50       #PID
        self.kiz = 0        #PID
        self.rate = 10      #PID

        self.timer = self.create_timer(0.05, self.loop)  # 20Hz

        # Start keyboard listener thread
        threading.Thread(target=self.start_keyboard_listener, daemon=True).start()

    def motion_callback(self, msg):
        try:
            fb, lr, ud = msg.data
            self.motion_fb = fb + 1
            self.motion_lr = lr + 1
            self.motion_ud = ud + 1
            # print(f"Command: Rotors set to {self.motion_fb} ,{self.motion_lr} , {self.motion_ud} ")
        except Exception as e:
            self.get_logger().error(f"Motion callback error: {e}")

    def start_keyboard_listener(self):
        def on_press(key):
            try:
                #-------------------------Teleop commands---------------------------------------------------------------
                if key.char == '8':
                    self.kpx = 20
                    # self.kpx = constrain(self.kpx + self.rate, 0, 20)
                    print(f"Command: Rotors set to {self.r}")
                elif key.char == '2':
                    self.kpx = 0
                    # self.kpx = constrain(self.kpx - self.rate, 0, 20)
                    print(f"Command: Rotors set to {self.r}")
                elif key.char == '4':
                    self.kdx = 0
                    # self.kdx = constrain(self.kdx - self.rate, 0, 20)
                    print(f"Command: Rotors set to {self.r}")
                elif key.char == '6':
                    # self.kdx = constrain(self.kdx + self.rate, 0, 20)
                    self.kdx = 20
                    print(f"Command: Rotors set to {self.r}")
                elif key.char == '0':
                    # self.kdx = constrain(self.kdx + self.rate, 0, 20)
                    self.kdx = 10
                    self.kpx = 10
                    print(f"Command: Rotors set to {self.r}")
                # if len(self.pressed_keys) == 0:
                #     self.kdx = 10
                #     self.kdx = 10
                # elif key.char == 'c':
                #     self.r = constrain(self.r + self.rate, 0, 500)
                #     print(f"Command: Rotors set to {self.r}")
                # elif key.char == 'q':
                #     self.kdx = constrain(self.kix - self.rate, 0, 200)
                # elif key.char == 'e':
                #     self.kdx = constrain(self.kix + self.rate, 0, 200)
                # elif key.char == 'c':
                #     self.kdx = constrain(self.kdx - self.rate, 0, 200)
                #--------------------------------PID tunning parameters--------------------------------------------
                elif key.char == 'w':
                    self.r = constrain(self.r + self.rate, 0, 600)
                elif key.char == 's':
                    self.r = constrain(self.r - self.rate, 0, 600)
                    print(f"Command: Rotors set to {self.r}")
                elif key.char == 'J':
                    self.kpx = constrain(self.kpx + self.rate, 0, 10000)
                    print("+ Change kpx")
                elif key.char == 'j':
                    self.kpx = constrain(self.kpx - self.rate, 0, 10000)
                    print("- Change kpx")
                elif key.char == 'K':
                    self.kdx = constrain(self.kdx + self.rate, 0, 10000)
                    print("+ Change kdx")
                elif key.char == 'k':
                    self.kdx = constrain(self.kdx - self.rate, 0, 10000)
                    print("- Change kdx")
                elif key.char == 'L':
                    self.kix = constrain(self.kix + self.rate, 0, 10000)
                    print("+ Change kix")
                elif key.char == 'l':
                    self.kix = constrain(self.kix - self.rate, 0, 10000)
                    print("- Change kix")
                elif key.char == 'F':
                    self.kpy = constrain(self.kpy + self.rate, 0, 10000)
                    print("+ Change kpy")
                elif key.char == 'f':
                    self.kpy = constrain(self.kpy - self.rate, 0, 10000)
                    print("- Change kpy")
                elif key.char == 'G':
                    self.kdy = constrain(self.kdy + self.rate, 0, 10000)
                    print("+ Change kdy")
                elif key.char == 'g':
                    self.kdy = constrain(self.kdy - self.rate, 0, 10000)
                    print("- Change kdy")
                elif key.char == 'H':
                    self.kiy = constrain(self.kiy + self.rate, 0, 10000)
                    print("+ Change kiy")
                elif key.char == 'h':
                    self.kiy = constrain(self.kiy - self.rate, 0, 10000)
                    print("- Change kiy")
                # elif key.char == 'U':
                #     self.kpz = constrain(self.kpz + self.rate, 0, 10000)
                #     print("+ Change kpz")
                # elif key.char == 'u':
                #     self.kpz = constrain(self.kpz - self.rate, 0, 10000)
                #     print("- Change kpz")
                # elif key.char == 'I':
                #     self.kdz = constrain(self.kdz + self.rate, 0, 10000)
                #     print("+ Change kdz")
                # elif key.char == 'i':
                #     self.kdz = constrain(self.kdz - self.rate, 0, 10000)
                #     print("- Change kdz")
                # elif key.char == 'O':
                #     self.kiz = constrain(self.kiz + self.rate, 0, 10000)
                #     print("+ Change kiz")
                # elif key.char == 'o':
                #     self.kiz = constrain(self.kiz - self.rate, 0, 10000)
                #     print("- Change kiz")
                elif key.char == 'n':
                    self.rate = 10
                    print("Set rate = 10")
                elif key.char == 'm':
                    self.rate = 1
                    print("Set rate = 1")
                else:
                    # self.kpx = 10  # emergency stop fallback
                    # self.kdx = 10
                    self.r = 0
            except AttributeError:
                pass

        listener = keyboard.Listener(on_press=on_press)
        listener.start()

    def loop(self):
        rotor_cmd = struct.pack(
            '<13H',
            self.r,
            self.kpx,
            self.kdx,
            self.kix,
            self.kpy,
            self.kdy,
            self.kiy,
            self.kpz,
            self.kdz,
            self.kiz,
            self.motion_fb,
            self.motion_lr,
            self.motion_ud
        )

        try:
            self.sock.sendto(rotor_cmd, (self.esp_ip, self.esp_port))
        except Exception as e:
            self.get_logger().warn(f"UDP send error: {e}")
            return

        try:
            data, _ = self.sock.recvfrom(64)
            if len(data) == 20:
                r1, r2, r3, r4, gx, gy, gz = struct.unpack('<4H3f', data)
                msg = Float32MultiArray()
                msg.data = [float(r1), float(r2), float(r3), float(r4), gx, gy, gz]
                self.publisher.publish(msg)

                print("throttle | r1 | r2 | r3 | r4 | gyroX | gyroY | gyroZ | rate | kpX | kdX | kiX | kpY | kdY | kiY | kpZ | kdZ | kiZ")
                print(f"{self.r} | {r1} | {r2} | {r3} | {r4} | {gx:.2f} | {gy:.2f} | {gz:.2f} | {self.rate} | {self.kpx} | {self.kdx} | {self.kix} | {self.kpy} | {self.kdy} | {self.kiy} | {self.kpz} | {self.kdz} | {self.kiz}")

        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = BeeControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
