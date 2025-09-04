#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class GyroPlotNode(Node):
    def __init__(self):
        super().__init__('gyro_plot_node')

        self.max_len = 50
        self.gyroX_data = np.zeros(self.max_len)
        self.gyroY_data = np.zeros(self.max_len)
        self.gyroZ_data = np.zeros(self.max_len)  # <--- Add gyroZ buffer

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/quadcopter_feedback',
            self.listener_callback,
            10
        )

        # Plotting setup
        self.fig, self.ax = plt.subplots()
        self.line_gyroX, = self.ax.plot(self.gyroX_data, label='gyroX')
        self.line_gyroY, = self.ax.plot(self.gyroY_data, label='gyroY')
        self.line_gyroZ, = self.ax.plot(self.gyroZ_data, label='gyroZ')  # <--- Add gyroZ line

        self.ax.set_ylim(-10, 10)
        self.ax.set_xlim(0, self.max_len)
        self.ax.set_title("Real-time Gyroscope Data")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("Degrees/sec")
        self.ax.legend()

        self.ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50)
        plt.tight_layout()
        plt.show(block=False)

    def listener_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 7:
            new_gyroX = msg.data[4]
            new_gyroY = msg.data[5]
            new_gyroZ = msg.data[6]

            self.gyroX_data = np.roll(self.gyroX_data, -1)
            self.gyroY_data = np.roll(self.gyroY_data, -1)
            self.gyroZ_data = np.roll(self.gyroZ_data, -1)

            self.gyroX_data[-1] = new_gyroX
            self.gyroY_data[-1] = new_gyroY
            self.gyroZ_data[-1] = new_gyroZ

    def update_plot(self, frame):
        self.line_gyroX.set_ydata(self.gyroX_data)
        self.line_gyroY.set_ydata(self.gyroY_data)
        self.line_gyroZ.set_ydata(self.gyroZ_data)
        self.fig.canvas.draw()
        return self.line_gyroX, self.line_gyroY, self.line_gyroZ

def main(args=None):
    rclpy.init(args=args)
    node = GyroPlotNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
