#!/usr/bin/env python3

# Copyright (c) 2024 Mateus Menezes

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class FutureReferencesPublisher(Node):
    def __init__(self):
        super().__init__('future_refs_publisher')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/unconstrained_mpc_controller_for_light_configuration/future_refs', 10)

    def read_csv_file(self, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            future_refs = []
            for line in lines:
                future_refs.append([float(value) for value in line.split(',')])
        return future_refs

    def publish_future_refs(self, future_refs):
        msg = Float64MultiArray()
        for ref in future_refs:
            for value in ref:
                msg.data.append(value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    future_refs_publisher = FutureReferencesPublisher()
    file = '/home/bluerov2/ros-ws/src/unconstrained_mpc_controller/scripts/trajectory_example.csv'
    future_refs = future_refs_publisher.read_csv_file(file)
    future_refs_publisher.publish_future_refs(future_refs)
    future_refs_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
