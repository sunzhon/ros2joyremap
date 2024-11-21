import os,sys
import rclpy
from rclpy.node import Node
from ambot_msgs.msg import UserCommand 
import ast
import operator as op
import traceback
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Pose
import time
from collections import OrderedDict
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RestrictedEvaluator(object):
    def __init__(self):
        self.operators = {
            ast.Add: op.add,
            ast.Sub: op.sub,
            ast.Mult: op.mul,
            ast.Div: op.truediv,
            ast.BitXor: op.xor,
            ast.USub: op.neg,
        }
        self.functions = {
            'abs': lambda x: abs(x),
            'max': lambda *x: max(*x),
            'min': lambda *x: min(*x),
        }

    def _reval_impl(self, node, variables):
        if isinstance(node, ast.Num):
            return node.n
        elif isinstance(node, ast.BinOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.left, variables),
                      self._reval_impl(node.right, variables))
        elif isinstance(node, ast.UnaryOp):
            op = self.operators[type(node.op)]
            return op(self._reval_impl(node.operand, variables))
        elif isinstance(node, ast.Call) and node.func.id in self.functions:
            func = self.functions[node.func.id]
            args = [self._reval_impl(n, variables) for n in node.args]
            return func(*args)
        elif isinstance(node, ast.Name) and node.id in variables:
            return variables[node.id]
        elif isinstance(node, ast.Subscript) and node.value.id in variables:
            var = variables[node.value.id]
            idx = node.slice.value.n
            try:
                return var[idx]
            except IndexError:
                raise IndexError("Variable '%s' out of range: %d >= %d" % (node.value.id, idx, len(var)))
        else:
            raise TypeError("Unsupported operation: %s" % node)

    def reval(self, expr, variables):
        expr = str(expr)
        if len(expr) > 1000:
            raise ValueError("The length of an expression must not be more than 1000 characters")
        try:
            return self._reval_impl(ast.parse(expr, mode='eval').body, variables)
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            raise e


class JoyRemap(Node):
    def __init__(self, namespace=None):
        super().__init__("joy_remap")
        self.evaluator = RestrictedEvaluator()

        # ROS2 publish
        qos_profile = QoSProfile(
            reliability = ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=2
        )

        self.pub_joycmd = self.create_publisher(
            UserCommand,
            namespace+"/joy_cmd",
            qos_profile
        )

        # ROS2 subscribers
        self.sub_joy = self.create_subscription(
            Joy,
            namespace+"/joy",
            self.callback,
            1
        )
        self.get_logger().info("joy remap node run")

        self.declare_parameter('axes', "")
        self.declare_parameter('buttons', "")
        self.axes = self.get_parameter('axes').get_parameter_value().string_value
        self.buttons = self.get_parameter('buttons').get_parameter_value().string_value

        self.get_logger().warn(f"axes: {self.axes}")

        if self.axes=="":
            self.axes = ["axes[1]","axes[0]","axes[2]", "axes[3]","axes[4]"]
        if self.buttons=="":
            self.buttons = [
                    "buttons[0]","buttons[1]","buttons[2]","buttons[3]","buttons[4]",
                    "buttons[5]","buttons[6]","buttons[7]","buttons[8]","buttons[9]",
                    ]

        self.speed_gain = 1.0

    def callback(self, in_msg):
        out_msg = Joy(header=in_msg.header)
        map_axes = self.axes
        map_btns = self.buttons
        out_msg.axes = [0.0] * len(map_axes)
        out_msg.buttons = [0] * len(map_btns)
        in_dic = {"axes": in_msg.axes, "buttons": in_msg.buttons}
        for i, exp in enumerate(map_axes):
            try:
                out_msg.axes[i] = self.evaluator.reval(exp, in_dic)
            except NameError as e:
                self.get_logger().error(f"You are using vars other than 'buttons' or 'axes': {e}")
            except UnboundLocalError as e:
                self.get_logger().error(f"Wrong form: {e}")
            except Exception as e:
                raise e

        for i, exp in enumerate(map_btns):
            try:
                if self.evaluator.reval(exp, in_dic) > 0:
                    out_msg.buttons[i] = 1
            except NameError as e:
                self.get_logger().error(f"You are using vars other than 'buttons' or 'axes': {e}")
            except UnboundLocalError as e:
                self.get_logger().error(f"Wrong form: {e}")
            except Exception as e:
                raise e

        # fill twist msg
        user_msg =  UserCommand()
        user_msg.vx = out_msg.axes[0]
        user_msg.vy = out_msg.axes[1]
        user_msg.wz = out_msg.axes[2]
        
        if out_msg.buttons[9]==1:
            user_msg.motion_mode=-1 # turn off 


        # pub message
        self.pub_joycmd.publish(user_msg)


def main():
    rclpy.init()
    node = JoyRemap(namespace="/ambotw1_ns")
    while rclpy.ok():
        rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    #import argparse
    #parser = argparse.ArgumentParser()
    #parser.add_argument("--namespace", type= str, default=None)
    #args = parser.parse_args()
    main()
