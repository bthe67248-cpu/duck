"""duck_mujoco_sim -- MuJoCo simulation of the Duckling V3 as a ROS 2 node.

Publishes  /clock (MuJoCo simulation time: every stamp below is sim time; run consumers with use_sim_time),
           /joint_states (sensor_msgs/JointState), /imu (sensor_msgs/Imu),
           /duck/pitch_deg (std_msgs/Float64), TF odom -> trunk_base
Subscribes /duck/push (std_msgs/Float64): forward push in newtons, applied for 0.05 s
           /duck/balance (std_msgs/Bool): balance loop on/off
           /duck/reset (std_msgs/Empty): put the duck back on its feet
The controller is the same one as in Simulink: 100 Hz, complementary tilt filter, PD to ankles.
"""
import math

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Empty, Float64
from tf2_ros import TransformBroadcaster


class DuckSim(Node):
    def __init__(self):
        super().__init__("duck_mujoco_sim")
        self.declare_parameter("mjcf", "")
        self.declare_parameter("k_th", 2.5)
        self.declare_parameter("k_d", 0.20)
        self.m = mujoco.MjModel.from_xml_path(self.get_parameter("mjcf").value)
        self.d = mujoco.MjData(self.m)
        self.key = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_KEY, "sim_start")
        self.home = self.m.key_ctrl[self.key][:14].copy()
        self.jnames = [self.m.joint(i).name for i in range(1, self.m.njnt)]
        self.gyro = self.m.sensor_adr[self.m.sensor("gyro").id]
        self.acc = self.m.sensor_adr[self.m.sensor("accel").id]
        self.push_id = self.m.actuator("push_x").id
        self.dt = 0.01
        self.n_sub = round(self.dt / self.m.opt.timestep)
        self.alpha = 0.5 / (0.5 + self.dt)
        self.balance = True
        self.reset()
        self.pub_clock = self.create_publisher(ClockMsg, "clock", 10)
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_imu = self.create_publisher(Imu, "imu", 10)
        self.pub_pitch = self.create_publisher(Float64, "duck/pitch_deg", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Float64, "duck/push", self.on_push, 10)
        self.create_subscription(Bool, "duck/balance", self.on_balance, 10)
        self.create_subscription(Empty, "duck/reset", lambda _: self.reset(), 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info("Duckling V3 MuJoCo sim running (100 Hz control, 500 Hz physics)")

    def reset(self):
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.key)
        mujoco.mj_forward(self.m, self.d)
        self.th = 0.0
        self.push_force, self.push_steps = 0.0, 0

    def on_push(self, msg):
        self.push_force = float(msg.data)
        self.push_steps = round(0.05 / self.m.opt.timestep)
        self.get_logger().info(f"push {msg.data:.1f} N for 0.05 s = {msg.data * 0.05:.3f} N.s")

    def on_balance(self, msg):
        self.balance = bool(msg.data)
        self.get_logger().info(f"balance loop {'ON' if self.balance else 'OFF'}")

    def step(self):
        d, m = self.d, self.m
        gy, ax, az = d.sensordata[self.gyro + 1], d.sensordata[self.acc], d.sensordata[self.acc + 2]
        self.th = self.alpha * (self.th + gy * self.dt) + (1 - self.alpha) * math.atan2(-ax, az)
        u = self.home.copy()
        if self.balance:
            k_th = self.get_parameter("k_th").value
            k_d = self.get_parameter("k_d").value
            delta = float(np.clip(k_th * self.th + k_d * gy, -0.25, 0.25))
            u[4] += delta       # left ankle
            u[13] -= delta      # right ankle (mirrored joint)
        d.ctrl[:14] = u
        for _ in range(self.n_sub):
            d.ctrl[self.push_id] = self.push_force if self.push_steps > 0 else 0.0
            self.push_steps = max(0, self.push_steps - 1)
            mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)          # sensors/transforms now describe the state just integrated

        now = rclpy.time.Time(nanoseconds=int(round(d.time * 1e9))).to_msg()
        self.pub_clock.publish(ClockMsg(clock=now))
        js = JointState()
        js.header.stamp = now
        js.name = self.jnames
        js.position = d.qpos[7:].tolist()
        js.velocity = d.qvel[6:].tolist()
        self.pub_js.publish(js)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "trunk_base"
        t.transform.translation.x = float(d.qpos[0])
        t.transform.translation.y = float(d.qpos[1])
        t.transform.translation.z = float(d.qpos[2])
        t.transform.rotation.w = float(d.qpos[3])
        t.transform.rotation.x = float(d.qpos[4])
        t.transform.rotation.y = float(d.qpos[5])
        t.transform.rotation.z = float(d.qpos[6])
        self.tf.sendTransform(t)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "trunk_base"
        g = d.sensordata[self.gyro:self.gyro + 3]
        a = d.sensordata[self.acc:self.acc + 3]
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(v) for v in g)
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(v) for v in a)
        imu.orientation_covariance[0] = -1.0     # orientation not provided
        self.pub_imu.publish(imu)
        self.pub_pitch.publish(Float64(data=math.degrees(self.th)))


def main():
    rclpy.init()
    node = DuckSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
