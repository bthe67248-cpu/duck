"""command_node -- turns plain words into robot actions.

Subscribes  /duck/command (std_msgs/String), e.g. "forward 3", "turn left", "sit", "dance", "explore", "stop"
Publishes   /cmd_vel, /head_yaw_cmd, /duck/pose_cmd, /duck/behavior, /nav/enable, /duck/reply (what it understood)
It only publishes topics, so the same node works on the real robot once a hardware node listens to them.

Commands (a trailing number is seconds; default 3):
  forward | back | left | right        walk / turn on the spot          ("turn left 2", "go forward 5")
  step left | step right                side-step
  stop                                  stand still, cancel everything
  sit | stand                           sit down / stand back up
  roll                                  forward roll
  pick                                  touch the ground with the beak
  dance                                 20 s routine
  look left | right | up | down | ahead move the head
  explore                               hand control to the SLAM navigator (it goes to the beacon and maps the room)
  help                                  list the commands
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

WALK, TURN, SIDE = 0.5, 1.2, 0.5
MOVES = {"forward": (WALK, 0.0, 0.0), "back": (-WALK, 0.0, 0.0), "left": (0.0, 0.0, TURN), "right": (0.0, 0.0, -TURN),
         "step left": (0.0, SIDE, 0.0), "step right": (0.0, -SIDE, 0.0)}
LOOKS = {"left": (2, 0.9), "right": (2, -0.9), "up": (1, -0.35), "down": (1, 0.35)}     # (head slot, rad)
ALIASES = {"forwards": "forward", "ahead": "forward", "backward": "back", "backwards": "back", "reverse": "back",
           "halt": "stop", "freeze": "stop", "stand up": "stand", "get up": "stand", "sit down": "sit",
           "flip": "roll", "somersault": "roll", "peck": "pick", "go explore": "explore", "auto": "explore"}
HELP = "forward, back, left, right, step left, step right [seconds] | stop | sit | stand | roll | pick | dance | look left/right/up/down/ahead | explore"


def parse(text):
    """-> (command, seconds or None). Unknown text returns (None, None)."""
    words = text.lower().replace(",", " ").split()
    secs = None
    if words and words[-1].replace(".", "", 1).isdigit():
        secs = float(words.pop())
    words = [w for w in words if w not in ("go", "walk", "turn", "move", "please", "the", "duck", "seconds", "second", "for", "s")]
    phrase = " ".join(words)
    phrase = ALIASES.get(phrase, phrase)
    if phrase in MOVES or phrase in ("stop", "sit", "stand", "roll", "pick", "dance", "explore", "help"):
        return phrase, secs
    if phrase.startswith("look"):
        return phrase, secs
    return None, None


class CommandNode(Node):
    def __init__(self):
        super().__init__("duck_command")
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_head = self.create_publisher(Float64, "head_yaw_cmd", 10)
        self.pub_pose = self.create_publisher(Float32MultiArray, "duck/pose_cmd", 10)
        self.pub_beh = self.create_publisher(String, "duck/behavior", 10)
        self.pub_nav = self.create_publisher(Bool, "nav/enable", 10)
        self.pub_reply = self.create_publisher(String, "duck/reply", 10)
        self.create_subscription(String, "duck/command", self.on_command, 10)
        self.twist = (0.0, 0.0, 0.0); self.until = 0.0; self.moving = False
        self.dance_t0 = None; self.head = [0.0, 0.0, 0.0, 0.0]; self.manual = False
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def reply(self, text):
        self.get_logger().info(text); self.pub_reply.publish(String(data=text))

    def take_over(self):
        """Manual command: switch the navigator off and stop the head sweep."""
        self.pub_nav.publish(Bool(data=False)); self.manual = True
        self.pub_head.publish(Float64(data=0.0))

    def on_command(self, msg):
        cmd, secs = parse(msg.data)
        if cmd is None:
            self.reply(f"did not understand '{msg.data}'. Try: {HELP}"); return
        if cmd == "help":
            self.reply(HELP); return
        if cmd == "explore":
            self.dance_t0 = None; self.moving = False; self.manual = False
            self.pub_pose.publish(Float32MultiArray(data=[0.0] * 10))
            self.pub_nav.publish(Bool(data=True)); self.reply("exploring on my own (SLAM navigator in control)"); return
        self.take_over()
        if cmd == "stop":
            self.dance_t0 = None; self.until = 0.0; self.moving = True       # one zero twist goes out on the next tick
            self.reply("stopping")
        elif cmd in MOVES:
            secs = 3.0 if secs is None else min(secs, 30.0)
            self.dance_t0 = None; self.twist = MOVES[cmd]; self.until = self.now() + secs; self.moving = True
            self.reply(f"{cmd} for {secs:g} s")
        elif cmd in ("sit", "stand", "roll", "pick"):
            self.dance_t0 = None; self.until = 0.0; self.moving = True
            self.pub_beh.publish(String(data=cmd)); self.reply(cmd)
        elif cmd == "dance":
            self.dance_t0 = self.now(); self.reply("dancing for 20 s")
        elif cmd.startswith("look"):
            where = cmd.split()[-1]
            if where in LOOKS:
                slot, val = LOOKS[where]; self.head[slot] = val
            else:
                self.head = [0.0, 0.0, 0.0, 0.0]
            self.pub_pose.publish(Float32MultiArray(data=self.head + [0.0] * 6)); self.reply(f"looking {where}")

    def tick(self):
        if not self.manual:
            return
        t = self.now()
        if self.dance_t0 is not None:
            self.dance(t - self.dance_t0); return
        if self.moving:
            tw = Twist()
            if t < self.until:
                tw.linear.x, tw.linear.y, tw.angular.z = self.twist
            else:
                self.moving = False
            self.pub_cmd.publish(tw)

    def dance(self, t):
        """Same choreography as scripts/motions.py: sway, nod, head shake + bob, spin, finish. 120 bpm."""
        pose = [0.0] * 10; tw = Twist(); ph = 2 * math.pi * t / 1.0
        if t < 4:
            pose[7] = 0.15 * math.sin(ph); pose[3] = -0.3 * math.sin(ph)                 # body roll, head counter-roll
        elif t < 8:
            pose[8] = 0.12 * math.sin(ph); pose[1] = 0.35 * math.sin(2 * ph)             # body pitch, head nod
        elif t < 12:
            pose[2] = 0.8 * math.sin(ph); pose[6] = -0.015 * (1 - math.cos(2 * ph)) / 2  # head shake, bob
        elif t < 16:
            tw.angular.z = 1.5; pose[2] = 0.5                                            # spin
        elif t < 20:
            tw.angular.z = -1.5 if t < 19 else 0.0; pose[7] = 0.12 * math.sin(ph)
        else:
            self.dance_t0 = None; self.reply("dance finished")
        self.pub_pose.publish(Float32MultiArray(data=pose)); self.pub_cmd.publish(tw)


def console():
    """`ros2 run duck_nav duck_say` -> type commands. `ros2 run duck_nav duck_say forward 3` -> send one and exit."""
    import sys
    import threading
    rclpy.init()
    node = Node("duck_say")
    pub = node.create_publisher(String, "duck/command", 10)
    node.create_subscription(String, "duck/reply", lambda m: print(f"  duck: {m.data}", flush=True), 10)
    args = [a for a in sys.argv[1:] if not a.startswith("--ros-args")]
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    import time
    time.sleep(1.0)                                    # let discovery finish before the first message
    if args:
        pub.publish(String(data=" ".join(args))); time.sleep(1.0)
    else:
        print(f"Type a command and press Enter (Ctrl+C to quit).\n  {HELP}")
        try:
            while True:
                line = input("> ").strip()
                if line:
                    pub.publish(String(data=line))
        except (EOFError, KeyboardInterrupt):
            pass
    rclpy.try_shutdown()


def main():
    rclpy.init()
    node = CommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.try_shutdown()
