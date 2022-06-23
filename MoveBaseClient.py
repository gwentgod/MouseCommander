import numpy as np
from scipy.spatial.transform import Rotation

import rospy as ros
import actionlib
from move_base_msgs.msg import *


class MoveBaseClient:
    def check_stuck(self, event):
        ros.loginfo('Checking stuck')
        if self.last_pose is not None and self.is_active:
            displacement = np.linalg.norm(self.pose - self.last_pose)
            if displacement < 0.01:
                ros.logwarn("I'm stuck! Please plan a new path!")
                self.reset()
                return
        self.last_pose = self.pose

    def reset(self):
        self.current_index = 0

        self.last_pose = None

        self.sketchpad.reset_path()

        if self.is_active:
            self.action_client.cancel_goal()
            self.is_active = False

        ros.loginfo('Reseted')

    def set_milestones(self, milestones):
        self.milestones = milestones.astype('float64') / np.array([40, -40]) + np.array([-10, 9])
        if not self.is_active:
            self.is_active = True
            self.send_goal()
        ros.loginfo('Got new milestones')

    def __init__(self, sketchpad):
        self.sketchpad = sketchpad
        self.is_active = False
        self.milestones = None
 
        self.pose = None
        ros.init_node("lineFollowClient")
        ros.sleep(1)

        self.goal_msg = MoveBaseGoal()
        self.goal_msg.target_pose.header.frame_id = 'map'

        self.action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        ros.loginfo('Connecting to action server')
        self.action_client.wait_for_server()

        self.timer = ros.Timer(ros.Duration(3), self.check_stuck)

    def feedback_callback(self, feedback):
        self.pose = np.array([feedback.base_position.pose.position.x, feedback.base_position.pose.position.x])

    def active_callback(self):
        ros.loginfo(f'Activated')

    def done_callback(self, status, result):
        if self.current_index < self.milestones.shape[0]-1 and status != 2:
            self.current_index += 1
            self.send_goal()
            self.sketchpad.mark_milestones()
        else:
            self.is_active = False
            self.reset()
            ros.loginfo("Finished")

    def send_goal(self):
        if self.is_active:
            current_milestone = self.milestones[self.current_index]
            self.goal_msg.target_pose.pose.position.x = current_milestone[0]
            self.goal_msg.target_pose.pose.position.y = current_milestone[1]
            
            if self.current_index < self.milestones.shape[0]-1:
                diff = self.milestones[self.current_index+1] - current_milestone
            else:
                diff = np.array([0, 1])
            r = Rotation.from_euler('z', np.arctan2(*diff[::-1]))
            quat = r.as_quat()
            self.goal_msg.target_pose.pose.orientation.x = quat[0]
            self.goal_msg.target_pose.pose.orientation.y = quat[1]
            self.goal_msg.target_pose.pose.orientation.z = quat[2]
            self.goal_msg.target_pose.pose.orientation.w = quat[3]
            ros.loginfo(f"Sending goal: {self.goal_msg.target_pose.pose}")
            self.action_client.send_goal(self.goal_msg,
                                         feedback_cb=self.feedback_callback,
                                         active_cb=self.active_callback,
                                         done_cb=self.done_callback)
