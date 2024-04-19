#!/usr/bin/env python3
import rospy
import rospkg

import time
import yaml
import math
import numpy as np
from functools import partial
from pyquaternion import Quaternion
from threading import Lock

from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

from std_srvs.srv import Trigger, TriggerResponse

import task_viz

TASK1_PENALTY = 10
TASK1_TIME_LIMIT = 20
TASK2_TIME_LIMIT = 30
TASK2_ROT_THREDSHOLD = 0.5

class TaskEval:
    def __init__(self):
        # get parameters
        self.task = rospy.get_param("/task")
        self.task_file = rospy.get_param("/task_file")
        
        self.state_lock = Lock()

        self.goal_id = -1 # not started
        self.total_score = 0
        self.current_pom = None
        self.current_normal = []
        
        
        # get task specification
        if self.task == 1:
            self.tag_id = rospy.get_param("/tag")
            self.time_limit = 20
            with open(self.task_file) as f:
                try:
                    self.task_spec = yaml.safe_load(f)["waypoints"]
                    self.num_waypoints = len(self.task_spec)
                except yaml.YAMLError as exc:
                    rospy.logerr('Invalid Task File')
                    print(exc)
                    return

        elif self.task == 2:
            self.time_limit = 30
            self.cube_file = rospy.get_param("/cube_file")
            with open(self.cube_file) as f:
                try:
                    self.cube_tags = {}
                    self.face_id = {}
                    tags = yaml.safe_load(f)["faces"]
                    for i, face in enumerate(tags):
                        for tag in tags[face]["tag"]:
                            self.cube_tags[tag] = i
                        self.face_id[face] = i
                        # self.cube_normal[face] = tags[face]["normal"]
                    
                except yaml.YAMLError as exc:
                    rospy.logerr('Invalid Cube File')
                    print(exc)
                    return
                
            with open(self.task_file) as f:
                try:
                    self.task_spec = yaml.safe_load(f)["faces"]
                    self.num_waypoints = len(self.task_spec)
                except yaml.YAMLError as exc:
                    rospy.logerr('Invalid Cube File')
                    print(exc)
                    return
        else:
            print("error")
            return
        
        # setup subscribers
        self.tag_subsriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback_tag, queue_size=1)

        # setup publisher
        if self.task == 1:
            self.goal_pub = rospy.Publisher('rgcm_eval/task1/goal', Point, queue_size=1)
        if self.task == 2:
            self.goal_pub = rospy.Publisher('rgcm_eval/task2/goal', String, queue_size=1)
        # set up services
        self.start = rospy.Service('/rgcm_eval/start', Trigger, self.handle_start_task)
        self.record = rospy.Service('/rgcm_eval/record', Trigger, self.handle_record_waypoint)
        self.stop = rospy.Service('rgcm_eval/stop', Trigger, self.handle_stop_task)

        

    def callback_tag(self, msg):
        self.state_lock.acquire()
        detections = msg.detections
        self.current_normal = [[]] * 6
        self.current_pom = None
        for tag in detections:
            if self.task == 1 and self.tag_id in tag.id:
                self.current_pom = tag.pose.pose.pose

            if self.task == 2 and tag.id[0] in self.cube_tags:
                pose = tag.pose.pose.pose
                p = [pose.position.x, pose.position.y, pose.position.z]
                q = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                n = list(q.rotate([0, 0, 1]))
                i = self.cube_tags[tag.id[0]]
                self.current_normal[i]= self.current_normal[i] + [p + n]
                
        if self.task == 2:
            task_viz.viz_cube(self.current_normal)
        self.state_lock.release()

    def viz_task1(self):
        task_viz.viz_traj(self.all_traj)
        task_viz.viz_waypoint(self.all_waypoint)
        task_viz.viz_goal(self.goal)

    def handle_start_task(self, _):
        if self.current_pom is None and len(self.current_normal) == 0:
            return TriggerResponse(False, "Unable to detect the object")
        elif self.goal_id != -1:
            return TriggerResponse(False, "The task is started")
        
        self.state_lock.acquire()
        if self.task == 1:
            # Task 1 Start
            self.goal_id = 0
            cur = self.current_pom
            self.start_pose = [cur.position.x, cur.position.y, cur.position.z]
            self.goal = [self.start_pose[0] + float(self.task_spec[self.goal_id]["x"]),
                              self.start_pose[1] + float(self.task_spec[self.goal_id]["y"]),
                              self.start_pose[2] + float(self.task_spec[self.goal_id]["z"])]
            self.all_traj = [self.start_pose]
            self.all_waypoint = [self.start_pose]
            self.total_score = 0
            self.viz_task1()
            
        elif self.task == 2:
            # Task 2 Start
            self.goal_id = 0
            self.total_success = 0

        self.start_time = time.time()
        self.pub_timer = rospy.Timer(rospy.Duration(1.0/10), self.handle_goal_pub, self.goal_id)
        self.exec_timer = rospy.Timer(rospy.Duration(self.time_limit), self.handle_timeout, oneshot=True)  
        
        self.state_lock.release()
        return TriggerResponse(True, "Start Task%d"%(self.task))

    def handle_record_waypoint(self, _):
        if self.goal_id < 0:
            return (False, "The Task%d is not started yet."%(self.task))
        
        self.state_lock.acquire()
        t = time.time() - self.start_time
        if self.task == 1:
            # Task 1 Record
            self.exec_timer.shutdown()
            if self.current_pom is None:
                dis = 10
            else:
                cur = self.current_pom
                cur_pose = [cur.position.x, cur.position.y, cur.position.z]
                dis = math.sqrt((cur_pose[0] - self.goal[0]) * (cur_pose[0] - self.goal[0]) + 
                                (cur_pose[1] - self.goal[1]) * (cur_pose[1] - self.goal[1]) +
                                (cur_pose[2] - self.goal[2]) * (cur_pose[2] - self.goal[2])) * 100
            self.total_score += dis 

            # visualization
            self.all_traj += [cur_pose]
            self.all_waypoint += [self.goal]
            self.viz_task1()
            msg = "Current Waypoint ID [%d]\nDistance Error [%f] cm\nAccumulated Error [%f] cm\nRemaining Waypoints [%d]\nAccumulated Execution Time [%f] s"%(
                    self.goal_id, dis, self.total_score, self.num_waypoints - self.goal_id - 1, t)
            print(msg)
            print("---------------------")

            self.goal_id += 1
            if self.goal_id < self.num_waypoints:
                self.goal = [self.start_pose[0] + float(self.task_spec[self.goal_id]["x"]),
                                self.start_pose[1] + float(self.task_spec[self.goal_id]["y"]),
                                self.start_pose[2] + float(self.task_spec[self.goal_id]["z"])]
                task_viz.viz_goal(self.goal)
            else:
                self.handle_stop_task(_)
                self.pub_timer.shutdown()
                # print("Finished Task 1 Total Error %f"%(self.total_score))
                self.state_lock.release()
                return TriggerResponse(True, msg)
            
        elif self.task == 2:
            # Task 2 Record
            self.exec_timer.shutdown()
            
            i = self.face_id[self.task_spec[self.goal_id]]
            result = 0
            if len(self.current_normal[i]) > 0:
                for j in range(len(self.current_normal[i])):
                    d = np.arccos(np.dot(self.current_normal[i][j][3:], [0, 0, -1]))
                    if d <= TASK2_ROT_THREDSHOLD:
                        result = 1

            self.total_success += result
            msg = "Current Tartget [%s]\nResult [%s]\nProgress [%d/%d]\nRemaining Targets [%d]\nAccumulated Execution Time [%f] s"%(
                    self.task_spec[self.goal_id],"Success" if result==1 else "Failure",  self.total_success, self.goal_id + 1, self.num_waypoints - self.goal_id - 1, t)
            print(msg)
            print("---------------------")

            self.goal_id += 1
            if self.goal_id >= self.num_waypoints:
                self.handle_stop_task(_)
                self.pub_timer.shutdown()
                # print("Finished Task 2 Total Score %d/%d"%(self.total_success, self.num_waypoints))
                self.state_lock.release()
                return TriggerResponse(True, msg)

            
        self.exec_timer = rospy.Timer(rospy.Duration(self.time_limit), self.handle_timeout, oneshot=True)
        self.state_lock.release()
        return TriggerResponse(True, msg)

    def handle_stop_task(self, _):
        if self.goal_id == -1:
            return TriggerResponse(False, "No Task Running")
        self.goal_id = -1
        self.pub_timer.shutdown()
        self.exec_timer.shutdown()
        return TriggerResponse(True, "Stopped Task")

    def handle_timeout(self, _):
        print("Time out for Waypoint [%d]"%(self.goal_id))
        self.handle_record_waypoint(None)
    
    # Publish goal state every 1/10 seconds
    def handle_goal_pub(self, _):
        if self.task == 1:
            p = Point()
            p.x = self.goal[0]
            p.y = self.goal[1]
            p.z = self.goal[2]
            self.goal_pub.publish(p)
        elif self.task == 2:
            s = String()
            s.data = self.task_spec[self.goal_id]
            self.goal_pub.publish(s)



if __name__ == "__main__":
    rospy.init_node("task evaluation", anonymous=True)
    node = TaskEval()
    rospy.spin()