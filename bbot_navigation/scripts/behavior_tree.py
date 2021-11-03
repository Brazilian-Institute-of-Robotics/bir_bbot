#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

import functools
from os import execv
import tf
from tf.transformations import quaternion_from_euler
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import actionlib
import geometry_msgs.msg as geometry_msgs
import mbf_msgs.msg as mbf_msgs
import actionlib_msgs.msg as actionlib_msgs
import std_msgs.msg as std_msgs


##############################################################################
# ToBlackboard
##############################################################################

class SetBuffer(py_trees.blackboard.SetBlackboardVariable):
    def __Buffer_To_Goal(self):
        pose_buffer = py_trees.blackboard.Blackboard().get('pose_buffer')
        pose_buffer.header.stamp = rospy.Time.now()
        target_pose  = pose_buffer
        return target_pose
    
    def initialise(self):
        self.variable_value = self.__Buffer_To_Goal()
        super(SetBuffer, self).initialise()

class MarkerListener(py_trees.behaviour.Behaviour):
    def __init__(self, name,tf1,tf2,xy=(0,0),yaw=0):
        self.tf1 = tf1
        self.tf2 = tf2
        self.tf_listener = tf.TransformListener()
        self.BB = py_trees.blackboard.Blackboard()
        self.tf_msg = geometry_msgs.PoseStamped()
        self.xy = xy
        self.yaw = yaw
        
        super(MarkerListener,self).__init__(name)
    
    def initialise(self):
        self.tf_msg = geometry_msgs.PoseStamped()
        self.tf_msg.header.frame_id = "map"
        self.tf_msg.header.stamp = rospy.Time.now()
        
    def update(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.tf1, self.tf2, rospy.Time(0))
            self.tf_msg.pose.position.x = trans[0] + self.xy[0]
            self.tf_msg.pose.position.y = trans[1] + self.xy[1]
            self.tf_msg.pose.position.z = 0.0
            rot2quaternion = quaternion_from_euler(0,0,self.yaw)       
            self.tf_msg.pose.orientation.x = rot2quaternion[0]
            self.tf_msg.pose.orientation.y = rot2quaternion[1]
            self.tf_msg.pose.orientation.z = rot2quaternion[2]
            self.tf_msg.pose.orientation.w = rot2quaternion[3]
            self.BB.set('target_pose',self.tf_msg)
            self.BB.set('pose_buffer',self.tf_msg)
            return py_trees.Status.SUCCESS
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return py_trees.Status.RUNNING
        
class CheckMarkerPosition(py_trees.behaviour.Behaviour):
    def __init__(self, name=""):
        self.msg1 = geometry_msgs.PoseStamped()
        self.msg2 = geometry_msgs.PoseStamped()
        self.BB = py_trees.blackboard.Blackboard()
        
        super(CheckMarkerPosition, self).__init__(name)
    
    def initialise(self):
        self.msg1 = self.BB.get("target_pose")
        
    def update(self):
        self.msg2 = self.BB.get("target_pose")
        trans_diff = ((self.msg1.pose.position.x - self.msg2.pose.position.x)**2 + (self.msg1.pose.position.y - self.msg2.pose.position.y)**2)**0.5
        if trans_diff > 0.1:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING
 
class GoBackHome(py_trees.behaviour.Behaviour):
    def __init__(self, name, xy, yaw):
        self.BB = py_trees.blackboard.Blackboard()
        self.tf_listener = tf.TransformListener()
        self.tolerance = rospy.get_param("/move_base_flex/DWAPlannerROS/xy_goal_tolerance", default=0.15)
        
        self.home_pose = geometry_msgs.PoseStamped()
        self.home_pose.header.frame_id = "map"
        self.home_pose.pose.position.x = xy[0]
        self.home_pose.pose.position.y = xy[1]
        rot2quaternion = quaternion_from_euler(0,0,yaw)       
        self.home_pose.pose.orientation.x = rot2quaternion[0]
        self.home_pose.pose.orientation.y = rot2quaternion[1]
        self.home_pose.pose.orientation.z = rot2quaternion[2]
        self.home_pose.pose.orientation.w = rot2quaternion[3]
        
        super(GoBackHome, self).__init__(name)       
        
    def initialise(self):
        self.home_pose.header.stamp = rospy.Time.now()
    
    def update(self):
        try:
            (trans,rot) = self.tf_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
            trans_diff =  ((self.home_pose.pose.position.x - trans[0])**2 + (self.home_pose.pose.position.y - trans[1])**2)**0.5
            if trans_diff > self.tolerance:
                self.BB.set("target_pose",self.home_pose)
                self.BB.set("pose_buffer",self.home_pose)
            return py_trees.Status.SUCCESS
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return py_trees.Status.RUNNING
##############################################################################
# Actions
##############################################################################

class CheckGetPathResult(py_trees.behaviours.Failure):
    def setup(self, timeout):
        self.action_client = actionlib.SimpleActionClient("/move_base_flex/get_path", mbf_msgs.GetPathAction)
        return True
    
    def initialise(self):
        outcome = py_trees.blackboard.Blackboard().get("get_path_outcome").outcome
        # rospy.loginfo("get_path_outcome = "+str(outcome))
        if outcome == 55 or outcome == 50: #Goal on obstacle
            rospy.Publisher(f"{ns}/cancel_action",std_msgs.Empty,queue_size=10).publish(std_msgs.Empty()) #Cancel in behavior tree

class GetPath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target pose from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.GetPathGoal(target_pose=py_trees.blackboard.Blackboard().get("target_pose"))
        super(GetPath, self).initialise()

    def update(self):
        """
        On success, set the resulting path on the blackboard, so ExePath can use it
        """
        if py_trees.blackboard.Blackboard().get("InRecovery") == False:
            status = super(GetPath, self).update()
            if status == py_trees.Status.SUCCESS:
                py_trees.blackboard.Blackboard().set("path", self.action_client.get_result().path)
            return status
        return py_trees.Status.FAILURE
    
    def terminate(self, new_status):
        py_trees.blackboard.Blackboard().set("get_path_outcome",self.action_client.get_result())
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

class ExePath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get path from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.ExePathGoal(path=py_trees.blackboard.Blackboard().get("path"))
        super(ExePath, self).initialise()

class Recovery(py_trees_ros.actions.ActionClient):
    def setup(self, timeout):
        """
        Read the list of available recovery behaviors so we can try them in sequence
        """
        self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
        return super(Recovery, self).setup(timeout)
    
    def initialise(self):
        py_trees.blackboard.Blackboard().set("InRecovery",True)
        super(Recovery, self).initialise()

    def update(self):
        try:
            self.logger.debug("{0}.update()".format(self.__class__.__name__))
            if not self.action_client:
                self.feedback_message = 'no action client, did you call setup() on your tree?'
                return py_trees.Status.INVALID
            if not self.sent_goal:
                self.action_goal = mbf_msgs.RecoveryGoal(behavior=self._behaviors.pop(0)["name"])
                self.action_client.send_goal(self.action_goal)
                self.sent_goal = True
                self.feedback_message = 'sent goal to the action server'
                return py_trees.Status.RUNNING
            self.feedback_message = self.action_client.get_goal_status_text()
            if self.action_client.get_state() in [actionlib_msgs.GoalStatus.ABORTED,
                                                actionlib_msgs.GoalStatus.PREEMPTED]:
                return py_trees.Status.FAILURE
            result = self.action_client.get_result()
            if result:
                target_pose = py_trees.blackboard.Blackboard().get("target_pose")
                target_pose.header.stamp = rospy.Time.now()
                py_trees.blackboard.Blackboard().set("target_pose",target_pose)
                return py_trees.Status.SUCCESS
            else:
                self.feedback_message = 'bla' #self.action_goal.behavior #self.override_feedback_message_on_running
                return py_trees.Status.RUNNING
        except IndexError:
            self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
            return py_trees.Status.FAILURE

    def terminate(self, new_status):
        py_trees.blackboard.Blackboard().set("InRecovery",False)
        self.logger.debug("%s.terminate(%s)" % (self.__class__.__name__, "%s->%s" % (self.status, new_status) if self.status != new_status else "%s" % new_status))
        if self.action_client is not None and self.sent_goal:
            motion_state = self.action_client.get_state()
            if ((motion_state == actionlib_msgs.GoalStatus.PENDING) or (motion_state == actionlib_msgs.GoalStatus.ACTIVE) or
               (motion_state == actionlib_msgs.GoalStatus.PREEMPTING) or (motion_state == actionlib_msgs.GoalStatus.RECALLING)):
                self.action_client.cancel_goal()
        self.sent_goal = False

##############################################################################
# Behaviours
##############################################################################

def create_root():
    # Create all behaviors
    bt_root = py_trees.composites.Parallel("MBF")
    bt = py_trees.composites.Sequence("BT")

    get_goal = py_trees.composites.Selector("GetGoal")
    # get_buffer = py_trees.composites.Selector("GetBuffer")
    apply_buffer = py_trees.composites.Sequence("ApplyBuffer")
    chk_preemp = py_trees.composites.Selector("CheckPreemption")
    fallback = py_trees.composites.Selector("Fallback")
    navigate = py_trees.composites.Sequence("Navigate")
    new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                     topic_name="/move_base_simple/goal",
                                                     topic_type=geometry_msgs.PoseStamped,
                                                     blackboard_variables = {'target_pose': None,'pose_buffer': None})
    
    have_goal = py_trees.blackboard.CheckBlackboardVariable(name="HaveGoal?", variable_name="target_pose")
    have_buffer = py_trees.blackboard.CheckBlackboardVariable(name="HaveBuffer?", variable_name="pose_buffer")
    
    clr_goal1 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_buffer = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoalBuffer", variable_name="pose_buffer")
    clr_goal2 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    
    set_buffer = SetBuffer(name="SetBuffer",variable_name='target_pose',variable_value=None)
    set_preempt = py_trees.meta.success_is_running(py_trees.meta.running_is_failure(py_trees_ros.subscribers.ToBlackboard))(name="GotPreemption?",
                                                                    topic_name="/move_base_simple/goal",
                                                                    topic_type=geometry_msgs.PoseStamped,
                                                                    blackboard_variables = {'target_pose': None,'pose_buffer': None})
    chk_marker_pos = py_trees.meta.success_is_running(py_trees.meta.running_is_failure(CheckMarkerPosition))(name="CheckArucoPos")
    
    GetPath_seq = py_trees.composites.Selector("GetPathSeq")
    get_path = GetPath(name="GetPath",
                       action_namespace="/move_base_flex/get_path",
                       action_spec=mbf_msgs.GetPathAction)
    check_getpath_result = CheckGetPathResult(name="CheckGetPathResult")
    
    exe_path = ExePath(name="ExePath",
                       action_namespace="/move_base_flex/exe_path",
                       action_spec=mbf_msgs.ExePathAction)
    
    
    recovery = Recovery(name="Recovery",
                        action_namespace="/move_base_flex/recovery",
                        action_spec=mbf_msgs.RecoveryAction)


    cancelling = py_trees.composites.Sequence("Cancelling?")
    cancel2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cancel2BB",
        topic_name=f"{ns}/cancel_action",
        variable_name="event_cancel_button")
    is_cancel_requested = py_trees.blackboard.CheckBlackboardVariable(
        name="Cancel?",
        variable_name='event_cancel_button',
        expected_value=True
    )
    
    marker2bb = MarkerListener("Aruco2BB","/map","/aruco_id_0",(2,1),1.57)
    
    back_home = GoBackHome("GoHome",(-3.0,-2.8),1.57)

    clr_goal3 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_buffer3 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoalBuffer", variable_name="pose_buffer")

    # Compose tree
    bt_root.add_children([cancel2bb,marker2bb, bt])
    bt.add_children([get_goal, chk_preemp])
    chk_preemp.add_children([cancelling, set_preempt, chk_marker_pos,fallback])
    cancelling.add_children([is_cancel_requested, clr_goal3, clr_buffer3])
    get_goal.add_children([have_goal, apply_buffer, new_goal])
    # get_buffer.add_children([apply_buffer,new_goal])
    apply_buffer.add_children([have_buffer,set_buffer])
    navigate.add_children([GetPath_seq, exe_path, clr_goal1,clr_buffer,back_home])
    GetPath_seq.add_children([get_path,check_getpath_result])
    fallback.add_children([navigate, recovery,clr_goal2])
    
    #Initialise Blackboard Varibles
    py_trees.blackboard.Blackboard().set("InRecovery",False)
    return bt_root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    rospy.init_node("BehaviorTree")
    ns = rospy.get_name() # Node name
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)