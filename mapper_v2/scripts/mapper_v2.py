#! /usr/bin/python
#################################################################
### This is a 2nd version node for generating the workspace   ###
### for an ABB industrial arm (specifically the IRB1200 7/0.7)###
### author: Oliver Ebeling-Koning <odek@vt.edu>               ###
### date: 03/06/2016                                          ###
#################################################################

import rospy
from sensor_msgs.msg import JointState
import xml.dom.minidom
import time, copy, tf, csv
import sys, os
from numpy import arange
import moveit_commander
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class WorkspaceMapper():
    def __init__(self, csvFileName, resolutionOfSweeps, baseLink, finalLink):
        if not rospy.is_shutdown():

            # Open up csv file for writing 
            self.csvFile = open(csvFileName, 'w+')
            self.csvWriter = csv.writer(self.csvFile)
            # self.csvWriter.writerow(trans)

            rospy.loginfo("waiting for compute_ik service...")
            rospy.wait_for_service('/compute_ik')
            rospy.loginfo("found compute_ik service")

            # Iterate over all points that we want to check
            reachablePoints = []
            
            for xx in arange(0.2, 0.8, float(resolutionOfSweeps)):
                for yy in arange(-0.75, 0.75, float(resolutionOfSweeps)):
                    for zz in arange(0.0, 1.1, float(resolutionOfSweeps)):
                        if self.computeIKPossible(xx, yy, zz):
                            self.csvWriter.writerow((xx, yy, zz))
                            self.csvFile.flush()
                            # reachablePoints.append(xx, yy, zz)
            self.csvFile.close()
            print "\n\t\t****Complete!\n\n"   

    # This is a recursive function to compute all possible combinations of joint orientations follwing the current joint
    def computeIKPossible(self, x, y, z, qx=0.707, qy=0.707, qz=0.0, qw=0.0):
        req = GetPositionIKRequest()
        req.ik_request.group_name = "manipulator"
        req.ik_request.robot_state = RobotState()
        # req.ik_request.robot_state.joint_state = self.joint_state
        req.ik_request.avoid_collisions = True 
        req.ik_request.pose_stamped = PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose.position.x = x
        req.ik_request.pose_stamped.pose.position.y = y
        req.ik_request.pose_stamped.pose.position.z = z
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        req.ik_request.timeout = rospy.Duration(0.05)

        try:
            serviceCallLink = rospy.ServiceProxy('/compute_ik', GetPositionIK)
            response = serviceCallLink(req)
            # print response
            if response.error_code.val is not 1: # if nost success then issue
                print "No solution at %0.2f, %0.2f, %0.2f" % (x,y,z)
            else:
                return True
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)        
        return False

if __name__ == '__main__':
    # Check arguments
    if len(sys.argv) < 7 or len(sys.argv) > 7:
        print "ERROR: wrong number of arguments. (%d given) \n\tUSAGE: roslaunch robot_arm_workspace_mapper <launch_file> arg_name:=<save_path_of_output_csv_file> <resolution_of_sweeps_on_axis> <base_link_name_in_tf> <end_effector_link_name_in_tf>" % len(sys.argv)
        exit(1)
    else:
        csvFileName = os.path.expanduser(sys.argv[1])
        print "\nSaving file to location: %s" % csvFileName
        if os.path.exists(csvFileName):
            print "ERROR: cannot overwrite file %s" % csvFileName
            exit(1)
        else:
            try:
                rospy.init_node('workspace_mapper')
                wm = WorkspaceMapper(csvFileName, sys.argv[2], sys.argv[3], sys.argv[4])
            except rospy.ROSInterruptException: pass
