#! /usr/bin/python
################################################################
### This is a node for generating the workspace for          ###
### an ABB industrial arm (specifically the IRB1200 7/0.7)   ###
### author: Oliver Ebeling-Koning <odek@vt.edu>              ###
### date: 09/06/2015                                         ###
################################################################

import rospy
from sensor_msgs.msg import JointState
import xml.dom.minidom
import time
import copy

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class WorkspaceMapper():
    def __init__(self):
        self.pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        description = get_param('robot_description')
        try:
            robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        except:
            print "ERROR: please load the robot model into the param server first"
            rospy.signal_shutdown("Could not parse robot model")
            exit(1)

        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = get_param("dependent_joints", {})
        use_mimic = get_param('use_mimic_tags', True)
        use_small = get_param('use_smallest_joint_limits', True)

        self.zeros = get_param("zeros")

        pub_def_positions = get_param("publish_default_positions", True)
        pub_def_vels = get_param("publish_default_velocities", False)
        pub_def_efforts = get_param("publish_default_efforts", False)

        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                self.joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if use_small and len(safety_tags)==1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if use_mimic and len(mimic_tags)==1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    self.dependent_joints[name] = entry
                    continue

                if name in self.dependent_joints:
                    continue

                if self.zeros and name in self.zeros:
                    zeroval = self.zeros[name]
                elif minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval}
                if pub_def_positions:
                    joint['position'] = zeroval
                if pub_def_vels:
                    joint['velocity'] = 0.0
                if pub_def_efforts:
                    joint['effort'] = 0.0

                if jtype == 'continuous':
                    joint['continuous'] = True
                self.free_joints[name] = joint


    def computePositionsForNextJoint(self, currPositions, jointsToSweep, resolutionOfSweeps, maxs, mins, joint_list):
        if len(jointsToSweep) != 0:    
            for kk in range(0, int(1.0/resolutionOfSweeps)): # The actual sweeping
                currPositions[joint_list.index(jointsToSweep[0])] 
                currPositions[joint_list.index(jointsToSweep[0])] = mins[joint_list.index(jointsToSweep[0])]+(resolutionOfSweeps*kk)*(maxs[joint_list.index(jointsToSweep[0])]-mins[joint_list.index(jointsToSweep[0])])
                if (len(jointsToSweep) > 1):
                    self.computePositionsForNextJoint(currPositions, jointsToSweep[1:len(jointsToSweep)], resolutionOfSweeps, maxs, mins, joint_list)

                # Combine message and publish
                msg = JointState()
                msg.header.stamp = rospy.Time.now()

                msg.name = self.joint_list
                # print self.free_joints
                msg.position = currPositions
                msg.velocity = [0.0]*len(maxs)
                msg.effort = [0.0]*len(maxs)
                # time.sleep(0.005)
                self.pub.publish(msg)

    def mainLoop(self):
        while not rospy.is_shutdown():
            print "\n\nStarting up..."
            time.sleep(20) # Wait for rviz to start up
            # EndEffectorPositions

            # Get all values ready for message
            maxs = []
            mins = []
            zeros = []
            currPositions = []
            for joint in self.joint_list:
                maxs.append(self.free_joints[joint]["max"])
                mins.append(self.free_joints[joint]["min"])
                zeros.append(self.free_joints[joint]["zero"])
                currPositions.append(self.free_joints[joint]["position"])

            # Sweep all values within limits
            reverse_joint_list = copy.copy(self.joint_list)
            reverse_joint_list.reverse()
            resolutionOfSweeps = 0.01



            for ii,j1 in enumerate(reverse_joint_list): # For each joint we want to test
                jointsToSweep = reverse_joint_list[0:ii]
                jointsToSweep.reverse()
                self.computePositionsForNextJoint(currPositions, jointsToSweep, resolutionOfSweeps, maxs, mins,self.joint_list)                    





if __name__ == '__main__':
    try:
        rospy.init_node('workspace_mapper')
        wm = WorkspaceMapper()
        wm.mainLoop()

    except rospy.ROSInterruptException: pass
