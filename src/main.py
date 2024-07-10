#!/usr/bin/env python

import rospy
import math
from xarm_msgs.srv import *
from xarm_msgs.msg import *

HOME_POSITION = [-math.pi / 2, 0, 0, 0, 0, 0, 0] # starting position for this demo

# XY grid configuration
X_STEPS = 8; X_STEP_LEN = 40.00 # number of steps and length of each step (in mm)
Y_STEPS = 8; Y_STEP_LEN = -40.00
DZ_DOWN = 80.00 # delta Z for moving end effector down

# end effector angle test configuration
DZ_UP = -150.0 # delta Z for clearing the workbench
DRX = [math.pi / 2, -math.pi]
DRY = [math.pi / 2, -math.pi]
DRZ = [math.pi / 2, -math.pi]

class MoveTest:
    def __init__(self):
        rospy.loginfo('waiting for services to come online')
        rospy.wait_for_service('/xarm/motion_ctrl')
        rospy.wait_for_service('/xarm/set_mode')
        rospy.wait_for_service('/xarm/set_state')
        rospy.wait_for_service('/xarm/move_joint')
        rospy.wait_for_service('/xarm/move_line_tool')

        rospy.logdebug('setting up service proxies')
        self.srv_set_axis = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
        self.srv_set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
        self.srv_set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
        self.srv_move_joint = rospy.ServiceProxy('/xarm/move_joint', Move)
        self.srv_move_line_tool = rospy.ServiceProxy('/xarm/move_line_tool', Move)

        rospy.logdebug('subscribing to telemetry')
        rospy.Subscriber('/xarm/xarm_states', RobotMsg, self.telemetry_cb)

        # initialise telemetry fields
        self.telemetry_available = False # set when we start receiving telemetry data
        self.tcp_pose = [float('nan')] * 6 # x, y, z, rx, ry, rz
        self.busy = False # set when the arm is moving
        self.last_state = 2 # last state reported

    def init(self): # initialise robot
        rospy.loginfo('initialising robot')
        self.srv_set_axis(8, 1)
        self.srv_set_mode(0)
        self.srv_set_state(0)

        rospy.loginfo('waiting for telemetry to become available')
        while not self.telemetry_available: pass

        rospy.loginfo('robot is ready')
    
    def move_joint(self, j1: float = 0, j2: float = 0, j3: float = 0, j4: float = 0, j5: float = 0, j6: float = 0, j7: float = 0, maxvel: float = 0.7, maxacc: float = 14.0):
        rospy.loginfo(f'srv: /xarm/move_joint [{j1},{j2},{j3},{j4},{j5},{j6},{j7}] {maxvel} {maxacc} 0 0')
        self.srv_move_joint([j1,j2,j3,j4,j5,j6,j7], maxvel, maxacc, 0, 0)
        self.busy = True # for blocking operations
    
    def set_home_pose(self):
        # while self.busy: pass # wait until we're no longer busy
        self.telemetry_available = False # discard current telemetry
        while not self.telemetry_available: pass # wait until we have new telemetry
        while self.last_state == 1: pass # and our telemetry indicates the robot is stopped

        self.home_pose = self.tcp_pose
        rospy.loginfo(f'home pose: x={self.home_pose[0]},y={self.home_pose[1]},z={self.home_pose[2]} (mm) ; rx={self.home_pose[3]},ry={self.home_pose[4]},rz={self.home_pose[5]} (rad)')

    def move_joint(self, joints: 'list[float]' = [0, 0, 0, 0, 0, 0, 0], maxvel: float = 0.7, maxacc: float = 14.0):
        rospy.loginfo(f'srv: /xarm/move_joint {joints} {maxvel} {maxacc} 0 0')
        self.srv_move_joint(joints, maxvel, maxacc, 0, 0)
        self.busy = True # for blocking operations

    def move_line_tool(self, dx: float = 0, dy: float = 0, dz: float = 0, drx: float = 0, dry: float = 0, drz: float = 0, maxvel: float = 400, maxacc: float = 4000):
        rospy.loginfo(f'srv: /xarm/move_line_tool [{dx},{dy},{dz},{drx},{dry},{drz}] {maxvel} {maxacc} 0 0')
        self.srv_move_line_tool([dx,dy,dz,drx,dry,drz], maxvel, maxacc, 0, 0)
        self.busy = True # for blocking operations
    
    def telemetry_cb(self, data):
        # rospy.loginfo(f'state: {data.state}, err: {data.err}, cmdnum: {data.cmdnum}')
        # rospy.loginfo(f'TCP pose: x={data.pose[0]},y={data.pose[1]},z={data.pose[2]} (mm) ; rx={data.pose[3]},ry={data.pose[4]},rz={data.pose[5]} (rad)')
        
        # extract telemetry
        self.tcp_pose = data.pose
        if self.last_state == 1 and data.state != 1: self.busy = False
        elif self.last_state != 1 and data.state == 1: self.busy = True # busy busy        
        self.telemetry_available = True
        self.last_state = data.state
    
    def is_busy(self) -> bool:
        return (not self.telemetry_available) or self.busy # if there's no telemetry then we'll assume the arm is busy

    def run(self): # our main procedure
        self.init()

        rospy.loginfo('moving joint to home position')
        self.move_joint(HOME_POSITION) # go back to our home position
        self.set_home_pose()

        for y in range(Y_STEPS):
            for x in range(X_STEPS):
                rospy.loginfo(f'grid: x = {x}, y = {y}')

                self.move_line_tool(dz = DZ_DOWN) # lower end effector
                self.move_line_tool(dz = -DZ_DOWN) # raise end effector again (this will be queued after the above)
                self.move_line_tool(dx = X_STEP_LEN) # move to next cell on this row
            
            self.move_line_tool(dx = -X_STEPS * X_STEP_LEN, dy = Y_STEP_LEN) # move back to beginning of next row
            while self.is_busy(): pass # queue commands for entire row

        rospy.loginfo(f'moving to grid centre')
        self.move_line_tool(dx = X_STEPS / 2 * X_STEP_LEN, dy = -(1 + Y_STEPS / 2) * Y_STEP_LEN, dz = DZ_UP) # move to centre of grid for next demo
        while self.is_busy(): pass

        for d in DRX:
            rospy.loginfo(f'drx = {d}')
            self.move_line_tool(drx = d)
        while self.is_busy(): pass

        for d in DRZ:
            rospy.loginfo(f'drz = {d}')
            self.move_line_tool(drz = d)
        while self.is_busy(): pass

        for d in DRY:
            rospy.loginfo(f'dry = {d}')
            self.move_line_tool(dry = d)
        while self.is_busy(): pass

        rospy.loginfo('demo complete, moving back to home')
        self.move_joint(HOME_POSITION)

if __name__ == '__main__':
    rospy.init_node('move_test')

    mvtest = MoveTest()
    mvtest.run()