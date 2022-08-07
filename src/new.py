#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State,WaypointList,GlobalPositionTarget,AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest,WaypointSetCurrent
from math import sin, cos, sqrt, atan2, radians
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import numpy as np

from simple_pid import PID
pid = PID(1, 0.1, 0.05, setpoint=1)
pid.sample_time = 0.01  # update every 0.01 seconds


current_state = State()
curr_wp =WaypointList()
lpose = AttitudeTarget()
gpose= GlobalPositionTarget()
lvel=TwistStamped()
lat_list=[]
lon_list=[]
alt_list=[]
curr_wp_no=0
dist=0
heading=0
curr_lat=0
curr_lon=0
curr_wp_lat=0
curr_wp_lon=0
req_thr=0.5



def state_cb(msg):
    global current_state
    current_state = msg
def wp_cb(data):
    global curr_wp_no
    for i in range(len(data.waypoints)):
        lat= data.waypoints[i].x_lat
        lon= data.waypoints[i].y_long
        alt=data.waypoints[i].z_alt
        lat_list.append(lat)
        lon_list.append(lon)
        alt_list.append(alt)
        if(data.waypoints[i].is_current):
            curr_wp_no=i-1

def distance_meas(curr_lat,curr_lon,curr_wp_lat,curr_wp_lon):
    global dist, heading
    R = 6373.0

    curr_lat=radians(curr_lat)
    curr_lon=radians(curr_lon)
    curr_wp_lat=radians(curr_wp_lat)
    curr_wp_lon=radians(curr_wp_lon)

    dlon = curr_wp_lon - curr_lon
    dlat = curr_wp_lat - curr_lat
    a = (sin(dlat/2))**2 + cos(curr_wp_lat) * cos(curr_lat) * (sin(dlon/2))**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    dist = R * c*1000   # distance in meters

    heading = atan2(sin(-dlon)*cos(curr_lat), cos(curr_wp_lat)*sin(curr_lat) - sin(curr_wp_lat)*cos(curr_lat)*cos(-dlon))
    # print((heading*180/3.1416+360)%360)

def loc_cb(msg):
    global curr_alt
    curr_alt=msg.pose.pose.position.z

def pos_cb(msg):
    global curr_lat,curr_lon
    curr_lat=msg.latitude
    curr_lon=msg.longitude

def att_controller():

    lpose.header.stamp=rospy.Time.now()
    lpose.header.frame_id = 'global'
    # lpose.pose.position.x=0
    # lpose.pose.position.x=0
    # lpose.pose.position.x=0
    lpose.type_mask=128
    lpose.orientation.x = 0.999
    lpose.orientation.y = 0
    lpose.orientation.z = -0.0437
    lpose.orientation.w = 0
    local_pos_pub.publish(lpose)
    cal_thrust()                                   # thrust calculation is done using PID control
    lpose.thrust=req_thr                           # a controlled thrust value is needed

def cal_thrust():
    global req_thr
    # req_thr=(nxt_wp_alt-curr_alt)
    pid.setpoint = nxt_wp_alt
    pid.tunings = (0.8, 0.6, 0.4)
    req_thr = pid(curr_alt)
    # print(req_thr)


def vel_controller():
    lvel.header.stamp=rospy.Time.now()
    lvel.header.frame_id='global'
    lvel.header.seq=0
    lvel.twist.linear.x=25
    lvel.twist.linear.y=0
    lvel.twist.linear.z=0
    lvel.twist.Angular.z=0

def setCurrentWaypoint(wp_num):
    try:
        flightModeService = rospy.ServiceProxy('/mavros/mission/set_current', WaypointSetCurrent)
        isWPChanged = flightModeService(wp_seq=wp_num) #return true or false
        if isWPChanged.success:
            rospy.loginfo("Waypoint Changed")
    except rospy.ServiceException as e:
        print ("service success")



if __name__ == "__main__":
    rospy.init_node("new_node")
    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    mission_sub=rospy.Subscriber("/mavros/mission/waypoints",WaypointList,callback=wp_cb)
    global_pos_sub=rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback=pos_cb)
    local_pos_sub=rospy.Subscriber("/mavros/global_position/local",Odometry, callback=loc_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)
    local_vel_pub = rospy.Publisher("mavros/setpoint_attitude/cmd_vel",TwistStamped,queue_size=10)
    # global_pos_pub=rospy.Publisher("/mavros/setpoint_position/global",GeoPoseStamped,queue_size=10)
    global_pos_pub=rospy.Publisher("mavros/setpoint_raw/global",GlobalPositionTarget,queue_size=10)


    # arming service
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    #set_mode service
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    #set wp service
    set_wp_client = rospy.ServiceProxy("/mavros/mission/set_current", WaypointSetCurrent)

    # Setpoint publishing at 20Hz, minimum required is 2Hz in Offboard mode
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    mis_set_mode= SetModeRequest()
    mis_set_mode.custom_mode='AUTO.MISSION'

    land_set_mode=SetModeRequest()
    land_set_mode.custom_mode='AUTO.LAND'

    # next_wp=WaypointSetCurrent()
    # no=5
    # no=int(str(no))
    # next_wp.wp_seq=no

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    flag1=0
    flag2=0
    global nxt_wp_alt

    while(not rospy.is_shutdown()):
        # Arming
        if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle already armed")
            last_req = rospy.Time.now()

        # Takeoff in mission , will try to go to mission once only
        if(current_state.armed and flag1==0):
            flag1=1
            setCurrentWaypoint(2)
            set_mode_client.call(mis_set_mode)

        #checking for waypoint 2, if reached switch to offboard
        if(curr_wp_no==3 or curr_wp_no==4 or curr_wp_no==5 or curr_wp_no==6 or  curr_wp_no==7 ):
            # print(curr_wp_no)

            gpose.coordinate_frame=6
            gpose.latitude=lat_list[curr_wp_no+1]
            curr_wp_lat=lat_list[curr_wp_no+1]
            gpose.longitude=lon_list[curr_wp_no+1]
            curr_wp_lon=lon_list[curr_wp_no+1]
            gpose.altitude=alt_list[curr_wp_no+1]    # with respect to home
            nxt_wp_alt=alt_list[curr_wp_no+1]

            gpose.yaw=-heading+1.57                  # changing frame for heading as ROS uses ENU and PX4 in NED
            set_mode_client.call(offb_set_mode)
            gpose.header.stamp=rospy.Time.now()
            global_pos_pub.publish(gpose)
            setCurrentWaypoint(curr_wp_no+1)
            # calling attitude controller
            # att_controller()

            #calling velocity controller
            # vel_controller()

            distance_meas(lat_list[curr_wp_no+1],lon_list[curr_wp_no+1],curr_lat ,curr_lon)         #calculating acceptance radius for waypoint reached
            if(dist<5):
               curr_wp_no=curr_wp_no+1                                          # acceptance radius set to 5m for accepting a waypoint as reached


        #checking for wapoint 7 and changing back to mission mode
        if(curr_wp_no==8):
            set_mode_client.call(mis_set_mode)
