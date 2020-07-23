#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from std_msgs.msg import String

# Global Variables
accX = accY = accZ = 0.0
magX = magY = magZ = 0.0
gpsHeading = imuHeading = 0
lidarReading = ""
cvReading = ""

destCoordinate = (21.496168, 39.245682) # Destination example

def callback1(data):
    global gpsHeading

    current_gpsLat = math.radians(data.latitude)
    current_gpsLon = math.radians(data.longitude)
    dest_gpsLat, dest_gpsLon = destCoordinate

    dest_gpsLat = math.radians(dest_gpsLat)
    dest_gpsLon = math.radians(dest_gpsLon)
    
    deltaLon = dest_gpsLon - current_gpsLon
    y = math.cos(dest_gpsLat) * math.sin(deltaLon)
    x = math.cos(current_gpsLat) * math.sin(dest_gpsLat) - math.sin(current_gpsLat) * math.cos(dest_gpsLat) * math.cos(deltaLon)
    gpsHeading = math.atan2(y, x)
    gpsHeading = math.degrees(gpsHeading)
    
    if gpsHeading < 0: gpsHeading+= 360

def callback2(data):
    global accX
    global accY
    global accZ

    accX = data.linear_acceleration.x
    accY = data.linear_acceleration.y
    accZ = data.linear_acceleration.z

def callback3(data):
    global magX
    global magY
    global magZ

    global imuHeading

    magX = data.magnetic_field.x
    magY = data.magnetic_field.y
    magZ = data.magnetic_field.z
    
    accXnorm = accX / math.sqrt(accX * accX + accY * accY + accZ * accZ)
    accYnorm = accY / math.sqrt(accX * accX + accY * accY + accZ * accZ)

    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    magXcomp = magX * math.cos(pitch) + magZ * math.sin(pitch)
    magYcomp = magX * math.sin(roll) * math.sin(pitch) + magY * math.cos(roll) + magZ * math.sin(roll) * math.cos(pitch)

    imuHeading = 180 * math.atan2(magYcomp,magXcomp) / math.pi
    imuHeading -= 90 # offset
    if imuHeading < 0: imuHeading += 360

def callback4(data):
    global lidarReading
    if 'stop' in str(data):
        lidarReading = "Stop"
    elif 'go' in str(data):
        lidarReading = "Go"
        
def callback5(data):
    global cvReading

    if '"0"' in str(data):
        cvReading = "0"
    elif '"120"' in str(data):
        cvReading = "120"
    elif '"80"' in str(data):
        cvReading = "80"
    elif '"40"' in str(data):
        cvReading = "40"

def navControl():
    # Node Initilization
    rospy.init_node('motorCMD', anonymous=True)
    
    # Subscribers
    # IMU and GPS topics
    rospy.Subscriber('/android/gps', NavSatFix, callback1) 
    rospy.Subscriber('/android/imu', Imu, callback2)
    rospy.Subscriber('/android/mag', MagneticField, callback3)
    rospy.Subscriber('/scan', String, callback4)  # LIDAR topic
    rospy.Subscriber('/speed', String, callback5) # CV Module topic

    # Publisher
    pub = rospy.Publisher('motorCommands', String, queue_size=10)
    rate = rospy.Rate(2) # 2 Hz

    while not rospy.is_shutdown():
        theta = abs(int(imuHeading) - int(gpsHeading)) 
        directionCommand = speedCommand = ""
        command = ""
        
        if imuHeading > gpsHeading and theta < 180:
            directionCommand = "CCW"
        elif imuHeading > gpsHeading and theta >= 180:
            directionCommand = "CW"
        elif imuHeading < gpsHeading and theta < 180:
            directionCommand = "CW"
        elif imuHeading < gpsHeading and theta >= 180:
            directionCommand = "CCW"

        if (lidarReading == "Stop" or cvReading == "0"):
            speedCommand = "0"
        else:
            speedCommand = cvReading

        command = directionCommand + " - " + speedCommand
        
        print("GPS Heading: %d" % int(gpsHeading))
        print("IMU Heading: %d" % int(imuHeading))
        print("Direction  : %s" % directionCommand)
        print("Speed      : %s\n" % speedCommand)
        pub.publish(command)
        rate.sleep()
    
if __name__ == '__main__':
    navControl()

