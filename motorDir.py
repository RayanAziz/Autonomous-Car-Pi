#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from std_msgs.msg import String

# Global Variables
accX = accY = accZ = 0.0
magX = magY = magZ = 0.0
gpsHeading = imuHeading = 0

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
    accYnorm = accY / math.sqrt(accX ** 2 + accY ** 2 + accZ ** 2)

    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    magXcomp = magX * math.cos(pitch) + magZ * math.sin(pitch)
    magYcomp = magX * math.sin(roll) * math.sin(pitch) + magY * math.cos(roll) + magZ * math.sin(roll) * math.cos(pitch)

    imuHeading = 180 * math.atan2(magYcomp,magXcomp) / math.pi
    # imuHeading -= 90 # offset
    if imuHeading < 0: imuHeading += 360

def getDir():
    # Subscribers
    rospy.init_node('motorDir', anonymous=True)
    rospy.Subscriber('/android/gps', NavSatFix, callback1)
    rospy.Subscriber('/android/imu', Imu, callback2)
    rospy.Subscriber('/android/mag', MagneticField, callback3)

    # Publisher
    pub = rospy.Publisher('direction', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        
        theta = abs(int(imuHeading) - int(gpsHeading)) 
        motorDir = ""
    
        if imuHeading > gpsHeading and theta < 180:
            motorDir = "CCW - Left"
        elif imuHeading > gpsHeading and theta >= 180:
            motorDir = "CW - Right"
        elif imuHeading < gpsHeading and theta < 180:
            motorDir = "CW - Right"
        elif imuHeading < gpsHeading and theta >= 180:
            motorDir = "CCW - Left"

        print("GPS Heading: %d" % int(gpsHeading))
        print("IMU Heading: %d" % int(imuHeading))
        print("Direction  : %s\n" % motorDir)
        pub.publish(motorDir)
        rate.sleep()
    
if __name__ == '__main__':
    getDir()