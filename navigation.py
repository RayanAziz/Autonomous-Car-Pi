#!/usr/bin/env python

import rospy
import math
import serial
import time
import IMU
import datetime
import os
from sensor_msgs.msg import NavSatFix, Imu, MagneticField
from std_msgs.msg import String
import socket

lidarReading = ""
cvReading = ""
port = "/dev/serial0"

destCoordinates_list = [(21.4984701, 39.2489091), 
                        (21.4984339, 39.248661),
                        (21.4983865, 39.2483566),
                        (21.4983491, 39.2481098)]


destCoordinates_list = [(21.4988116, 39.2486381)]

destCoordinates_list = [(21.4991822, 39.2485657)]


destCoordinates_list = [(21.4990084, 39.2486005), 
                        (21.4990496, 39.2489103)]



destCoordinate = destCoordinates_list[0] # init to first point
currentCoordinate = [0, 0]


cRadius = 5 # radius of the circle


IMU_UPSIDE_DOWN = 0
M_PI = 3.14159265358979323846

# Compass calibration
magXmin =  -35
magYmin =  -1156
magZmin =  -745
magXmax =  2473
magYmax =  1252
magZmax =  1399


# This function return the distance in meters between two coordinates.
def getDistance(p1, p2):
    earthR = 6371 # earth radius (km)

    lat1, lon1 = p1
    lat2, lon2 = p2

    dLat = math.radians(lat2-lat1)
    dLon = math.radians(lon2-lon1)

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)

    a = math.sin(dLat/2) ** 2 + math.sin(dLon/2) ** 2 * math.cos(lat1) * math.cos(lat2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    return int(earthR * c * 1000)
    

def isInsideCircle(destPoint):
    global currentCoordinate
    dist = getDistance(currentCoordinate, destPoint)
    result = dist <= cRadius

    #print("The distance differance is: %d - Inside? %s" %(dist, result))
    return result


def calc_IMU_Heading():
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()
    
    #Apply compass calibration    
    MAGx -= (magXmin + magXmax) /2 
    MAGy -= (magYmin + magYmax) /2 
    MAGz -= (magZmin + magZmax) /2 
    
    
    if IMU_UPSIDE_DOWN:
        MAGy = -MAGy      #If IMU is upside down, this is needed to get correct heading.

    
    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    #Normalize accelerometer raw values.
    if not IMU_UPSIDE_DOWN:        
        #Use these two lines when the IMU is up the right way. Skull logo is facing down
        accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    else:
        #Us these four lines when the IMU is upside down. Skull logo is facing up
        accXnorm = -ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
        accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

    #Calculate pitch and roll

    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))


    #Calculate the new tilt compensated values
    magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
 
    #The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
    #is also reversed. This needs to be taken into consideration when performing the calculations
    if(IMU.LSM9DS0):
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS0
    else:
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS1

    #Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
                tiltCompensatedHeading += 360

    ############################ END ##################################
    
    return(int(tiltCompensatedHeading))

def parseGPS(data):
    global currentCoordinate
    
    #print ("raw:", str(data[0:6])) #prints raw data
    if data[0:6].decode("utf-8") == "$GNRMC" :
        sdata = data.decode("utf-8").split(",")
        if sdata[2] == 'V':
            print ("no satellite data available")
            return
        #print("---Parsing GPRMC---")
        time = sdata[1][0:2] + ":" + sdata[1][2:4] + ":" + sdata[1][4:6]
        lat = decode(sdata[3])  #latitude
        latH = sdata[3]         #latitude
        dirLat = sdata[4]       #latitude direction N/S
        lon = decode(sdata[5])  #longitute
        lonH = sdata[5]         #longitute
        dirLon = sdata[6]       #longitude direction E/W
        speed = sdata[7]        #Speed in knots
        trCourse = sdata[8]     #True course
        date = sdata[9][0:2] + "/" + sdata[9][2:4] + "/" + sdata[9][4:6] #date
        
        #print ("Start (GPS)         - latitude : %s(%s), longitude : %s(%s)" %(lat,dirLat,lon,dirLon,))
        #print ("Destination (Fixed) - latitude : 21.495202(N), longitude : 39.244551(E)")
        
        currentCoordinate[0] = float(lat)
        currentCoordinate[1] = float(lon)
        
        return(int(heading(latH, dirLat, lonH, dirLon, destCoordinate[0],  destCoordinate[1])))

def heading(startLat, latDir, startLon, lonDir, destLat, destLon):
    aLat = startLat.split(".")
    headLat = aLat[0]
    tailLat = aLat[1]
    minn = headLat[-2:]
    minutesLat = minn + tailLat 
    
    aLon = startLon.split(".")
    headLon = aLon[0]
    tailLon = aLon[1]   
    minn = headLon[-2:]
    minutesLon = minn + tailLon
    
    sLat = math.radians(float(headLat[0:-2] + "." + str(int(minutesLat) // 60))) # Start Lat - Radians
    sLon = math.radians(float(headLon[0:-2] + "." + str(int(minutesLon) // 60))) # Start Lon - Radians
    
    if latDir == "S": sLat = -sLat
    if lonDir == "W": sLon = -sLon
    
    dsLat = math.radians(destLat)
    dsLon = math.radians(destLon)

    dLon = dsLon - sLon
    y = math.cos(dsLat) * math.sin(dLon)
    x = math.cos(sLat) * math.sin(dsLat) - math.sin(sLat) * math.cos(dsLat) * math.cos(dLon)
    brng = math.atan2(y, x)
    brng = math.degrees(brng)
    
    if brng < 0: brng+= 360
    
    #print("Bearing = " + str(brng))
    return(brng)
  
def decode(coord):
    #Converts DDDMM.MMMMM > DD deg MM.MMMMM min
    x = coord.split(".")
    head = x[0]
    deg = head[0:-2]
    d_min = head[-2:]
    tail = x[1] 
    minutes = d_min + tail 
    
    return deg + "." + str(int(minutes) // 60)

def callback1(data):
    global lidarReading
    if 'stop' in str(data):
        lidarReading = "Stop"
    elif 'go' in str(data):
        lidarReading = "Go"
        
def callback2(data):
    global cvReading

    if '"0"' in str(data):
        cvReading = "0"
    elif '"120"' in str(data):
        cvReading = "120"
    elif '"80"' in str(data):
        cvReading = "80"
    elif '"40"' in str(data):
        cvReading = "40"
    else:
        cvReading = "40"

def navControl():
    global destCoordinate
    
    # Node Initilization
    rospy.init_node('motorCMD', anonymous=True)
    
    # Subscribers
    # IMU and GPS topics
    rospy.Subscriber('/scan', String, callback1)  # LIDAR topic
    rospy.Subscriber('/speed', String, callback2) # CV Module topic

    # Publisher
    pub = rospy.Publisher('motorCommands', String, queue_size=10)
    rate = rospy.Rate(5) # 5 Hz
    
    # Init motorController vars
    var1 = b'\x24'
    var2 = b'\x02'
    
    lm_speed = b'\x00'
    lm_dir = b'\x02' # Fixed to forward
    rm_speed = b'\x00'
    rm_dir = b'\x02' # Fixed to forward
    s_speed = b'\x00' 
    s_dir = b'\x00'
    
    dest_ptr = 0 # list index
    
    host = "192.168.4.1"
    port = 8000
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    
    
    while not rospy.is_shutdown():
        
        imuHeading = calc_IMU_Heading() # calculate IMU heading (compass)
        
        # Read GPS heading from GPS Module, needs clear access to sky (outdoor)
        data = ser.readline()
        
        if '$GNRMC' not in data:
            continue
        
        gpsHeading = parseGPS(data)

        #print(gpsHeading)
        #gpsHeading = 90 # Testing: east
                
        directionCommand = speedCommand = ""
        command = ""

        if ((gpsHeading - imuHeading + 360) % 360 < 180):
            # CW
            s_dir = b'\x03' # CW 
            angle = (gpsHeading - imuHeading + 360) % 360
            #speed = (((angle - 0) * (255 - 70)) / (179 - 0)) + 70
            speed = 75 + angle * 2
            if speed > 255: speed = 255
            if angle < 5: speed = 0
            s_speed = b''.join(bytes(hex(int(speed))[2:].zfill(2).decode("hex")))
        else:
            # CCW
            s_dir = '\x02' # CCW
            angle = (imuHeading - gpsHeading + 360) % 360
            #speed = (((angle - 0) * (255 - 70)) / (180 - 0)) + 70
            speed = 75 + angle * 2
            if speed > 255: speed = 255
            if angle < 5: speed = 0
            s_speed = b''.join(bytes(hex(int(speed))[2:].zfill(2).decode("hex")))


        #lm_speed = b'\x7F'
        #lm_dir = b'\x02'
        #rm_speed = b'\x7F'
        #rm_dir = b'\x02'

        if (lidarReading == "Stop" or cvReading == "0"):
            speedCommand = "0"
            lm_speed = b'\x00'
            rm_speed = b'\x00'
        else:
            speedCommand = cvReading
            if (cvReading == "120"):
                lm_speed = b'\xff'
                rm_speed = b'\xff'
            elif (cvReading == "80"):
                lm_speed = b'\x80'
                rm_speed = b'\x80'
            elif (cvReading == "40"):
                lm_speed = b'\x60'
                rm_speed = b'\x60'
        
        if isInsideCircle(destCoordinates_list[dest_ptr]):
            if dest_ptr < len(destCoordinates_list) - 1:
                print("\nCoordinates #%d Reached\n" %(dest_ptr+1))
                dest_ptr = dest_ptr + 1
                destCoordinate[0] = destCoordinates_list[dest_ptr][0]
                destCoordinate[1] = destCoordinates_list[dest_ptr][1]
            else:
                print("\nFinal Destination Reached !")
                # Send STOP signals for speed motors & dir motor
                lm_speed = b'\x00'
                rm_speed = b'\x00'
                s_dir = b'\x00'
                break
        
        #lm_speed = b'\xE7'
        #lm_dir = b'\x02'
        #rm_speed = b'\xE7'
        #rm_dir = b'\x02'
        
        command = directionCommand + " - " + speedCommand
        commandTCP = b''.join([var1, var2, rm_speed, rm_dir, lm_speed, lm_dir, s_speed, s_dir])
        s.send(commandTCP)
        
        #print("GPS Heading: %d" % int(gpsHeading))
        #print("IMU Heading: %d" % int(imuHeading))
        #print("\t-> Direction  : %s" % directionCommand)
        #print("\t-> Speed      : %s\n" % speedCommand)
        pub.publish(command)
        
        rate.sleep()
    
if __name__ == '__main__':
    # Init IMU & GPS
    IMU.detectIMU() #Detect if BerryIMUv1 or BerryIMUv2 is connected.
    IMU.initIMU()   #Initialise the accelerometer, gyroscope and compass

    ser = serial.Serial(port, baudrate = 9600, timeout = 0.5) # GPS serial port

    navControl()
