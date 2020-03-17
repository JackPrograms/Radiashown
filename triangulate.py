#!python2
# An example script to connect to Google using socket 
# programming in Python 
import socket # for socket 
import sys
import struct
import math

radAssistPort = 22001

global checkCps;
checkCps = False;
currentX = 10
currentY = 2 
currentAngle = 1.57 # orientation of the robot relative to y axis (radians)


sampleList = []
numSamples = 0 #total number of samples
radiationMap = {}

#used to dislay the radiation icon on screen
maxReading = 0
maxCoordinates = []

class Sample:
    def __init__(self, x, y, angle, az, el, avgCps):
        self.x = x
        self.y = y
        self.angle = angle
        self.az = az
        self.el = el
        self.avgCps = avgCps
        
        self.theta = self.angle + self.az
        self.slope = math.tan(self.theta)

        global maxReading
        global maxCoordinates

        #compare to maxReading 
        if self.avgCps > maxReading:
            maxReading = self.avgCps
            maxCoordinates = [self.x, self.y]

def getRadAssistData(port, x, y, angle, radAssistSocket):
    global numSamples

    az = []
    el = []
    detCps1 = []
    detCps2 = []
    detCps3 = []
    detCps4 = []
    detCps5 = []
    detCps6 = []
    detCps7 = []
    detCps8 = []
    thisCps = []
    
    totalCps = 0
    byteCount = 1
    detCps = [None] * 8


    detectors = [detCps1, detCps2, detCps3, detCps4, detCps5, detCps6, detCps7, detCps8]

    while True:
        # Store relevant bytes as an array of integers 
        if byteCount < 138:
            msg = radAssistSocket.recv(1)
            if 38 <= byteCount <= 45: # Store the 8 bytes of the double
                az.append(ord(msg))
            elif 46 <= byteCount <= 53:
                el.append(ord(msg))
            elif 54 <= byteCount <= 61:
                detCps1.append(ord(msg))
            elif 62 <= byteCount <= 69:
                detCps2.append(ord(msg))
            elif 70 <= byteCount <= 77:
                detCps3.append(ord(msg))
            elif 78 <= byteCount <= 85:
                detCps4.append(ord(msg))
            elif 86 <= byteCount <= 93:
                detCps5.append(ord(msg))
            elif 94 <= byteCount <= 101:
                detCps6.append(ord(msg))
            elif 102 <= byteCount <= 109:
                detCps7.append(ord(msg))
            elif 110 <= byteCount <= 117:
                detCps8.append(ord(msg))
                       
            byteCount += 1 
        else:
            # One full msg (137 bytes) has been recieved
            azUni = struct.pack('BBBBBBBB', *az)
            azimuth = struct.unpack('<d', azUni)[0]
            #azimuth = math.degrees(azimuth)
        
            elUni = struct.pack('BBBBBBBB', *el)
            elevation = struct.unpack('<d', elUni)[0]
            #elevation = math.degrees(elevation)

            #print "Azimuth = %f\n" % azimuth
            #print "Elevation = %f\n" % elevation

            detectors = [detCps1, detCps2, detCps3, detCps4, detCps5, detCps6, detCps7, detCps8]

            for d in range(0,8):
                thisCps = detectors[d]
                thisCpsUni = struct.pack('BBBBBBBB', *thisCps)
                detCps[d] = struct.unpack('<d', thisCpsUni)[0]
                #print "Detector %d Cps = %f\n" % (d+1, detCps[d])

            #calculate average cps value
            for i in range(0,8):
                totalCps += detCps[i]
            avgCps = totalCps/8
                
            #create an instance of Sample class
            sampleList.append(Sample(x, y, angle, azimuth, elevation, avgCps))
            
            #print "Azimuth = %f \n" % sampleList[numSamples].az
            numSamples += 1
            #radiation check completed, reset flag and byteCount
            byteCount = 1
            checkCps = False
            break

def triangulate(s1, s2):
    if s1.slope == s2.slope:
        print "No intersection"
        estimateX = None
        estimateY = None
    else:
        estimateX = ((s2.slope * s2.x) - (s1.slope * s1.x) + s1.y - s2.y) / (s2.slope - s1.slope)
        estimateY = s1.slope * (estimateX - s1.x) + s1.y
        print "Coordinates = %f, %f" % (estimateX, estimateY)

    return estimateX, estimateY


def main():
    global currentX
    tCount = 1
    
    try:
        # Create client socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        print "socket successfully created"
    except socket.error as err: 
        print "socket creation failed with error %s" %(err)
      
    port = 22001
      
    try: 
        host_ip = '192.168.1.128'
    except socket.gaierror: 
        print "there was an error resolving the host"
        sys.exit() 
      
    # Connect to the server 
    s.connect((host_ip, port)) 

    print "the socket has successfully connected"

    while True:
        checkCps = input("Take Reading?")
        if checkCps:
            currentX += 5
            getRadAssistData(radAssistPort, currentX, currentY, currentAngle, s)
        while tCount < numSamples:
            for i in range(0,tCount):
                print "Triangulate %d & %d\n" % (i, tCount)
                triangulate(sampleList[i],sampleList[tCount])
            tCount += 1
            
	
if __name__ == '__main__':
    main()
    
