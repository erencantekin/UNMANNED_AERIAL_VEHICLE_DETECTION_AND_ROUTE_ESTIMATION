import cv2 as cv
import numpy as np
from scipy.ndimage import gaussian_filter
from Advanced_UAV_Detection import UAVDetector
from kalman.kalmanfilter import KalmanFilter
import threading
import asyncio
import math



def euclidean_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)
    
def calculate_accuracy_percentage(actual_coords, predicted_coords):
    if len(actual_coords) == 0 or len(predicted_coords) == 0:
        return ""
       
    
    total_distance = 0
    for actual, predicted in zip(actual_coords, predicted_coords):
        total_distance += euclidean_distance(actual, predicted)
    
    avg_distance = total_distance / len(actual_coords)
    accuracy = 1 - (avg_distance / euclidean_distance((0, 0), (max(actual_coords, key=lambda x: x[0])[0], max(actual_coords, key=lambda x: x[1])[1])))
    accuracy_percentage = accuracy * 100
    return f"Prediction ACC Percentage: {accuracy_percentage:.2f}%"


#SPEED START
def speedFinder(coveredDistance, timeTaken):
    speed = coveredDistance / timeTaken
    return speed
   

def averageFinder(completeList, averageOfItems):
    lengthOfList = len(completeList)
    
    selectedItems = lengthOfList - averageOfItems
    
    selectedItemsList = completeList[selectedItems:]
    
    average = sum(selectedItemsList) / len(selectedItemsList)
    
    return average
#SPEED END

#DISTANCE CODES START
uav_width = 0


def face_data(image):
    global uav_width
    """
    This function Detect the face
    :param Takes image as argument.
    :returns uav_width in the pixels
    """

    #uav_width = 0
    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    faces = uav_detector.detectMultiScale(gray_image, 1.3, 5)
    for (x, y, h, w) in faces:
        cv.rectangle(image, (x, y), (x + w, y + h), WHITE, 1)
        uav_width = w

    return uav_width
    
    
    
def distance_finder(focal_length, real_uav_width, uav_width_in_frame):
    """
    Estimates the distance between object and camera
    focal_length_Finder
    Actual width of object
    width of object in the frame
    distance Estimated
    """
    distance = (real_uav_width * focal_length) / uav_width_in_frame
    return distance



def focal_length(measured_distance, real_width, refWidth):
    """
    This Function Calculate the Focal Length(distance between lens to CMOS sensor),
    : distance measured from object to the Camera 
    : Actual width of object, in real world 
    : object width in the frame /image in our case in the reference image
    """
    focal_length_value = (refWidth * measured_distance) / real_width
    return focal_length_value

# variables

#SPEED
initialTime = 0
initialDistance = 0
changeInTime = 0
changeInDistance = 0

listDistance = []
listSpeed = []

#SPEED


# distance from camera to object(face) measured
KNOWN_DISTANCE = 76.2  # centimeter
# width of face in the real world or Object Plane
KNOWN_WIDTH = 14.3  # centimeter
# Colors
WHITE = (255, 255, 255)
fonts = cv.FONT_HERSHEY_COMPLEX
uav_detector = cv.CascadeClassifier("cascade_uav_default.xml")
ref_const_uav_width = 1800
focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, ref_const_uav_width)


# DISTANCE CODES END

# Load detector
od = UAVDetector()

import time 
REDU = 8
def rgbh(xs,mask):
    def normhist(x): return x / np.sum(x)
    def h(rgb):
        return cv.calcHist([rgb], [0, 1, 2],mask, [256//REDU, 256//REDU, 256//REDU] , [0, 256] + [0, 256] + [0, 256])
    return normhist(sum(map(h, xs)))

def smooth(s,x):
    return gaussian_filter(x,s,mode='constant')



backgroundSubtractor = cv.createBackgroundSubtractorMOG2(500, 60, True)
cap = cv.VideoCapture("Videos/input1.mp4")
key = 0

kernel = np.ones((3,3),np.uint8)
getDrone = False
detectionContinue = False
termination = (cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT, 10, 1)
font = cv.FONT_HERSHEY_SIMPLEX
pause= False

pause= False


#kalman tespit

degree = np.pi/180


fps = 120
dt = 1/fps
noise = 3

A = np.array(                       # state tansition matrix
    [1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1 ]).reshape(4,4)


u = np.array([0, 5])                # control input
B = np.array(                       # Control input matrix
    [dt**2/2, 0,
    0, dt**2/2,
    dt, 0,
    0, dt ]).reshape(4,2)

H = np.array(                       # Observation matrix
    [1,0,0,0,
    0,1,0,0]).reshape(2,4)

# x position, y position, X velocity, Y velocity
mu = np.array([0,0,0,0])            # state estimate

P = np.diag([10,10,10,10])**2
res=[]
N = 15
sigmaM = 0.0001     # process noise convariance
sigmaZ = 3*noise    # measurement noise convariance

#Q = sigmaM**2 * np.eye(4)
#R = sigmaZ**2 * np.eye(2)
listCenterX=[]
listCenterY=[]

kf = KalmanFilter()

add_count = 0
mm=False



frame2 = None


SPEED_VAL = ""
DISTANCE_VAL = ""
def calcSpeedDistance():
    global initialDistance
    global frame2
    global initialTime
    global SPEED_VAL
    global DISTANCE_VAL
    
    # calling face_data function
    uav_width_in_frame = face_data(frame2)
    # finding the distance by calling function Distance
    
    if uav_width_in_frame != 0:
       
        Distance = distance_finder(focal_length_found, KNOWN_WIDTH, uav_width_in_frame)
        listDistance.append(Distance)
        averageDistance = averageFinder(listDistance, 2)
        distanceInMeters = averageDistance/100
        
        if initialDistance != 0:
            changeInDistance = initialDistance - distanceInMeters
            
            changeInTime = time.time() - initialTime
            
            speed = speedFinder(coveredDistance=changeInDistance, timeTaken=changeInTime)
            listSpeed.append(speed)
            averageSpeed = averageFinder(listSpeed, 10)
            if averageSpeed < 0:
                averageSpeed = averageSpeed * - 1
            
            speedFill = int(45+(averageSpeed) * 130)
            if speedFill > 235:
                speedFill = 235
            
            SPEED_VAL = (f"Speed: {round(averageSpeed, 2)} m/s")
            
        initialDistance = distanceInMeters
        initialTime = time.time()
        
        DISTANCE_VAL = (f"Distance {round(distanceInMeters, 2)} m")
            

colorMask = None
his = [1, 1, 1]
rb = None
OProiBox = None
x_ort = 0
y_ort = 0
x_width = 0
y_height = 0
frame = None
backgroundSubtractorS = None
predictionList = []
actualList = []
predicted = (0, 0)
x_uncertainty = [0]
y_uncertainty = [0]
x_estimated = [0]
y_estimated = [0]
x_pred = [0]
y_pred = [0]
x_pred_uncertainty = [0]
y_pred_uncertainty = [0]
async def beginOperation():
    global frame2
    global getDrone
    global initialTime
    global initialDistance
    global changeInTime
    global changeInDistance

    global listDistance
    global listSpeed

    global KNOWN_DISTANCE
    global KNOWN_WIDTH


    global WHITE
    global fonts
    global uav_detector
    global ref_const_uav_width
    global focal_length_found
    global backgroundSubtractor
    global cap
    global key

    global kernel
    global getDrone
    global detectionContinue 
    global termination
    global font
    global pause


    global degree
    global fps
    global dt
    global noise
    global A
    global u
    global B
    global H
    global mu
    global P
    global res
    global N
    global sigmaM
    global sigmaZ
    global Q
    global R
    global listCenterX
    global listCenterY
    global kf
    global add_count
    global mm

    global SPEED_VAL
    global DISTANCE_VAL
    global colorMask
    global his
    global rb
    global OProiBox
    global x_ort
    global y_ort
    global x_width
    global y_height
    global frame
    global backgroundSubtractorS
    global predictionList
    global actualList
    global predicted
    global x_uncertainty
    global y_uncertainty
    global x_estimated
    global y_estimated
    global x_pred
    global y_pred
    global x_pred_uncertainty
    global y_pred_uncertainty
    

    while(True):
        key = cv.waitKey(30) & 0xFF
        if key== ord("p"): getDrone = True
        
        ret, frame = cap.read()
        frame2 = frame
        if ret == False:
            break
        
        frame=cv.resize(frame,(1366,768))
        backgroundSubtractorS = backgroundSubtractor.apply(frame)
        backgroundSubtractorS = cv.erode(backgroundSubtractorS,kernel,iterations = 1)
        backgroundSubtractorS = cv.medianBlur(backgroundSubtractorS,3)
        backgroundSubtractorS = cv.dilate(backgroundSubtractorS,kernel,iterations=2)
        backgroundSubtractorS = (backgroundSubtractorS > 200).astype(np.uint8)*255
        colorMask = cv.bitwise_and(frame,frame,mask = backgroundSubtractorS)
        
        x_ort = 0
        y_ort = 0
        x_width = 0
        y_height = 0
        if(getDrone):
            fromCenter= False
            img = colorMask
            r = od.detect(frame) 
            actualList.append(r[4])

            imCrop = img[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
            #print(int(r[0]),int(r[1]),int(r[2]),int(r[3]),"\n\n")
            x_ort = (r[0] + r[2])/2
            y_ort = (r[1] + r[3])/2
            x_width = abs(r[1] - r[3])
            y_height = abs(r[0] - r[2])
            detectionContinue = True
            
            try: # if value is null imCrop is null
            
                imCropMask = cv.cvtColor(imCrop, cv.COLOR_BGR2GRAY)
            except Exception as e:
                continue
    
            
            ret,imCropMask = cv.threshold(imCropMask,30,255,cv.THRESH_BINARY)
            his = smooth(1,rgbh([imCrop],imCropMask))
            OProiBox = (int(r[0]), int(r[1]),int(r[2]), int(r[3]))
    
    
        if(detectionContinue):
       
       
            
            thread = threading.Thread(target=calcSpeedDistance, args = ())
            thread.start()
            
            cv.line(frame, (45,70), (235, 70), (0,0,0),22)
            cv.putText(frame, SPEED_VAL, (50,75), fonts, 0.6, (0,255,220),2)
            
            cv.line(frame, (45, 25), (255, 25), (0, 0, 0), 22)
            cv.putText(frame, DISTANCE_VAL, (50,30), fonts, 0.6, WHITE, 2)
            
           
    
            
            
            
            
            thread2 = threading.Thread(target=track_drone)
            thread2.start()


            cv.ellipse(frame, rb, (0, 255, 0), 2)
            cv.circle(frame, (predicted[0], predicted[1]), 10, (255, 0, 255),3)
            
            for n in range(len(listCenterX)):
                cv.circle(frame,(int(listCenterX[n]),int(listCenterY[n])),3,(0, 255, 0),-1)
            for n in [-1]:
                incertidumbre=(x_uncertainty[n]+y_uncertainty[n])/2
                cv.circle(frame,(int(x_estimated[n]),int(y_estimated[n])),5,(255, 255, 0),3)
            for n in range(len(x_pred)):
                incertidumbreP=(x_pred_uncertainty[n]+y_pred_uncertainty[n])/2
                cv.circle(frame,(int(x_pred[n]),int(y_pred[n])),int(incertidumbreP),(0, 0, 255))
            
            cv.line(frame, (45, 114), (420, 114), (0, 0, 0), 22)
            cv.putText(frame, calculate_accuracy_percentage(actualList[1:], predictionList[1:]), (50,120), fonts, 0.6, WHITE, 2)
                            # first element in predict 0, first element in actual is its own current location
            
        cv.imshow('Frame', frame)
        
    cv.destroyAllWindows()



def track_drone():
    global add_count, listCenterX, listCenterY, mm, res, mu, P, frame2, frame, predictionList
    global colorMask
    global his
    global rb
    global OProiBox
    global x_ort
    global y_ort
    global x_width
    global y_height
    global frame
    global backgroundSubtractorS
    global predictionList
    global actualList
    global predicted
    global xu
    global yu
    global x_estimated
    global y_estimated
    global x_pred
    global y_pred
    global x_pred_uncertainty
    global y_pred_uncertainty
    add_count += 1
    rgbr = np.floor_divide( colorMask , REDU)
    r,g,b = rgbr.transpose(2,0,1)
    l = his[r,g,b]
    maxl = l.max()
    

    aa=np.clip((1*l/maxl*255),0,255).astype(np.uint8)       # normalization keeps it [0,255]

    (rb, OProiBox) = cv.CamShift(l, OProiBox, termination)
    rb = ((x_ort, y_ort),(x_width, y_height), rb[2])        # green line detection
    
    xOrt_coord=x_ort 
    yOrt_coord=y_ort 
    error=(OProiBox[3])
    if(yOrt_coord<error or backgroundSubtractorS.sum()<50 ):
        predicted, mu, statePost, errorCovPre = kf.predict(int(xOrt_coord), int(yOrt_coord))
        #print("Predicted : " + str(predicted))
        predictionList.append(predicted)

        mu,P = kf.kal(mu,P,B,u,z=None) # update itself for next predictions
        m="None"
        mm=False
    else:
        predicted, mu, statePost, errorCovPre = kf.predict(int(xOrt_coord), int(yOrt_coord))
        #print("Predicted : " + str(predicted))
        predictionList.append(predicted)

        mu,P = kf.kal(mu,P,B,u,z=np.array([xOrt_coord,yOrt_coord])) # update itself for next predictions
        m="normal"
    mm=True
    if(mm):
        listCenterX.append(xOrt_coord)
        listCenterY.append(yOrt_coord)
    
    if len(listCenterX) > 2:
        res += [(mu,P)]
        #cv.circle(frame, (predicted[0], predicted[1]), 10, (255, 0, 255),3)
        
        
        # prediction baslat
        mu2 = mu
        P2 = P
        res2 = []
    
        for _ in range(fps*2):
            mu2,P2 = kf.kal(mu2,P2,B,u,z=None) # update itself for next predictions
            res2 += [(mu2,P2)]
    
        
        x_estimated = [mu[0] for mu,_ in res]            # estimated position
        x_uncertainty = [2*np.sqrt(P[0,0]) for _,P in res]         # uncertainty ( standard sapma)
                                                        # in position ( confidence )
        y_estimated = [mu[1] for mu,_ in res]            # estimated position
        y_uncertainty = [2*np.sqrt(P[1,1]) for _,P in res]         # uncertainty in position ( confidence )
        
        x_pred=[mu2[0] for mu2,_ in res2]          # predicted position
        y_pred=[mu2[1] for mu2,_ in res2]          # predicted position
    
        x_pred_uncertainty = [np.sqrt(P[0,0]) for _,P in res2]     #uncertainty in predicted position
        y_pred_uncertainty = [np.sqrt(P[1,1]) for _,P in res2]     #uncertainty in predicted position
                                        # how much it can vary to process noise alone
        
    
        if(len(listCenterY)>40):
            listCenterY=[]
            listCenterX=[]
            res=[]
    
            mu = np.array([0,0,0,0])
            P = np.diag([100,100,100,100])**2
    
    
                          

def main():
    asyncio.run(beginOperation())

main()
