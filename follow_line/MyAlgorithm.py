from sensors import sensor
import numpy as np
import threading
import cv2

#Remapeo espacial
class MyAlgorithm():

    def __init__(self, sensor):
        self.sensor = sensor
        self.imageRight=None
        self.imageLeft=None
        self.lock = threading.Lock()
        self.desviacionx_anterior=0



    def execute(self):
        #GETTING THE IMAGES
        imageLeft = self.sensor.getImageLeft()
        imageRight = self.sensor.getImageRight()
	#usar cvtColor y inRange, bit_wise
        hsv_left=cv2.cvtColor(imageLeft,cv2.COLOR_RGB2HSV)
        hsv_right=cv2.cvtColor(imageRight,cv2.COLOR_RGB2HSV)
        hsv_min=np.array([0,235,20])
        hsv_max=np.array([3,255,221])
        #Escala de conversion
        #H=360(2pi)-->0-180
	    #S=0-1-->*255
        #V=0-1 decimal-->*255

        hsv_filterleft=cv2.inRange(hsv_left, hsv_min, hsv_max) 
        hsv_filterright=cv2.inRange(hsv_right, hsv_min, hsv_max) 
        MASK3_right=np.dstack((hsv_filterright,hsv_filterright,hsv_filterright))
        MASK3_left=np.dstack((hsv_filterleft,hsv_filterleft,hsv_filterleft))
        
        shape = imageRight.shape
        ancho=shape[1]
        posicionx_left_arriba = []
        posicionx_left = []
        for i in range(0,ancho-1):
            if ((hsv_filterleft[370,i-1]-hsv_filterleft[370,i])!=0):
                    posicionx_left.append(i)
            if ((hsv_filterleft[300,i-1]-hsv_filterleft[300,i])!=0):
                    posicionx_left_arriba.append(i)



        if(len(posicionx_left)>1):
            posicionx_left[1]=posicionx_left[1]-1
        elif(len(posicionx_left)==1):
            if(posicionx_left[0]>=326/2):
                posicionx_left.append(326)
            else:
                posicionx_left.append(0)
        else:
            posicionx_left.append(0)
            posicionx_left.append(0)

        if(len(posicionx_left_arriba)>1):
            posicionx_left_arriba[1]=posicionx_left_arriba[1]-1
        elif(len(posicionx_left)==1):
            if(posicionx_left_arriba[0]>=326/2):
                posicionx_left_arriba.append(326)
            else:
                posicionx_left_arriba.append(0)
        else:
            posicionx_left_arriba.append(0)
            posicionx_left_arriba.append(0)

        
        
        xleft=(posicionx_left[0]+posicionx_left[1])/2
        xleft_arriba=(posicionx_left_arriba[0]+posicionx_left_arriba[1])/2
        
	#Centro de la linea en 326
        dif=xleft-xleft_arriba
        desviacionx=xleft-326


        # Add your code here
        #print "Runing"
       # print "dif" , dif
        if(abs(dif)<20):
            self.sensor.setW(-(0.008*desviacionx+0.0002*(desviacionx-self.desviacionx_anterior)))
        else:
            self.sensor.setW(-(0.003*desviacionx+0.00015*(desviacionx-self.desviacionx_anterior)))
        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
        if ((abs(desviacionx))<10):
            self.sensor.setV(6)
        elif((abs(desviacionx))<85):
            self.sensor.setV(3.5)
        else:
            self.sensor.setV(2)
        
        self.desviacionx_anterior=desviacionx
        
        #SHOW THE FILTERED IMAGE ON THE GUI


        self.setRightImageFiltered(MASK3_right)
        self.setLeftImageFiltered(MASK3_left)
      



    def setRightImageFiltered(self, image):
        self.lock.acquire()
        self.imageRight=image
        self.lock.release()


    def setLeftImageFiltered(self, image):
        self.lock.acquire()
        self.imageLeft=image
        self.lock.release()

    def getRightImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageRight
        self.lock.release()
        return tempImage

    def getLeftImageFiltered(self):
        self.lock.acquire()
        tempImage=self.imageLeft
        self.lock.release()
        return tempImage

