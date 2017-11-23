# -*- coding: utf-8 -*-
"""
Created on Mon May 1 16:55:38 2017

@author: Holger
"""

'''todo:
   correct speed
   convert camera pixels to distance in um
   why more than 1 recording in txt file?
   include pause as response
   handle more than one worm
   track only worm no (far) bigger object <- machine learning?
   improve result windows maybe show plots from analysis
   store preset protocols
   check whether computer is fast enough
   check whether image capturing is fast enough
   check whether user input is doable
   check whether filenames exist and do not overwrite
   two colomns for metadata incubation - experiment
   fade out steps
   other layout of GUI
   guide of how to pour plates
   help file
   requirements
   implement on rPI
   control LEDs ?
   implement copper ring: scan ring and define as limit
'''
from sklearn.preprocessing import normalize
import skimage.morphology as skimor
import numpy as np
import cv2
import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import threading
from datetime import datetime
import getpass
import os
from time import time
import time
import scipy.stats as st
import sys
import serial
import USTrackAnalysis as UTA
if sys.version_info >= (3,0):
    import urllib.parse
sys.path.insert(0, 'C:/Standa controller')
try:
    from pyximc import *
except ImportError as err:    
#    print ("Can't import pyximc module. The most probable reason is that you haven't copied pyximc.py to the working directory. See developers' documentation for details.")
    exit()
except OSError as err:
#    print ("Can't load libximc library. Please add all shared libraries to the appropriate places (next to pyximc.py on Windows). It is decribed in detail in developers' documentation. On Linux make sure you installed libximc-dev package.")
    exit()
    
class USPokingTracker:
    def __init__(self):
        '''Initilizes the USPokingTracker class
           the most recently read frame, thread for reading frames, and
           the thread stop event
        '''
#        self.undefined = True        
        self.cap =cv2.VideoCapture(1)                                           # captures video from 2nd webcam
        self.frame = None                                                       # actual frame
        self.thread = None                                                      # thread for threading
        self.stopEvent = None                                                   # trigger to stop threading
        self.filedir = 'C:/Users/'+getpass.getuser()+'/Desktop/'+datetime.today().strftime("%Y%m%d")+'/' # directory to save all results      
        if not os.path.exists(self.filedir):
            os.makedirs(self.filedir)                                           # if directory does not exist, create it
        self.fourcc = cv2.VideoWriter_fourcc(*'DIB ')                           # codec to save video (DIB )
#-------the main GUI------------------------------------------------------------#
        self.root = tk.Tk()                                                     # starts the GUI

        screen_width = self.root.winfo_screenwidth()                            # the width of the screen
        screen_height = self.root.winfo_screenheight()                          # the height of the screen
        if screen_width >= 650 and screen_height >= 755:                        # defines the size of the GUI according to the screen geometry
            self.root.geometry("%dx%d+0+0" % (650, 755))
        else:
            self.root.geometry("%dx%d+0+0" % (screen_width, screen_height))
        self.root.resizable(0,0)
        self.videoDisplay = None
#-------parameters for stage-distance to pixel callibration and conversion------#
        self.ImagestoCalibrate = []                                             # (3) images are recorded for callibration
        self.stagePostoCalibrate = []                                           # (3) stage positions for callibration
        self.rotationMatrix = [[1,1],[1,1]]                                     # matrix to convert stage postions to pixels
        self.InverseRotationMatrix = [[1,1],[1,1]]                              # matrix to convert pixels to stage postions                
#-------parameter for US stimulus-----------------------------------------------#
        self.stateProtocol = False                                              # trigger to use a protocol for US stimulation        
        self.conn = serial.Serial('COM3', 9600)                                 # serial connection of the Arduino       
        self.conn.timeout = 5                                                   # timeout for the readline of the Arduino
        self.USPin = 13                                                         # pin 13 of the Arduino is used to send a 5 V signal to a frequency generator
        self.standardProtocol = [50,300,300,10,10,False]                         # Duty cycle (%) , pulse length (ms), pulse repitition frequency (1-100 Hz), 
                                                                                    # inter-stimulus interval (s), number of pulses per animal, pulse on (False,True)
        self.PulseTrainNumber = 0                                               # number of puls within one puls train
        self.PulseNumber = 0                                                    # to keep track of the number of pulses applied to one worm      
        self.timeOn = (1000/self.standardProtocol[2])*(self.standardProtocol[0]/100)
        self.timeOff = (1000/self.standardProtocol[2])*((100-self.standardProtocol[0])/100)
#-------parameters for the video and the image analysis-------------------------#
        self.numberofChecks = 7                                                 # number of checkpoints while recording (7)
        self.numofFramesperPuls = (self.numberofChecks * 10) + 1                # number of frames that are recorded per US puls (needs to be (n*7)+1)
        self.numofFrames = self.numofFramesperPuls * self.standardProtocol[4]   # number of frames recorded for one worm 
        self.USPulseFifth = self.numberofChecks - 3                             # after self.USPulseFifth / 7 of self.numofFramesperPuls-1 images the US pulse is initiated
        self.firstrecording = True                                              # first recording of this worm
        self.recording = False                                                  # the GUI is recording
        self.PositionWorm = []                                                  # the positions list of a worm
        self.wormVelocities = []                                                # the velocities list of a worm
        self.individualResult = []                                              # the individual results of a worm ()
        self.allResults = []                                                    # all results
        self.WormNumber = 0                                                     # the number of the actual worm
        self.frameNumber = 0                                                    # the number of the actual frame
        self.PosWormX, self.PosWormY = [],[]                                    # positions of the worm during one recording 
        self.PosStageX, self.PosStageY = [],[]                                  # positions of the stage during one recording (in case a video of the entire worm track is desired)
        self.times = []
        self.line = []                                                          # list of lines that represent the worm's traces
        self.recordingStopped = False
        self.lockonotherEndpushed = False
        self.UTA = UTA.TrackAnalysis()
#        self.timeActualFrame = 0                                                # the time of the actual frame
#-------initilizes the stage----------------------------------------------------#        
        devenum = lib.enumerate_devices(EnumerateFlags.ENUMERATE_PROBE, None)   # enumarates the stage devices
        self.device_id=[]
        self.device_id.append(lib.open_device(lib.get_device_name(devenum,0)))
        self.device_id.append(lib.open_device(lib.get_device_name(devenum,1)))
        self.stageisCalibrated = False                                          # indicates whether the stage was calibrated
#-------frames, buttons and canvas for the GUI using tkinter--------------------#
#-------define the overall structure--------------------------------------------#    
        self.tkcolor = 'midnight blue'   
        tkButtonColor = 'light goldenrod'
        self.topFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=40, pady=3)
        calibrationFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=40, pady=3)        
        stageFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=120, pady=3)
        usProtocolFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=40, pady=3) 
        wormorFlyFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=40, pady=3)            
        self.videoFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=480, pady=3, padx=3)
        lastFrame = tk.Frame(self.root, bg=self.tkcolor, width = 640, height=40, pady=3)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(0, weight=1)       
        self.topFrame.grid(row=0, sticky="ew")
        calibrationFrame.grid(row=1, sticky="ew")
        stageFrame.grid(row=2, sticky="ew")       
        usProtocolFrame.grid(row=3, sticky="ew")
        wormorFlyFrame.grid(row=4, sticky="ew")         
        self.videoFrame.grid(row=5, sticky="ew") 
        lastFrame.grid(row=6, sticky="ew") 
#-------define ingeridients and the structure of the top frame------------------#     
        buttonSaveDir = tk.Button(self.topFrame, bg=tkButtonColor,text="Choose Directory", command=self.fileSave) # button to change the directory to save files to      
        labelSaveDir = tk.Label(self.topFrame, bg=self.tkcolor,width=60, fg='ivory', text="Files are safed to: "+self.filedir) # shows the directory that all files are saved to in the GUI    
        buttonMetadata = tk.Button(self.topFrame, bg=tkButtonColor,text="Metadata", command=self.change_Metadata) # button to change the metadata information
        labelSaveDir.grid(row = 0, column = 0, padx=3)
        buttonSaveDir.grid(padx=10, row = 0, column = 1)         
        buttonMetadata.grid(padx=10, row = 0, column = 2)
#-------define ingeridients and the structure of the calibration frame----------#         
        buttonCalibrateStage = tk.Button(calibrationFrame, bg=tkButtonColor, width=45, text="Calibrate Stage", command=self.calibrateStage) # button to calibrate the stage      
        buttonCalibrationUsPuls = tk.Button(calibrationFrame, bg=tkButtonColor,width=45,text="US Pulse for transducer-camera alignment", command=self.calibrationUSPuls) # button will induce an ultrasound puls
        self.USAlignement=False
        buttonCalibrateStage.grid(row = 0, column = 1)
        buttonCalibrationUsPuls.grid(row = 0, column = 0)         
#-------define ingeridients and the structure of the stage frame----------------#         
        buttonMoveLeft = tk.Button(stageFrame, bg=tkButtonColor,width=10,text="Move left", command=self.manualMoveStage_left) # button to move the stage left
        buttonMoveRight = tk.Button(stageFrame, bg=tkButtonColor,width=10,text="Move right", command=self.manualMoveStage_right) # button to move the stage right
        buttonMoveUp = tk.Button(stageFrame, bg=tkButtonColor,width=10,text="Move up", command=self.manualMoveStage_up) # button to move the stage up
        buttonMoveDown = tk.Button(stageFrame, bg=tkButtonColor,width=10,text="Move down", command=self.manualMoveStage_down) # button to move the stage down
        buttonCenterStage = tk.Button(stageFrame, bg=tkButtonColor,width=10,text="Center Stage", command=self.centerStage) # button to center the stage        
        buttonMoveLeft.grid(row = 1, column = 2)         
        buttonMoveRight.grid(row = 1, column = 4)
        buttonMoveUp.grid(row = 0, column = 3)         
        buttonMoveDown.grid(row = 2, column = 3)          
        buttonCenterStage.grid(row = 1, column = 3)        
#-------define ingridients and structure of the ultrasound frame----------------#
        self.v = tk.IntVar()
        self.v.set(1)
        rButton10Touch = tk.Radiobutton(usProtocolFrame, bg=self.tkcolor, fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="10 touch assay", variable=self.v, value=1, command=self.tenTouchChosen) # radio button to choose the standard ten touch assay
        rButtonSelfProtocol = tk.Radiobutton(usProtocolFrame, bg=self.tkcolor, fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="Custom Protocol", variable=self.v, value=2, command=self.protocolChosen) # radio button to choose the self-defined protocol
        buttonChangeProtocol = self.changeProtocol = tk.Button(usProtocolFrame, bg=tkButtonColor, text="Custom Protocol", command=self.change_Protocol, state='disabled') # button to define a protocol
        rButton10Touch.invoke()
        buttonUsPuls = tk.Button(usProtocolFrame, bg=tkButtonColor,width=10,text="US Pulse", command=self.US_puls) # button will induce an ultrasound puls
        rButton10Touch.grid(row = 0, column = 0,padx = 10)
        rButtonSelfProtocol.grid(row = 0, column = 1,padx = 10)         
        buttonChangeProtocol.grid(row = 0, column = 2, padx = 10)
        buttonUsPuls.grid(row = 0, column = 3, padx = 40)
#-------define ingeridients and the structure of the worm or fly frame----------#     
        self.u = tk.BooleanVar() 
        self.u.set(True)
        rButtonWorm = tk.Radiobutton(wormorFlyFrame,bg=self.tkcolor, fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="Worm", variable=self.u, value=True, command=self.wormChosen) # radio button to choose worm
        rButtonFly = tk.Radiobutton(wormorFlyFrame, bg=self.tkcolor, fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="Fly", variable=self.u, value=False, command=self.flyChosen) # radio button to choose fly    
        self.matchWormFly = 1
        rButtonWorm.select()
        rButtonWorm.grid(row = 0, column = 0,padx = 10)
        rButtonFly.grid(row = 0, column = 1,padx = 10)           
#-------define ingridients and structure of the video frame---------------------#
        self.startWormTracking = tk.BooleanVar()                                # variable to sest the radio button below
        self.startWormTracking.set(False)
        self.tracktheWorm = False                                               # variable to start worm tracking
        rButtonStartTrack = tk.Radiobutton(self.videoFrame, bg=self.tkcolor, fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="Start tracking", variable=self.startWormTracking, value=True, command=self.startTracking) # radio button to chose to start tracking the worm 
        self.searchforHead = True
        self.ButtonIsHead = tk.Button(self.videoFrame, bg=tkButtonColor, text="Define as Head", command=self.lockonHead, state='disabled') # button to define the current end of the worm as head        
        self.ButtonOtherEnd = tk.Button(self.videoFrame, bg=tkButtonColor, text="Lock on other end", command=self.lockonotherEnd, state='disabled') # button to define the current end of the worm as head                
        rButtonStopTrack = tk.Radiobutton(self.videoFrame, bg=self.tkcolor,fg='ivory', activeforeground=tkButtonColor,  activebackground=self.tkcolor, selectcolor=self.tkcolor, text="Stop tracking", variable=self.startWormTracking, value=False, command=self.stopTracking)  # radio button to chose to stop tracking the worm        
        rButtonStopTrack.select()
        rButtonStartTrack.grid(row = 0, column = 0)
        self.ButtonIsHead.grid(row = 0, column = 2)
        self.ButtonOtherEnd.grid(row = 0, column = 3)
        rButtonStopTrack.grid(row = 0, column = 1)
#-------define ingridients and structure of the last frame----------------------#
        self.RecButton = tk.Button(lastFrame, bg=tkButtonColor,text="Start rec.", state='disabled', command = self.start_Rec) # button to start a recording
        self.stopRecButton = tk.Button(lastFrame, bg=tkButtonColor,text="Stop rec.", state='disabled', command = self.stop_Rec) # button to stop the current recording        
        buttonQuit = tk.Button(lastFrame, bg=tkButtonColor,text="Save & Quit",command=self.onClose) # button to save and quit
        self.RecButton.grid(row = 0, column = 0)
        self.stopRecButton.grid(row = 0, column = 1)        
        buttonQuit.grid(row = 0, column = 2)             
#-------define additional tkinter triggers--------------------------------------#
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.start()
        self.root.wm_title("USPokingTracker")
        self.traceWindow = None
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)
        
    def wormChosen(self):
        '''Changes the acquisition rate and interstimulus interval to match the worm speed
        '''
        self.matchWormFly = 1
        self.standardProtocol[3] = 10
        
    def flyChosen(self):
        '''Changes the acquisition rate and interstimulus interval to match the fy speed
        '''
        self.matchWormFly = 2        
        self.standardProtocol[3] = 20      
        
    def videoLoop(self):
        '''Loops through the recording of a video and displays the newest image
           Also: image processing:
               - deletes the area far away from worm
               - uses stage to tracks the worm
        '''
        try:
            while not self.stopEvent.is_set():
                ret, self.frame = self.cap.read()                               # the camera is read out
                displayedImage = self.frame#[:,:,0]                             # the image that is dsiplayed in the GUI
                thresh = self.findThreshold()                                   # a threshold is estimated, like the Otsu threshold but 1/10 bigger
                ret1, img1 = cv2.threshold(displayedImage[:,:,0],thresh,255,cv2.THRESH_BINARY)  # the threshold is applied
                seeWorm = False
                self.currentStagePos = self.stagePosition()
                if self.tracktheWorm == True and self.stageisCalibrated == True:  # if a protocol is chosen and the stage is calibrated the image analysis starts
                    img1 = cv2.dilate(img1, None, iterations=5)                 # dilates the frame 5 times                
                    img1 = cv2.erode(img1, None, iterations=5)                  # erodes the frame 5 times 
                    img2 = skimor.remove_small_holes(np.divide(img1,255).astype(bool), # removes small holes
                                                     min_size=50, connectivity=1, in_place=False)
                    img3 = skimor.remove_small_objects(img2, min_size=50,       # removes small objects
                                                       connectivity=1, in_place=False).astype(int)
                    contours = None                                             # contours in the image
                    try:
                       im,contours,hierarchy = cv2.findContours(img3,cv2.RETR_FLOODFILL, cv2.CHAIN_APPROX_SIMPLE)   # tries to find contours
                    except:
                        pass
                    self.ellip = None                                           # ellipses
                    if len(contours) >= 2:
                        try:
                            cMax = max(contours, key = cv2.contourArea)         # if contours were found the largest contour areawise is determined
                            try:
                                self.ellip = cv2.fitEllipse(cMax)               # tries to fit an ellipse to the largest contour
                                mask = np.zeros(img3.shape).astype(img3.dtype)  # a mask of zeros
                                cv2.ellipse(mask,self.ellip,(255),-1)           # the ellipse is set to 255 in the mask 
                                img3 = np.divide(cv2.bitwise_and(img3*255, mask),255).astype(bool)  # only the area of the ellipse will be investigated further
                                img4 = skimor.skeletonize_3d(img3).astype(int)  # the area is skeletonized
                                self.endsofWorm = self.enpointsofWormSkeleton(img4) # the endpoints of the skeleton is determined and the endpoints that are farest apart
                                                                                        # are estimated to be the endpoints of the worm
                                self.moveStagetoFollowWorm()                    # the stage is moved to follow the endpoint in the direction of movement
                            except:                        
                                pass
                        except:                                                 # if no ellipse can be fitted the tracking stops
                            self.stopTracking()
                            pass                   
                    if self.ellip!= None:
                        seeWorm = True
                    else:
                        seeWorm = False
                if self.USAlignement==True or self.stageisCalibrated == True and self.recording == False:
                    cv2.circle(displayedImage, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2) 
                    if seeWorm == True:
                        cv2.circle(displayedImage, (int(self.ellip[0][0]),int(self.ellip[0][1])), 11, color = ( 0.2, 1, 1 ), thickness = 2)                         
                        try:                        
                            cv2.circle(displayedImage, (self.endsofWorm[1][0],self.endsofWorm[1][1]), 11, color = ( 0.2, 1, 1 ), thickness = 2) 
                            cv2.circle(displayedImage, (self.endsofWorm[0][0],self.endsofWorm[0][1]), 11, color = ( 0.2, 1, 1 ), thickness = 2) 
                            cv2.circle(displayedImage, (self.targetPos[0],self.targetPos[1]),5, color = ( 0.2, 1, 0.8 ), thickness = 5)   
                        except:
                            pass                            
                if self.stateProtocol == False or self.stageisCalibrated == False or seeWorm == False or self.recording == True:
                    self.RecButton.config(state="disabled")                     # recoding can not be started if no US protocol is chosen, the stage is not calibrated,
                                                                                    # no worm is found or another recording is still ongoing
                    self.ButtonIsHead.config(state="disabled")                  # sets the button to define the head as disabled   
                else:
                    self.RecButton.config(state="normal")
                    self.ButtonIsHead.config(state="normal")                    # sets the button to define the head as normal                    
#---------------display the image in the GUI------------------------------------#                                 
                image = ImageTk.PhotoImage(Image.fromarray(displayedImage))     
                if self.videoDisplay is None:
                    self.videoDisplay = tk.Label(self.videoFrame, image=image)
                    self.videoDisplay.image = image
                    self.videoDisplay.grid(row=2, columnspan = 4)  	
                else:
                    self.videoDisplay.configure(image=image)
                    self.videoDisplay.image = image
                time.sleep(0.05)
        except RuntimeError:
            pass

    def findThreshold(self):
        ''' evaluates a threhold similar to the Otsu method:
                - splits the histogram in two parts
                - minimizes the sum of the variance of the two parts 
            input is an 8-bit bw image
            output is an 8-bit value - the threshold
        '''
        hist = cv2.calcHist([self.frame[:,:,0]],[0],None,[256],[0,256])         # finds histogram
        hist_norm = hist.ravel()/hist.max()                                     # normalizes histogram
        Q = hist_norm.cumsum()                                                  # cumulates hisotgram
        bins = np.arange(256)                                                   # bins
        fn_min = np.inf                                                         # the minimum of the variance of the two histogram parts 
        thresh = -1                                                             # the threshold (id the same as with Otsu function)
        for i in range(1,256):                                                  # i is the value for splitting the histogram
            p1,p2 = np.hsplit(hist_norm,[i])                                    # the probabilities
            q1,q2 = Q[i],Q[255]-Q[i]                                            # cumulative sum of classes
            if q1 != 0 and q2 != 0:                                             # only evaluates the threshold if both parts of the histogram include values
                b1,b2 = np.hsplit(bins,[i])                                     # the weights
                m1,m2 = np.sum(p1*b1)/q1, np.sum(p2*b2)/q2                      # the means
                v1,v2 = np.sum(((b1-m1)**2)*p1)/q1,np.sum(((b2-m2)**2)*p2)/q2   # the variances
                fn = v1*q1 + v2*q2                                              # calculates the minimization function
                if fn < fn_min:                                                 # if the minimum is smaller than any minimum before a new way to split the histogram was found
                    fn_min = fn
                    thresh = i
        return thresh+int(thresh/10)                                            # adds 1/10 to the Otsu threshold

    def enpointsofWormSkeleton(self, image):
        '''Determines the endoints of the worm's skeleton
           input: an image with one(!) connected skeleton
           output: two points (opposite endpoints of a skeletone line)
        '''
        countNeighborsFilter = np.array([ [1,1,1],                              # matrix as filter to count neighbors
                                          [1,0,1],
                                          [1,1,1] ], dtype = np.uint8)
        distMax = 0                                                             # maximal distance between endpoints
        image1 = (image/255).astype(np.uint8)                                   # loads the image as 8bit with 1 on the skeletone
        neighborsCount = cv2.filter2D(image1, -1, countNeighborsFilter, borderType = cv2.BORDER_CONSTANT)   # counts the neighbors of each pixel in the image and
                                                                                                                # puts this number as the pixelvalue
        pixels = np.multiply(neighborsCount,image1)                             # multiplies the image of the skeleton with the image of the number of neighboring pixels
        e = np.where(pixels==1)                                                 # the image above will only at the endpoints of the skeleton be 1
        for i in range (0,len(e[0])):
            for j in range (i+1,len(e[0])):
                if np.linalg.norm(np.subtract([e[1][i],e[0][i]],[e[1][j],e[0][j]]))>=distMax:   # calculates the distance between each pair of endpoints
                    distMax = np.linalg.norm(np.subtract([e[1][i],e[0][i]],[e[1][j],e[0][j]]))  # the maximum distance is the length of the worm
                    a = [[e[1][i],e[0][i]],[e[1][j],e[0][j]]]                   # the endpoints of the worm's skeleton
        return a 

    def fileSave(self):
        f = filedialog.askdirectory()
        if f is None:                                                           # asksaveasfile return `None` if dialog closed with "cancel".
            self.filedir = self.filedir
        else:
            self.filedir = str(f+'/')                            # starts from `1.0`, not `0.0`
            self.labelSaveDir = tk.Label(self.topFrame, bg=self.tkcolor, fg='ivory', text="Files are safed to: "+self.filedir) # shows the directory that all files are saved to in the GUI  
            self.labelSaveDir.grid(row = 0, columnspan = 2, padx=3)

    def startTracking(self):
        '''Starts tracking of the worm
        '''
        self.tracktheWorm = True
        self.startWormTracking.set(True)
        self.searchforHead = True
    
    def stopTracking(self):
        '''Stops tracking of the worm
        '''
        self.tracktheWorm = False
        self.ButtonIsHead.config(state="disabled")
        self.startWormTracking.set(False)
        self.searchforHead = True
        
    def tenTouchChosen(self):
        '''Trigger if standard protocol is chosen
        '''
        self.changeProtocol.config(state="disabled")
        self.stateProtocol = True
        self.protocol = self.standardProtocol
 #       self.timeOn = ((self.protocol[0]/100)*1000/(self.protocol[2]))/1000
  #      self.timeOff = (((100-self.protocol[0])/100)*1000/(self.protocol[2]))/1000
#        print(self.timeOn,self.timeOff)
        
    def protocolChosen(self): 
        '''Trigger if a custom protocol is chosen
        '''
        self.changeProtocol.config(state="normal")
        self.stateProtocol = False

    def saveProtocol(self):
        '''Saves the parameters of custom US protocols and saves them into a .txt file
        '''
        for i in range (0, len(self.PNames)):
            if  i == 0:
                text = self.PNames[i] + ": " + (datetime.now().strftime('%Y/%m/%d, %H:%M:%S')) + "\n"
            elif i == 1:
                text = self.PNames[i] + ": " + self.Pentry1.get() + "\n"
                self.protocol[0] = self.Pentry1.get()
            elif i == 2:
                text = self.PNames[i] + ": " + self.Pentry2.get() + "\n"
                self.protocol[1] = self.Pentry2.get()
            elif i == 3:
                text = self.PNames[i] + ": " + self.Pentry3.get() + "\n"
                self.protocol[2] = self.Pentry3.get()                
            elif i == 4:    
                text = self.PNames[i] + ": " + self.Pentry4.get() + "\n"
                self.protocol[3] = self.Pentry4.get()                  
            elif i == 5:                
                text = self.PNames[i] + ": " + self.Pentry5.get() + "\n" + "\n"     
                self.protocol[4] = self.Pentry5.get()                                        
            with open(self.filedir+"protocol.txt", "a") as f:
                f.write(text)
        self.protocol[5] = False  
        self.stateProtocol = True
        self.setProtocol.destroy()

    def change_Protocol(self):
        '''Changes the protocol for the US stimulus if custom protocol is chosen
        grabs individual US parameters, saves them into a .txt file
        '''
        self.setProtocol = tk.Toplevel(bg=self.tkcolor)
        self.setProtocol.resizable(0,0)  
        self.setProtocol.wm_title("Protocol")
        self.PNames = []
        heightText = 2
        widthText = 30
        r = 6
        for i in range(0,r):
            if i == 0:            
                self.PNames.append('Date, Time')
                self.Pentry = datetime.now().strftime('%Y/%m/%d, %H:%M:%S\n')                
            tk.Label(self.setProtocol, bg=self.tkcolor, fg='ivory', height=heightText, width = widthText, text=self.Pentry).grid(row=0, column=1)                    
            if i == 1:
                self.PNames.append('Duty Cycle (0-100) [%]')
                self.Pentry1 = tk.Entry(self.setProtocol, bg='ivory')
                self.Pentry1.grid(row=i, column=1)
            elif i == 2:
                self.PNames.append('Pulse length [ms]')
                self.Pentry2 = tk.Entry(self.setProtocol, bg='ivory')
                self.Pentry2.grid(row=i, column=1)
            elif i == 3:
                self.PNames.append('Pulse repitition frequency (<100) [Hz]')
                self.Pentry3 = tk.Entry(self.setProtocol, bg='ivory')
                self.Pentry3.grid(row=i, column=1)
            elif i == 4:
                self.PNames.append('Inter-stimulus interval [s]')
                self.Pentry4 = tk.Entry(self.setProtocol, bg='ivory')
                self.Pentry4.grid(row=i, column=1)                
            elif i == 5:
                self.PNames.append('Number of pulses per animal')
                self.Pentry5 = tk.Entry(self.setProtocol, bg='ivory')
                self.Pentry5.grid(row=i, column=1)
            tk.Label(self.setProtocol, bg=self.tkcolor, fg='ivory', height=heightText, width = widthText, text=self.PNames[i]).grid(row=i, column=0)   
        tk.Button(self.setProtocol, bg='light goldenrod', text="Save & Quit", command=self.saveProtocol).grid(row=r+1, column=1)   
        
    def saveMetaData(self):
        '''Saves the metadata into a .txt file
        '''
        for i in range (0, len(self.Names)):
            if  i == 0:
                text = self.Names[i] + ": " + (datetime.now().strftime('%Y/%m/%d, %H:%M:%S')) + "\n"
            elif i == 1:
                text = self.Names[i] + ": " + self.entry1.get() + "\n"
            elif i == 2:
                text = self.Names[i] + ": " + self.entry2.get() + "\n"
            elif i == 3:
                text = self.Names[i] + ": " + self.entry3.get() + "\n"
            elif i == 4:    
                text = self.Names[i] + ": " + self.entry4.get() + "\n"
            elif i == 5:    
                text = self.Names[i] + ": " + self.entry5.get() + "\n"     
            elif i == 6:    
                text = self.Names[i] + ": " + self.entry6.get() + "\n"  
            elif i == 7:    
                text = self.Names[i] + ": " + self.entry7.get() + "\n"                  
            elif i == 8:                
                text = self.Names[i] + ": " + self.entry8.get() + "\n" + "\n"             
            with open(self.filedir+"metadata.txt", "a") as f:
                f.write(text)
        self.metaData.destroy()
                
    def change_Metadata(self):
        '''Changes the metadata
        '''
        self.Names = []
        heightText = 2
        widthText = 20
        self.metaData = tk.Toplevel(bg=self.tkcolor)
        self.metaData.resizable(0,0)        
        self.metaData.wm_title("Metadata")
        r = 9
        for i in range(0,r):
            if i == 0:            
                self.Names.append('Date, Time')
                self.entry = datetime.now().strftime('%Y/%m/%d, %H:%M:%S\n')                
            tk.Label(self.metaData, bg=self.tkcolor, fg='ivory', height=heightText, width = widthText, text=self.entry).grid(row=0, column=1)                    
            if i == 1:
                self.Names.append('Temperature [°C]')
                self.entry1 = tk.Entry(self.metaData, bg='ivory')
                self.entry1.grid(row=i, column=1)
            elif i == 2:
                self.Names.append('Humidity [%]')
                self.entry2 = tk.Entry(self.metaData, bg='ivory')
                self.entry2.grid(row=i, column=1)
            elif i == 3:
                self.Names.append('Worm strain')
                self.entry3 = tk.Entry(self.metaData, bg='ivory')
                self.entry3.grid(row=i, column=1)
            elif i == 4:
                self.Names.append('Worm age')
                self.entry4 = tk.Entry(self.metaData, bg='ivory')
                self.entry4.grid(row=i, column=1)                
            elif i == 5:
                self.Names.append('Protocol')
                self.entry5 = tk.Entry(self.metaData, bg='ivory')
                self.entry5.grid(row=i, column=1)
            elif i == 6:
                self.Names.append('Added drugs?')
                self.entry6 = tk.Entry(self.metaData, bg='ivory')
                self.entry6.grid(row=i, column=1)
            elif i == 7:
                self.Names.append('Cultivation')
                self.entry7 = tk.Entry(self.metaData, bg='ivory')
                self.entry7.grid(row=i, column=1)                
            elif i == 8:
                self.Names.append('Comments')
                self.entry8 = tk.Entry(self.metaData, bg='ivory')
                self.entry8.grid(row=i, column=1)
            tk.Label(self.metaData, bg=self.tkcolor, fg='ivory', height=heightText, width = widthText, text=self.Names[i]).grid(row=i, column=0)     
        tk.Button(self.metaData, bg='light goldenrod', text="Save & Quit", command=self.saveMetaData).grid(row=r+1, column=1)           

    def digital_write(self, digital_value):
        '''Writes the digital_value on self.USPin
           Internally sends b'WD{pin_number}:{digital_value}' over the serial
           connection
        '''      
        command = (''.join(('WD', str(self.USPin), ':',
            str(digital_value), '/', str(0),'-', str(0)))).encode()
        self.conn.write(command) 

    def digital_write_frequency(self, pulses, timeOn, timeOff):
        '''Writes a sequence of 1 and 0 to the self.USPin, with the number of (1/0),
           the time for 1 and the time for 0. Internally sends b'FD{pin_number}:{pulses}/{timeOn}-{timeOff}' over the serial
           connection
        '''
        command = (''.join(('FD', str(self.USPin),':',str(pulses), '/', str(int(self.timeOn*1000)),'-', str(int(self.timeOff*1000))))).encode()
        self.conn.write(command) 
          
    def arduinoClose(self):
        """
        To ensure we are properly closing our connection to the
        Arduino device. 
        """
        self.conn.close()  

    def US_puls(self):
        '''Applies an US pulse, according to the next step in the chosen protocol
        '''
        self.digital_write_frequency(int(self.protocol[2]*self.protocol[1]/1000), (int)(self.timeOn*1000), (int)(self.timeOff*1000))
        self.digital_write(0)

    def calibrationUSPuls(self):
        '''Gives US pulses that can be used to align the US transducer with the camera.
           The Arduino pin controlling the US transducer is on for 0.1 s and off for 1 s for total
           of 25 times. While the pin is off the videoloop is displaying images. 
           On theses images the fountain caused by the US should be visible and can be used 
           to align the US focus with the center of the camera's field of view which is displayed with a circle.
        '''
        if self.PulseTrainNumber == 0:
            self.USAlignement=True
        self.digital_write(1)                                      # turn US on 
        time.sleep(0.1)                                                         # leaves the US on for 0.1 s
        self.digital_write(0)                                      # turn US off
        if self.PulseTrainNumber < 25:
            self.PulseTrainNumber += 1
            self.videoDisplay.after(int(1000), self.calibrationUSPuls)
        else: 
            self.digital_write(0)                                  # turn US off                
            self.USAlignement=False
            self.PulseTrainNumber = 0        

    def calibrateStage(self):
        '''Calibrates stage movement to pixel
           takes an initial image, moves left takes a second image, moves back and down and takes a third image
           uses openCVs findTransformEPS algorithm to compare the second/third image with the first image to identify transformation
           calculates a transformation matrix from pixels to stage movement and the inverse
        '''
        moveStageSteps = 20
        if len(self.ImagestoCalibrate)<=2:
            x_pos = get_position_t()
            result = lib.get_position(self.device_id[0], byref(x_pos))
            yPos = repr(x_pos.Position)
            result2 = lib.get_position(self.device_id[1], byref(x_pos))
            xPos = repr(x_pos.Position)  
            self.stagePostoCalibrate.append([xPos,yPos])
            self.ImagestoCalibrate.append(self.frame[:,:,0])
            if len(self.ImagestoCalibrate) == 1:
                lib.command_movr(self.device_id[1], int((-moveStageSteps)), 0)
            elif len(self.ImagestoCalibrate) == 2:
                lib.command_movr(self.device_id[1], int((moveStageSteps)), 0)
                lib.command_movr(self.device_id[0], int((-moveStageSteps)), 0)
            if len(self.ImagestoCalibrate)<=2:
                self.videoDisplay.after(1000, self.calibrateStage)               
            else:
                sz = self.ImagestoCalibrate[0].shape
                warp_mode = cv2.MOTION_TRANSLATION                              # motion model is translation
                number_of_iterations = 5000;                                    # number of iterations
                termination_eps = 1e-10;                                        # threshold of the increment in the correlation coefficient between two iterations
                criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, number_of_iterations,  termination_eps) # termination criteria     
                for i in range (1,3):
#-------------------Runs the ECC algorithm on both images-----------------------#
#-------------------the results are stored in warp_matrix-----------------------#                    
                    warp_matrix = np.eye(2, 3, dtype=np.float32)                 
                    (cc, warp_matrix) = cv2.findTransformECC(self.ImagestoCalibrate[0], self.ImagestoCalibrate[i], warp_matrix, warp_mode, criteria)
                    if i == 1:
                        for k in range (0,2):
                            if np.absolute(warp_matrix[k][2]) >= 0.5:
                                self.rotationMatrix[k][i-1] = -warp_matrix[k][2]/(float(self.stagePostoCalibrate[i][0])-float(self.stagePostoCalibrate[0][0]))
                            else:
                                self.rotationMatrix[k][i-1] = 0
                    if i == 2:
                        for k in range (0,2):
                            if np.absolute(warp_matrix[k][2]) >= 0.5:
                                self.rotationMatrix[k][i-1] = -warp_matrix[k][2]/(float(self.stagePostoCalibrate[i][1])-float(self.stagePostoCalibrate[0][1]))
                            else:
                                self.rotationMatrix[k][i-1] = 0
                factor = 1/((self.rotationMatrix[0][0]*self.rotationMatrix[1][1])-(self.rotationMatrix[1][0]*self.rotationMatrix[0][1]))
                self.InverseRotationMatrix = [[self.rotationMatrix[1][1]*factor,-self.rotationMatrix[1][0]*factor],[-self.rotationMatrix[0][1]*factor,self.rotationMatrix[0][0]*factor]]
                self.stageisCalibrated = True

    def manualMoveStage_up(self):
        '''Moves the stage up
        '''
        lib.command_movr(self.device_id[1], int((240*self.InverseRotationMatrix[0][1])), 0)
        lib.command_movr(self.device_id[0], int((240*self.InverseRotationMatrix[1][1])), 0)

    def manualMoveStage_down(self):
        '''Moves the stage down
        '''
        lib.command_movr(self.device_id[1], int((-240*self.InverseRotationMatrix[0][1])), 0)
        lib.command_movr(self.device_id[0], int((-240*self.InverseRotationMatrix[1][1])), 0)        

    def manualMoveStage_left(self):
        '''Moves the stage left
        '''
        lib.command_movr(self.device_id[1], int((320*self.InverseRotationMatrix[0][0])), 0)
        lib.command_movr(self.device_id[0], int((320*self.InverseRotationMatrix[1][0])), 0)         
        
    def manualMoveStage_right(self):
        '''Moves the stage right
        '''
        lib.command_movr(self.device_id[1], int((-320*self.InverseRotationMatrix[0][0])), 0)
        lib.command_movr(self.device_id[0], int((-320*self.InverseRotationMatrix[1][0])), 0)
        

    def wormVelocityCalc(self):
        '''Here the last ten positions of the worm (so far the positions of the stage) are stored
           and the worm's normalized velocity of these ten positions is calculated
        '''
        StageX, StageY = self.currentStagePos
        self.PositionWorm.append([int(self.ellip[0][0]+StageX),int(self.ellip[0][1]+StageY)])
        lentoTrack = 20
        if len(self.PositionWorm)==lentoTrack:  
            self.wormVelocities.append([self.PositionWorm[lentoTrack-1][0]-self.PositionWorm[0][0],self.PositionWorm[lentoTrack-1][1]-self.PositionWorm[0][1]])
            del self.PositionWorm[0]           
            if len(self.wormVelocities)==20:
                del self.wormVelocities[0]            
            self.wormVelocity = np.mean(self.wormVelocities, axis=0)
            self.wormVelocity=self.wormVelocity.reshape(1, -1)
            self.normWormVelocity = [normalize(self.wormVelocity, axis=1, norm='l2')[0][0], normalize(self.wormVelocity, axis=1, norm='l2')[0][1]]         
        else:
            self.normWormVelocity = [self.ellip[0][0],self.ellip[0][1]]

    def wormHead(self):
        '''Decides which end of the worm is the one to track, 
           based on the direction of movement
        '''
        pos3 = np.array([[(self.endsofWorm[0][0]-self.ellip[0][0]), (self.endsofWorm[0][1]-self.ellip[0][1])]])
        pos4 = np.array([[(self.endsofWorm[1][0]-self.ellip[0][0]), (self.endsofWorm[1][1]-self.ellip[0][1])] ])                 
        pos3 = [normalize(pos3, axis=1, norm='l2')[0][0], normalize(pos3, axis=1, norm='l2')[0][1]]
        pos4 = [normalize(pos4, axis=1, norm='l2')[0][0], normalize(pos4, axis=1, norm='l2')[0][1]]                
        if np.dot(self.normWormVelocity,pos3)>=np.dot(self.normWormVelocity,pos4):
            return self.endsofWorm[1][0],self.endsofWorm[1][1]
        else:
            return self.endsofWorm[0][0],self.endsofWorm[0][1]

    def lockonHead(self):
        '''Defines and locks on the currently focused end of the worm as the target
        '''
        self.searchforHead = False
        self.ButtonOtherEnd.config(state="normal") 
        self.oldHead = self.wormHead()
        
    def lockonotherEnd(self):
        '''Defines and locks on the currently focused end of the worm as the target
        '''
        if self.oldHead==(self.endsofWorm[0][0],self.endsofWorm[0][1]):
            self.oldHead = self.endsofWorm[1][0],self.endsofWorm[1][1] 
        else:
            self.oldHead =self.endsofWorm[0][0],self.endsofWorm[0][1]
        if self.recording == True:
            self.lockonotherEndpushed = True         

    def stagePosition(self):
        '''Returns the position of the stage in units of pixels
        '''
        x_pos = get_position_t()
        result = lib.get_position(self.device_id[0], byref(x_pos))
        yPos = int(repr(x_pos.Position))
        result2 = lib.get_position(self.device_id[1], byref(x_pos))
        xPos = int(repr(x_pos.Position))
        StageXPos = self.rotationMatrix[0][0]*xPos+self.rotationMatrix[0][1]*yPos
        StageYPos = self.rotationMatrix[1][0]*xPos+self.rotationMatrix[1][1]*yPos  
        return -StageXPos, -StageYPos

    def closerEndofWorm(self):
        '''Determines end of worm that is closest to the last frame's head
        '''
        if ((self.endsofWorm[0][0]-self.oldHead[0])**2+(self.endsofWorm[0][1]-self.oldHead[1])**2)<=((self.endsofWorm[1][0]-self.oldHead[0])**2+(self.endsofWorm[1][1]-self.oldHead[1])**2):
            self.oldHead =self.endsofWorm[0][0],self.endsofWorm[0][1]
            return self.endsofWorm[0][0],self.endsofWorm[0][1]
        else:
            self.oldHead = self.endsofWorm[1][0],self.endsofWorm[1][1]  
            return self.endsofWorm[1][0],self.endsofWorm[1][1]            
            
    def moveStagetoFollowWorm(self):
        '''Moves the stage to track the worm
        '''
        if self.searchforHead == True:        
            self.wormVelocityCalc()                                             # evaluates the worm's velocity vector
            if (self.wormVelocities[-1][0]**2+self.wormVelocities[-1][1]**2)>=100:
                self.targetPos = self.wormHead()                                # evaluates the position of the worm (head or tail) that faces the direction of movement 
            else:
                self.targetPos = self.ellip[0][0],self.ellip[0][1]
        else:
            self.targetPos = self.closerEndofWorm()
        move1 = int(self.InverseRotationMatrix[0][0]*(self.targetPos[0]-320) + (self.targetPos[1]-240)*self.InverseRotationMatrix[0][1]) # the distance in x that the stage has to move to center the head/tail
        move0 = int(self.InverseRotationMatrix[1][0]*(self.targetPos[0]-320) + (self.targetPos[1]-240)*self.InverseRotationMatrix[1][1]) # the distance in y that the stage has to move to center the head/tail
        lib.command_movr(self.device_id[1], -int(move1*0.15), 0)                 # moves the x-axis of the stage 
        lib.command_movr(self.device_id[0], -int(move0*0.15), 0)                 # moves the y-axis of the stage 

    def stop_Rec(self):
        '''Stops the current recording of a video:
           The video recording can be stopped in case anything unforseen happens
        '''
        self.recordingStopped = True

    '''
    def positionsforCheck(self):
        if self.undefined == True:
            self.ja = 0
            self.PosWX,self.PosWY,self.PosSX,self.PosSY=[],[],[],[]
            for i in range (0,71):
                self.PosWX.append(i)
                self.PosWY.append(i)
                self.PosSX.append(320)
                self.PosSY.append(240)
            self.undefined = False
        else:
            self.ja +=1
        return self.PosWX[self.ja], self.PosWY[self.ja], self.PosSX[self.ja], self.PosSY[self.ja]
    '''                
    def start_Rec(self):
        '''Starts the recording of a video:
           records the time of the taken images, estimates position of the worms
           opens a canvas to show the velocity vs time, moves the stage according to worm position and
           saves the individual results before destroying all involvoed processes
        '''
        if self.recordingStopped == True:
            self.stopTracking()
            self.stopRecButton.config(state="disabled")                             # disables the button to stop the recording 
            self.PulseNumber = 0
            self.WormNumber += 1
            self.frameNumber = 0
            self.individualResult = []
            self.PosWormX, self.PosWormY = [],[]
            self.PosStageX, self.PosStageY = [],[]
            self.times = []                           
            self.out.release()
            self.recording = False
            self.firstrecording = True
            cv2.destroyAllWindows() 
            self.recordingStopped = False
        elif self.lockonotherEndpushed == True:
            try:
                self.wormsTrace.delete(self.line[self.PulseNumber][:])   
                del self.line[self.PulseNumber]
                self.out.release()                
            except:
                pass
            self.ab = 1
            self.frameNumber = self.numofFramesperPuls*(self.PulseNumber)
            self.individualResult = []
            self.PosWormX, self.PosWormY = [],[]
            self.PosStageX, self.PosStageY = [],[]
            self.times = []            
            self.lockonotherEndpushed = False
            self.videoDisplay.after(100*self.matchWormFly, self.start_Rec)    
        else:
            imagetoSave = np.copy(self.frame)                                       # copy the video from the videoloop to save it later
            self.frameNumber += 1                                                   # frame of the current recording
            if self.firstrecording == True:
    #-----------initilizations at the first recorded frame--------------------------#
                self.stopRecButton.config(state="normal")                           # enables the button to stop the recording
                self.firstrecording = False                                         # sets the trigger for not being the first recording of this worm
                self.recording = True                                               # the software is recording, no other recording can be started  
                self.allResults.append([])
    #-----------create a window for showing the tracks of this worm-----------------#
                self.traceWindow = tk.Toplevel()                                    # new window to show the track of this worm
                self.traceWindow.wm_title("Worm Trace")
                self.wormsTrace = tk.Canvas(self.traceWindow, width=1000, height=800, bg='ivory')
                self.NumbRec = tk.Label(self.traceWindow, bg=self.tkcolor, fg='ivory', height=2, width=120, text=self.frameNumber)
                self.NumbRec.pack(fill = tk.X)
                self.wormsTrace.create_line(790, 720, 940, 720, fill =self.tkcolor, width = 5)
                self.la = tk.Label(self.traceWindow, bg='ivory', fg=self.tkcolor, height=2, width=150, text='5 mm')  
                self.la.place(x = 790, y = 760, width=150, height=25)
                self.wormsTrace.pack()
    #-----------create a window for showing the results of all worms----------------#  
                if self.WormNumber == 0:
                    self.touchAssayWindow = tk.Toplevel()
                    self.touchAssayWindow.wm_title("Touch Assay Result")
                    self.touchAssay = tk.Canvas(self.touchAssayWindow, width=200, height=600, bg='ivory')
                    self.touchAssay.pack()        
#                    self.touchAssay.overrideredirect(1)
            if self.frameNumber==1:
                self.timeFirstFrame = time.time()                                   # the time of the first frame is the comparison time for all the other frames 
                self.startingPosStageX, self.startingPosStageY = int(self.currentStagePos[0]), int(self.currentStagePos[1]) # initial position of the stage   
            if self.frameNumber == (self.numofFramesperPuls*(self.PulseNumber))+1:
                self.line.append([])                                                # appends a new empty list for the trace                                        
                self.ab = 1                                                         # counter for dividing the frames per pulse into 5 5th                       
                self.individualResult = []
    #-----------initialize the video output-----------------------------------------#
                videoName = 'w'+'{0:0=3d}'.format(self.WormNumber)+'p'+'{0:0=2d}'.format(self.PulseNumber+1)+'.avi' # the name for the saved video file
                #todo: if this name exist already chose the next highest worm number
                
                self.out = cv2.VideoWriter(self.filedir+videoName,                  # defines the video writer for the saved video
                                           cv2.VideoWriter_fourcc(*'DIB '),
                                           20.0, (640,480), False)  
            if self.frameNumber <= self.numofFrames:
                self.timeActualFrame = np.round(time.time()-self.timeFirstFrame,3)  # the time that this frame was recorded
                self.times.append(self.timeActualFrame)
 #               Positions = self.positionsforCheck()
    #            print(Positions)
       #         self.PosStageX.append(Positions[2])
       #         self.PosStageY.append(Positions[3])
                self.PosStageX.append(int(self.currentStagePos[0])-self.startingPosStageX)  # the stage's x-position when this frame was recorded
                self.PosStageY.append(int(self.currentStagePos[1])-self.startingPosStageY)  # the stage's y-position when this frame was recorded                
                if self.ellip != None:                                              # if an ellipse was fitted to the outline of the worm in the videoloop
                    self.PosWormY.append(int(-(self.InverseRotationMatrix[0][0]*(self.ellip[0][0]-self.PosStageX[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1])+
                                         self.InverseRotationMatrix[0][1]*(self.ellip[0][1]-self.PosStageY[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1]))*2.5)/1000) # the x-position of the worm when this frame was recorded
                    self.PosWormX.append(int((self.InverseRotationMatrix[1][0]*(self.ellip[0][0]-self.PosStageX[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1])+
                                         self.InverseRotationMatrix[1][1]*(self.ellip[0][1]-self.PosStageY[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1]))*2.5)/1000)   # the y-position of the worm when this frame was recorded
 #                   self.PosWormX.append(Positions[0])
 #                   self.PosWormY.append(Positions[1])                    
                    if self.frameNumber != (self.numofFramesperPuls*(self.PulseNumber))+1:  # updates the trace only from the second image of each recording on
                        self.NumbRec.configure(text=self.frameNumber)
                        try:
                            self.displayWormTrace()                                 # displays the trace of the worm in the trace window                  
                        except:
                            pass
                    self.individualResult.append([self.timeActualFrame,self.PosStageX[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1],self.PosStageY[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1],
                                                  self.PosWormX[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1],self.PosWormY[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1]])  # stores the time of the frame, the position of the stage, and the worm              
                else:                                                               # if no ellipse was fitted to the outline of the worm in the videoloop
                    self.PosWormX.append('NaN')                                     # puts the x and y position of the worm to 'NaN'
                    self.PosWormY.append('NaN')
                    self.individualResult.append([self.timeActualFrame,self.PosStageX[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1],
                                                  self.PosStageY[self.frameNumber-(self.numofFramesperPuls*self.PulseNumber)-1],'',''])  # stores the time of the frame, the position of the stage, and leaves the position of the worm empty
                if self.frameNumber == self.numofFrames:
    #---------------last image for this worm: results are calculated and displayed--# 
    #---------------then everything is reset for next worm--------------------------#
 #                   speedUp, pause, angChange = self.analyzeWormMovment()                # analyzes whether there was a change in speed or angle of velocity
                    speedUp, pause, angChange = self.UTA.analyzeTrack(0.1/(2*self.matchWormFly),self.USPulseFifth,
                                                                      self.times,self.PosWormX,self.PosWormY)
                    if speedUp == True or angChange == True :                    # if the speed and the angle changed a black box is put into the assay window
                        boxColor = 'black'
                        self.allResults[self.WormNumber].append(1)
                    elif pause == True:                                         # if only the speed changed a blue box is put into the assay window
                        boxColor = 'blue'
                        self.allResults[self.WormNumber].append(0)
#                    elif angChange == True:                                         # if only the angle changed a green box is put into the assay window
#                        boxColor = 'green'
#                        self.allResults[self.WormNumber].append(1)
                    else:                                                           # if neither the speed nor the angle changed a white box is put into the assay window
                        boxColor = 'white'
                        self.allResults[self.WormNumber].append(0)
                    BoxX, BoxY = 32+(15*self.PulseNumber), 22+(25*self.WormNumber)  # the size and position of the box
                    self.touchAssay.create_rectangle(BoxX, BoxY, BoxX+10, BoxY+20, fill=boxColor)                
                    self.PulseNumber +=1
                    self.stopTracking()
                    self.saveIndividualResults()                                    # the results for this worm are saved
                    self.PulseNumber = 0
                    self.WormNumber += 1
                    self.frameNumber = 0
                    self.individualResult = []
                    self.PosWormX, self.PosWormY = [],[]
                    self.PosStageX, self.PosStageY = [],[]
                    self.times = []
                    self.stopRecButton.config(state="disabled")                            # disables the button to stop the recording                 
 #                   cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)                
                    self.out.write(imagetoSave[:,:,0])
                    self.out.release()
                    self.recording = False
                    self.firstrecording = True
                    cv2.destroyAllWindows()
                elif self.frameNumber == self.numofFramesperPuls*(self.PulseNumber+1):
    #---------------last image for this puls: results are calculated and displayed--#
    #---------------then interstimulus time is waited-------------------------------#
                    speedUp, pause, angChange = self.UTA.analyzeTrack(0.1/(2*self.matchWormFly),self.USPulseFifth,
                                                                      self.times,self.PosWormX,self.PosWormY)                # analyzes whether there was a change in speed or angle of velocity
                    if speedUp == True or angChange == True :                    # if the speed and the angle changed a black box is put into the assay window
                        boxColor = 'black'
                        self.allResults[self.WormNumber].append(1)
                    elif pause == True:                                         # if only the speed changed a blue box is put into the assay window
                        boxColor = 'blue'
                        self.allResults[self.WormNumber].append(0)
#                    elif angChange == True:                                         # if only the angle changed a green box is put into the assay window
#                        boxColor = 'green'
#                        self.allResults[self.WormNumber].append(1)
                    else:                                                           # if neither the speed nor the angle changed a white box is put into the assay window
                        boxColor = 'white'
                        self.allResults[self.WormNumber].append(0)
                    BoxX, BoxY = 32+(15*self.PulseNumber), 22+(25*self.WormNumber)
                    self.touchAssay.create_rectangle(BoxX, BoxY, BoxX+10, BoxY+20, fill=boxColor) 
                    #self.ab = 1                
                    self.PulseNumber +=1
 #                   cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)                
                    self.out.write(imagetoSave[:,:,0])
                    self.ExtraFrames = 0                  
                    self.out.release()             
                    self.saveIndividualResults()                                    # the results for this worm are saved    
                    self.individualResult = []    
                    self.PosWormX, self.PosWormY = [],[]
                    self.PosStageX, self.PosStageY = [],[]
                    self.times = []
                    if self.protocol[3]*1000>=((self.numofFramesperPuls/10)*1000):
                        intervalWaitTime = int(self.protocol[3]*1000 -((self.numofFramesperPuls/10)*1000))
                    else:
                        intervalWaitTime = 100*self.matchWormFly
                    self.videoDisplay.after(intervalWaitTime, self.start_Rec)
                elif self.frameNumber == (int)(((self.ab*(self.numofFramesperPuls-1)/self.numberofChecks)+1)+(self.numofFramesperPuls*(self.PulseNumber))):
    #---------------at the end of each 7th (starting from 2nd) of the recording-----#
    #---------------the program checks whether the worm turned - -------------------#
    #---------------if it did, the recording is started new-------------------------#
                    if self.ab<2:
                        self.ab += 1
#                        cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)
                        self.out.write(imagetoSave[:,:,0])
                        waitTime = int((100*self.matchWormFly)-np.round(time.time()-self.timeFirstFrame-self.timeActualFrame,3)*1000)
                        self.videoDisplay.after(waitTime, self.start_Rec)                    
                    elif self.ab<=self.USPulseFifth:
    #-------------------finds out whether any of the relevant positions of the worm-#
    #-------------------could not be estimated--------------------------------------#
                        if self.PosWormX[(int)(((self.ab-1)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1] =='NaN' or self.PosWormX[(int)(((self.ab-2)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1] =='NaN' or self.PosWormX[(int)(((self.ab)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1] == 'NaN':
    #-----------------------start all over again with this trial--------------------#                         
                            self.wormsTrace.delete(self.line[self.PulseNumber][:])   
                            del self.line[self.PulseNumber]
                            self.ab = 1
                            self.frameNumber = self.numofFramesperPuls*(self.PulseNumber)
                            self.individualResult = []
                            self.PosWormX, self.PosWormY = [],[]
                            self.PosStageX, self.PosStageY = [],[]
                            self.times = []
                            self.out.release()
                            self.videoDisplay.after(100*self.matchWormFly, self.start_Rec)
                        else:
                            vector1 = [self.PosWormX[(int)(((self.ab-1)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]-self.PosWormX[(int)(((self.ab-2)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1],self.PosWormY[(int)(((self.ab-1)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]-self.PosWormY[(int)(((self.ab-2)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]]
                            vector2 = [self.PosWormX[(int)(((self.ab)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]-self.PosWormX[(int)(((self.ab-1)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1],self.PosWormY[(int)(((self.ab)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]-self.PosWormY[(int)(((self.ab-1)*(self.numofFramesperPuls-1)/self.numberofChecks)+1)-1]]
                            if np.absolute(self.calculateAngle(vector1,vector2))<=1/2*np.pi:
    #-----------------------at the end of each 7th of the recording (starting with--#
    #-----------------------the 2nd) this checks whether the worm made a turn-------#
    #-----------------------(the velocity vector changed by more than 1/2*pi)-------#
    #-----------------------if not the recording goes on----------------------------#
                                self.ab += 1
                                if self.frameNumber == (int)(((self.USPulseFifth*(self.numofFramesperPuls-1)/self.numberofChecks)+1)+(self.numofFramesperPuls*(self.PulseNumber))):
    #---------------------------The ultrasound stimulus is initiated----------------#
                                    cv2.circle(imagetoSave, (320, 240), 31, color = ( 1, 1, 1 ), thickness = 5)
                                    currentFrame=(self.frameNumber-(self.PulseNumber*self.numofFramesperPuls))
                                    self.wormsTrace.create_oval(475+(self.PosWormX[currentFrame-1])*30, 355+(self.PosWormY[currentFrame-1])*30, 485+(self.PosWormX[currentFrame-1])*30, 365+(self.PosWormY[currentFrame-1])*30, outline='black', width=2)
                                    self.US_puls()   
              #                  else:
#                                    cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)
                                self.out.write(imagetoSave[:,:,0])
                                waitTime = int((100*self.matchWormFly)-np.round(time.time()-self.timeFirstFrame-self.timeActualFrame,3)*1000)
                                self.videoDisplay.after(waitTime, self.start_Rec)
                            else: 
    #-----------------------start all over again with this worm---------------------#                         
                                self.wormsTrace.delete(self.line[self.PulseNumber][:])
                                del self.line[self.PulseNumber]
                                self.ab = 1
                                self.frameNumber = self.numofFramesperPuls*(self.PulseNumber)
                                self.individualResult = []
                                self.PosWormX, self.PosWormY = [],[]
                                self.PosStageX, self.PosStageY = [],[]
                                self.times = []
                                self.out.release()
                                self.videoDisplay.after(100*self.matchWormFly, self.start_Rec)
                    else:
                        self.ab += 1
   #                     cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)
                        self.out.write(imagetoSave[:,:,0])
                        waitTime = int((100*self.matchWormFly)-np.round(time.time()-self.timeFirstFrame-self.timeActualFrame,3)*1000)
                        self.videoDisplay.after(waitTime, self.start_Rec)                                 
                else:
    #---------------In any other case the program waits until taking the next image-#
   #                 cv2.circle(imagetoSave, (320, 240), 31, color = ( 0.2, 1, 1 ), thickness = 2)
                    self.out.write(imagetoSave[:,:,0])
                    waitTime = int((100*self.matchWormFly)-np.round(time.time()-self.timeFirstFrame-self.timeActualFrame,3)*1000)
                    self.videoDisplay.after(waitTime, self.start_Rec)

        
    def calculateAngle(self,v1,v2):
        '''Calculates the clockwise angle between 2 vectors
           input: two 2D-vectors each as a list
           output: the clockwise angle between the two vectors in rad
        '''
        if (v1[0]*v2[1]-v1[1]*v2[0])<0:                                         # if the determine of the two vectors is smaller than 0 the angle is between 
            return np.arccos(np.clip(np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)), -1.0, 1.0))                    
        else:
            return -np.arccos(np.clip(np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)), -1.0, 1.0))
    '''                        
    def analyzeWormMovment(self):
        'Analyzes the worms movement based on 8 points (A,B,C,D,E,F,G,H):
           calculates the speed of the worm between point B&C, C&D, D&E, E&F, F&G, G&H            
           calculates the angle between (B-A)&(C-B), (C-B)&(D-C), (D-C)&(E-D), (E-D)&(F-E), (F-E)&(G-F), (G-F)&(G-H)
           output: booleans of change in speed, change in angle
        '
        j=0
        points, times, vectors, speeds, angles = [], [], [], [], []             # the points and times that the vectors, speed and angles are calculated from
        for i in range (0,8):
            if self.PosWormX[int(((self.numofFramesperPuls-1)/self.numberofChecks)*i)] != 'NaN':
                points.append([self.PosWormX[int(((self.numofFramesperPuls-1)/self.numberofChecks)*i)],
                                             self.PosWormY[int(((self.numofFramesperPuls-1)/self.numberofChecks)*i)]])
                times.append(self.individualResult[int(((self.numofFramesperPuls-1)/self.numberofChecks)*i)][0])
            else:
                for j in range (0,11):
                    if self.PosWormX[int((((self.numofFramesperPuls-1)/self.numberofChecks)*i)-j)] != 'NaN':
                        points.append([self.PosWormX[int((((self.numofFramesperPuls-1)/self.numberofChecks)*i)-j)],
                                                     self.PosWormY[int((((self.numofFramesperPuls-1)/self.numberofChecks)*i)-j)]])
                        times.append(self.individualResult[int((((self.numofFramesperPuls-1)/self.numberofChecks)*i)-j)][0])
                        break                   
            if i == 0:
                vectors.append(points[0])
            else:
                vectors.append(np.subtract(points[i],points[i-1]))
                if i >= 2:
                    speeds.append((np.sqrt((vectors[i][0])**2+(vectors[i][1])**2))/(times[i]-times[i-1]))
                    angles.append(self.calculateAngle(vectors[i],vectors[i-1]))
                if i == self.USPulseFifth-1:
#                    avgSpeed = np.average(speeds)
                    stdSpeed = st.t.interval(0.95, len(speeds)-1, loc=np.mean(speeds), scale=st.sem(speeds))
#                    avgAngle = np.average(angles)
#                    stdAngle = np.std(angles)
                elif i == self.USPulseFifth:
                    if stdSpeed[1]<=(speeds[i-2]):
                        SpeedUp = True
                        Pause = False
                    elif (speeds[i-2])<=stdSpeed[0]:
                        SpeedUp = False
                        Pause = True
                    else:
                        SpeedUp = False
                        Pause = False
                    if speeds[i-2]>=0.1/(2*self.matchWormFly):
                        if np.absolute(angles[i-2])>=1/2*np.pi:
                             angChanged = True
                        else:
                            angChanged = False
                    else:
                        angChanged = False   
                elif i >= self.USPulseFifth+1:
                    angles[i-2]=(int(self.calculateAngle(vectors[i],vectors[i-2-j])*100)/100)
                    if stdSpeed[1]<=(speeds[i-2]):
                        SpeedUp = True
                    elif (speeds[i-2])<=stdSpeed[0]:
                        Pause = True                    
                    if speeds[i-2]>=0.1/(2*self.matchWormFly):
                        if np.absolute(angles[i-2])>=1/2*np.pi:
                             angChanged = True
                    j+=1    
        print(speeds, angles)
        return SpeedUp, Pause, angChanged
    '''

    def displayWormTrace(self):
        '''Displays the velocity trace of the current recording in a canvas
        '''
#        colors = ['#a93226','#1f618d','#148f77','#d4ac0d','#909497','#d35400','#7d3c98','#2e4053','#1e8449','#5dade2']
        currentFrame=(self.frameNumber-(self.PulseNumber*self.numofFramesperPuls))      
        self.line[self.PulseNumber].append(self.wormsTrace.create_line(480+(self.PosWormX[currentFrame-2])*30, 360+(self.PosWormY[currentFrame-2])*30, 480+(self.PosWormX[currentFrame-1])*30, 360+(self.PosWormY[currentFrame-1])*30, fill =self.tkcolor, width = 2))
        
    def saveIndividualResults(self):
        '''Saves the results of one recording
        '''
        resultName = 'w'+'{0:0=3d}'.format(self.WormNumber)+'p'+'{0:0=2d}'.format(self.PulseNumber)+'.txt'
        text = 'Time [s], Stage x-position, Stage y-position, Worm x-position, Worm y-position \n'
        with open(self.filedir+resultName, "a") as f:
                f.write(text)        
        for i in range (0,self.numofFramesperPuls):
            text = str(self.individualResult[i][0]) + ", " + str(self.individualResult[i][1]) + ", " + str(self.individualResult[i][2]) + ", " + str(self.individualResult[i][3]) + ", " + str(self.individualResult[i][4]) + "\n"
            with open(self.filedir+resultName, "a") as f:
                    f.write(text)
        
    def saveResults(self):
        '''Saves the results of all recordings in a file
        '''
        if len(self.allResults)>=1 and self.allResults!=[[]]:
            text = ''
            for i in range (0,len(self.allResults)):
                if self.allResults[i] != []:
                    text += 'worm ' + str(i) + ', '
                    for j in range (0,len(self.allResults[i])-1):
                        text += str(self.allResults[i][j]) + ', '
                    text += str(self.allResults[i][len(self.allResults[i])-1]) + '\n'
            try:
                with open(self.filedir+'allresults.txt', "a") as f:
                    f.write(text)
            except:
                pass

    def centerStage(self):
        '''Centers the stage
        '''
        x_pos = get_position_t()
        result = lib.get_position(self.device_id[0], byref(x_pos))
        yPos = repr(x_pos.Position)
        result2 = lib.get_position(self.device_id[1], byref(x_pos))
        xPos = repr(x_pos.Position)         
        lib.command_movr(self.device_id[1], -int(xPos), 0)
        time.sleep(0.5)                              
        lib.command_movr(self.device_id[0], -int(yPos), 0)
        time.sleep(0.5)             

    def testFunctionTracking(self):
        '''pretends a worm is at a position testWormPos and tests the programs tracking
        '''
        
        
    def onClose(self):
        '''Saves the results, sets the stop event, cleans up the camera,
           and quits process
        '''
        self.centerStage()                                                      # centers the stage
        self.saveResults()                                                      # saves results automatically so no results are lost
        self.stopEvent.set()                                                    # sets the stop event for the video loop
        self.cap.release()                                                      # releases the captured images
        self.root.quit()                                                        # quits the root window
        lib.close_device(byref(cast(self.device_id[0], POINTER(c_int))))        # releases the stage
        lib.close_device(byref(cast(self.device_id[1], POINTER(c_int))))        # releases the stage
        
#Starts the app (class)
pba = USPokingTracker()
pba.root.mainloop()
pba.root.destroy()