# -*- coding: utf-8 -*-
"""
Created on Mon May  1 16:55:38 2017

@author: Holger
"""

'''todo:
   track the head or whatever is the most outer part in the direction of movement
   track the direction of movement
   control the stage to follow a worm 
   handle more than one worm
   apply US
   two colomns for metadata incubation - experiment
   implement on rPI
   control LEDs ?
   control Arduino ?
'''
from sklearn.preprocessing import normalize
import skimage.morphology as skimor
import numpy as np
import cv2
import tkinter as tk
from PIL import Image, ImageTk
import threading
from datetime import datetime
import getpass
import os
from time import time
import time
import sys
if sys.version_info >= (3,0):
    import urllib.parse
sys.path.insert(0, 'C:/Standa controller')
try:
    from pyximc import *
except ImportError as err:    
    print ("Can't import pyximc module. The most probable reason is that you haven't copied pyximc.py to the working directory. See developers' documentation for details.")
    exit()
except OSError as err:
    print ("Can't load libximc library. Please add all shared libraries to the appropriate places (next to pyximc.py on Windows). It is decribed in detail in developers' documentation. On Linux make sure you installed libximc-dev package.")
    exit()
    
class USPokingTracker:
    def __init__(self):
        '''Initilizes the USPokingTracker class
        the most recently read frame, thread for reading frames, and
        the thread stop event
        '''
        self.cap =cv2.VideoCapture(1)
        self.frame = None
        self.thread = None
        self.stopEvent = None
        self.stateProtocol = False
        self.filedir = 'C:/Users/'+getpass.getuser()+'/Desktop/'+datetime.today().strftime("%Y%m%d")+'/'            
        if not os.path.exists(self.filedir):
            os.makedirs(self.filedir)
        self.fourcc = cv2.VideoWriter_fourcc(*'DIB ')
        self.root = tk.Tk()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        if screen_width >= 800 and screen_height >= 640:
            self.root.geometry("%dx%d+0+0" % (800, 600))
        else:
            self.root.geometry("%dx%d+0+0" % (800, 600))
        self.panel = None
        # parameter for US stimulus
        self.InterStimulusInterval = 10
        self.PulseLength = 2
        self.PulseFrequency = 1000
        self.PulseOn = False
        self.standardProtocol = [10,2,1000,False]
        # parameters for the video and the image analysis
        self.numofFrames = 999
        self.PositionWorm = []
        self.wormVelocities = []
        self.conversionPixeltoUM = 5.6*12
        self.WormNumber = 0
        self.RecNumber = 0
        self.PosWormX, self.PosWormY = [],[]
        self.PosStageX, self.PosStageY = [],[]
        devenum = lib.enumerate_devices(EnumerateFlags.ENUMERATE_PROBE, None)
        self.device_id=[]
        self.device_id.append(lib.open_device(lib.get_device_name(devenum,0)))
        self.device_id.append(lib.open_device(lib.get_device_name(devenum,1)))
        self.timeLastFrame, self.timeActualFrame = 0,0
        # buttons and canvas and other tkinter stuff
        self.traceWindow = None
        tk.Label(self.root, height=2, width = 100, text="Files are safed to: "+self.filedir).grid(row=0, column=0)             
        tk.Button(self.root, text="Metadata", command=self.change_Metadata).grid(row=0, column=1)
        v = 0
        tk.Radiobutton(self.root, text="10 touch assay", variable=v, value=0, command=self.tenTouchChosen).grid(row=1, column=0) 
        tk.Radiobutton(self.root, text="Protocol", variable=v, value=1, command=self.protocolChosen).grid(row=1, column=1)
        self.changeProtocol = tk.Button(self.root, text="Protocol",command=self.change_Protocol, state='disabled')
        self.changeProtocol.grid(row=2, column=1)              
        tk.Button(self.root, text="Quit",command=self.onClose).grid(row=4, column=1)
        self.RecButton = tk.Button(self.root, text="Start rec.", state='disabled', command = self.start_Rec)
        self.RecButton.grid(row=4, column=0)
        self.stopEvent = threading.Event()
        self.thread = threading.Thread(target=self.videoLoop, args=())
        self.thread.start()
        self.root.wm_title("USPokingTracker")
        self.root.wm_protocol("WM_DELETE_WINDOW", self.onClose)

    def findThreshold(self):
        ''' evaluates a threhold similar to the Otsu method:
                - splits the histogram in two parts
                - minimizes the sum of the variance of the two parts 
            input is an 8-bit bw image
            output is an 8-bit value - the threshold
        '''
        hist = cv2.calcHist([self.frame[:,:,0]],[0],None,[256],[0,256])                        # finds histogram
        hist_norm = hist.ravel()/hist.max()                                        # normalizes histogram
        Q = hist_norm.cumsum()                                                     # cumulates hisotgram
        bins = np.arange(256)                                                      # bins
        fn_min = np.inf                                                            # the minimum of the variance of the two histogram parts 
        thresh = -1                                                                # the threshold (id the same as with Otsu function)
        for i in range(1,256):                                                     # i is the value for splitting the histogram
            p1,p2 = np.hsplit(hist_norm,[i])                                       # the probabilities
            q1,q2 = Q[i],Q[255]-Q[i]                                               # cumulative sum of classes
            if q1 != 0 and q2 != 0:                                                # only evaluates the threshold if both parts of the histogram include values
                b1,b2 = np.hsplit(bins,[i])                                        # the weights
                m1,m2 = np.sum(p1*b1)/q1, np.sum(p2*b2)/q2                         # the means
                v1,v2 = np.sum(((b1-m1)**2)*p1)/q1,np.sum(((b2-m2)**2)*p2)/q2      # the variances
                fn = v1*q1 + v2*q2                                                 # calculates the minimization function
                if fn < fn_min:                                                    # if the minimum is smaller than any minimum before a new way to split the histogram was found
                    fn_min = fn
                    thresh = i
        return thresh+int(thresh/10)                                                # subtracts 1/5 from the Otsu threshold, this is hard-coded to compensate for the pipette intensity

    def videoLoop(self):
        '''Loops through the recording of a video and displays the newest image
           Also: image processing:
               - deletes the area far away from worm
        '''
        try:
            while not self.stopEvent.is_set():
                ret, self.frame = self.cap.read()
                image = self.frame#[:,:,2]
                displayedImage = self.frame[:,:,0]
                #Check whether there is a worm in the FOV
                #If yes set the StartRec Button to normal
                thresh = self.findThreshold()
                ret1, img1 = cv2.threshold(image[:,:,0],thresh,255,cv2.THRESH_BINARY)
                if self.stateProtocol == True:
                    img1 = cv2.dilate(img1, None, iterations=5)                           # dilates the frame 10 times                
                    img1 = cv2.erode(img1, None, iterations=5) 
                    img2 = skimor.remove_small_holes(np.divide(img1,255).astype(bool), min_size=50, connectivity=1, in_place=False)
                    img3 = skimor.remove_small_objects(img2, min_size=50, connectivity=1, in_place=False)
                    imgcnt = img3.astype(int)
                    contours = None
                    try:
                       im,contours,hierarchy = cv2.findContours(imgcnt,cv2.RETR_FLOODFILL, cv2.CHAIN_APPROX_SIMPLE)
                    except:
                        pass
                    self.ellip = None
#                    mask = None
                    if contours != None:
                        cMax = max(contours, key = cv2.contourArea)             # the largest contour areawise
                        try:
                            self.ellip = cv2.fitEllipse(cMax)
                            cv2.ellipse(image, (int(self.ellip[0][0]),int(self.ellip[0][1])),(int(self.ellip[1][0]/2),int(self.ellip[1][1]/2)),int(self.ellip[2]),0,360, color = (1,1,1), thickness = 5)
                            self.moveStage()
#                            mask = np.zeros(imgcnt.shape).astype(imgcnt.dtype)
#                            cv2.ellipse(mask,self.ellip,(255),-1)
#                            imgcnt = np.divide(cv2.bitwise_and(img3*255, mask),255).astype(bool)
#                            img4 = skimor.skeletonize_3d(imgcnt).astype(int)
#                            self.moveStage()
#                            displayedImage = (img3*255).astype(int) 
                        except:
                            displayedImage = (img3*255).astype(int)                                
                            pass
                    else:
                        displayedImage = (img3*255).astype(int)                        
                    if self.ellip!= None:
                        seeWorm = True
                        cv2.circle(image, (int(self.wormHead()[0]),int(self.wormHead()[1])), 31, color = ( 1, 1, 1 ), thickness = 2)   
#                        self.moveStage()
                    else:
                        seeWorm = False
                if self.stateProtocol == False:
                    self.RecButton.config(state="disabled")
                elif seeWorm == False:
                    self.RecButton.config(state="disabled")
                else:
                    self.RecButton.config(state="normal")
                #Display the image in the GUI
#               image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB);
                image = Image.fromarray(image)
                image = ImageTk.PhotoImage(image)
                if self.panel is None:
                    self.panel = tk.Label(image=image)
                    self.panel.image = image
                    self.panel.grid(row=3)  	
                else:
                    self.panel.configure(image=image)
                    self.panel.image = image
        except RuntimeError:
            print()

    def tenTouchChosen(self):
        '''Trigger if standard protocol is chosen
        '''     
        self.changeProtocol.config(state="disabled")
        self.stateProtocol = True
        self.protocol = self.standardProtocol
        
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
            elif i == 2:
                text = self.PNames[i] + ": " + self.Pentry2.get() + "\n"
            elif i == 3:
                text = self.PNames[i] + ": " + self.Pentry3.get() + "\n"
            elif i == 4:    
                text = self.PNames[i] + ": " + self.Pentry4.get() + "\n"
            elif i == 5:                
                text = self.PNames[i] + ": " + self.Pentry5.get() + "\n" + "\n"             
            with open(self.filedir+"protocol.txt", "a") as f:
                f.write(text)
        self.stateProtocol = True
        self.setProtocol.destroy()
      
    def change_Protocol(self):
        '''Changes the protocol for the US stimulus if custom protocol is chosen
        grabs individual US parameters, saves them into a .txt file
        '''
        # to be written: load a presaved protocol
        self.setProtocol = tk.Toplevel()
        self.setProtocol.wm_title("Protocol")
        self.PNames = []
        heightText = 2
        widthText = 20
        r = 6
        for i in range(0,r):
            if i == 0:            
                self.PNames.append('Date, Time')
                self.Pentry = datetime.now().strftime('%Y/%m/%d, %H:%M:%S\n')                
            tk.Label(self.setProtocol, height=heightText, width = widthText, text=self.Pentry).grid(row=0, column=1)                    
            if i == 1:
                self.PNames.append('Frequency [Hz]')
                self.Pentry1 = tk.Entry(self.setProtocol)
                self.Pentry1.grid(row=i, column=1)
            elif i == 2:
                self.PNames.append('Amplitude')
                self.Pentry2 = tk.Entry(self.setProtocol)
                self.Pentry2.grid(row=i, column=1)
            elif i == 3:
                self.PNames.append('whatever else')
                self.Pentry3 = tk.Entry(self.setProtocol)
                self.Pentry3.grid(row=i, column=1)
            elif i == 4:
                self.PNames.append('Protocol name')
                self.Pentry4 = tk.Entry(self.setProtocol)
                self.Pentry4.grid(row=i, column=1)                
            elif i == 5:
                self.PNames.append('Comments')
                self.Pentry5 = tk.Entry(self.setProtocol)
                self.Pentry5.grid(row=i, column=1)
            tk.Label(self.setProtocol, height=heightText, width = widthText, text=self.PNames[i]).grid(row=i, column=0)   
        tk.Button(self.setProtocol, text="Save & Quit", command=self.saveProtocol).grid(row=r+1, column=1)   
        
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
                text = self.Names[i] + ": " + self.entry5.get() + "\n" + "\n"             
            with open(self.filedir+"metadata.txt", "a") as f:
                f.write(text)
        self.metaData.destroy()
                
    def change_Metadata(self):
        '''Changes the metadata
        '''
        self.Names = []
        heightText = 2
        widthText = 20
        self.metaData = tk.Toplevel()
        self.metaData.wm_title("Metadata")
        r = 7
        for i in range(0,r):
            if i == 0:            
                self.Names.append('Date, Time')
                self.entry = datetime.now().strftime('%Y/%m/%d, %H:%M:%S\n')                
            tk.Label(self.metaData, height=heightText, width = widthText, text=self.entry).grid(row=0, column=1)                    
            if i == 1:
                self.Names.append('Temperature [°C]')
                self.entry1 = tk.Entry(self.metaData)
                self.entry1.grid(row=i, column=1)
            elif i == 2:
                self.Names.append('Humidity [%]')
                self.entry2 = tk.Entry(self.metaData)
                self.entry2.grid(row=i, column=1)
            elif i == 3:
                self.Names.append('Worm strain')
                self.entry3 = tk.Entry(self.metaData)
                self.entry3.grid(row=i, column=1)
            elif i == 4:
                self.Names.append('Protocol')
                self.entry4 = tk.Entry(self.metaData)
                self.entry4.grid(row=i, column=1)
            elif i == 5:
                self.Names.append('Added drugs?')
                self.entry4 = tk.Entry(self.metaData)
                self.entry4.grid(row=i, column=1)
                                
            elif i == 6:
                self.Names.append('Comments')
                self.entry5 = tk.Entry(self.metaData)
                self.entry5.grid(row=i, column=1)
            tk.Label(self.metaData, height=heightText, width = widthText, text=self.Names[i]).grid(row=i, column=0)     
        tk.Button(self.metaData, text="Save & Quit", command=self.saveMetaData).grid(row=r+1, column=1)           

    def US_puls(self):
        '''Applies an US puls, according to the next step in the chosen protocol
        '''
        #to be written, where do I keep track of the protocol
        self.InterStimulusInterval = self.standardProtocol[0]
        self.PulseLength = self.standardProtocol[1]
        self.PulseFrequency = self.standardProtocol[2]
        self.PulseOn = self.standardProtocol[3]       
        
    def displayWormTrace(self):
        '''Displays the veolcity trace of the current recording in a canvas
        '''
        Vecnew = (self.PosWormX[self.RecNumber]-self.PosWormX[self.RecNumber-1], self.PosWormY[self.RecNumber]-self.PosWormY[self.RecNumber-1])
        VelNew = np.sqrt((self.PosWormX[self.RecNumber]-self.PosWormX[self.RecNumber-1])**2+(self.PosWormY[self.RecNumber]-self.PosWormY[self.RecNumber-1])**2)
        Vecold1 = (self.PosWormX[self.RecNumber-1]-self.PosWormX[self.RecNumber-2], self.PosWormY[self.RecNumber-1]-self.PosWormY[self.RecNumber-2])
        VelOld = np.sqrt((self.PosWormX[self.RecNumber-1]-self.PosWormX[self.RecNumber-2])**2+(self.PosWormY[self.RecNumber-1]-self.PosWormY[self.RecNumber-2])**2)
        Vecold2 = (self.PosWormX[self.RecNumber-2]-self.PosWormX[self.RecNumber-3], self.PosWormY[self.RecNumber-2]-self.PosWormY[self.RecNumber-3])
        self.Anglenew = np.arccos(np.clip(np.dot(Vecnew/np.linalg.norm(Vecnew), Vecold1/np.linalg.norm(Vecold1)), -1.0, 1.0))
        self.Angleold = np.arccos(np.clip(np.dot(Vecold1/np.linalg.norm(Vecold1), Vecold2/np.linalg.norm(Vecold2)), -1.0, 1.0))
        self.w.create_line(self.timeLastFrame, VelOld, self.timeActualFrame, VelNew, fill = 'mediumslateblue', width = 5)
#        print(self.timeLastFrame, VelOld, self.timeActualFrame, VelNew)
#        self.w.create_line(self.PosWormX[self.RecNumber], self.PosWormY[self.RecNumber], self.PosWormX[self.RecNumber-1], self.PosWormY[self.RecNumber-1], fill = 'mediumslateblue', width = 5)
#        self.w.pack()

    def manualMoveStage_updown(self):  
        '''Moves the stage up and down
        '''
        #needs to be written                           
        lib.command_movr(self.device_id[0], -int((self.ellip[0][1]-240)*conFactor), 0)   # ist zur Zeit y movement
        time.sleep(0.5)
        x_pos = get_position_t()
        result = lib.get_position(self.device_id[0], byref(x_pos))
        yPos = repr(x_pos.Position)     


    def manualMoveStage_rightleft(self):
        '''Moves the stage left and right
        '''
        #needs to be written
        lib.command_movr(self.device_id[1], int((self.ellip[0][0]-320)*conFactor), 0)
        time.sleep(0.5)                              
        x_pos = get_position_t()
        result2 = lib.get_position(self.device_id[1], byref(x_pos))
        xPos = repr(x_pos.Position)         

    def wormVelocityCalc(self):
        '''here the last ten positions of the worms are stored
           and the worm's normalized velocity of these ten is calculated
        '''     
        self.PositionWorm.append([self.ellip[0][0],self.ellip[0][1]])
        if len(self.PositionWorm)==3:
            del self.PositionWorm[0]
            self.wormVelocities.append([self.PositionWorm[1][0]-self.PositionWorm[0][0],self.PositionWorm[1][1]-self.PositionWorm[0][1]])
            if len(self.wormVelocities)==10:
                del self.wormVelocities[0]
            self.wormVelocity = np.mean(self.wormVelocities, axis=0)
#            print(self.wormVelocity)
            self.wormVelocity=self.wormVelocity.reshape(1, -1)
            self.normWormVelocity = [normalize(self.wormVelocity, axis=0, norm='l2')[0][0], normalize(self.wormVelocity, axis=0, norm='l2')[0][1]]
        else:
            self.normWormVelocity = [self.ellip[0][0],self.ellip[0][1]]

    def wormHead(self):
        '''decides which end of the worm is the one to track, 
           based on the direction of movement
        '''
        pos1 = [np.sin(np.pi*self.ellip[2]/180), -np.cos(np.pi*self.ellip[2]/180)]
        pos2 = [-np.sin(np.pi*self.ellip[2]/180), np.cos(np.pi*self.ellip[2]/180)]
#        print(pos1, pos2, self.normWormVelocity,np.dot(self.normWormVelocity,pos1),np.dot(self.normWormVelocity,pos2))
        if np.dot(self.normWormVelocity,pos1)>=np.dot(self.normWormVelocity,pos2):
            return self.ellip[0][0]+((pos1[0]*self.ellip[1][1])/2), self.ellip[0][1]+((pos1[1]*self.ellip[1][1])/2)
        else:
            return self.ellip[0][0]+((pos2[0]*self.ellip[1][1])/2), self.ellip[0][1]+((pos2[1]*self.ellip[1][1])/2)

    def moveStage(self):
        '''Moves the stage
        '''
        # to be written
        conFactor = 0.2
        self.wormVelocityCalc()
        targetPos = self.wormHead()
        print(self.wormHead()[0],self.wormHead()[1])
        lib.command_movr(self.device_id[1], int((targetPos[0]-320)*conFactor), 0)
        if len(self.wormVelocities)<=3: 
            time.sleep(0.2)                              
        lib.command_movr(self.device_id[0], -int((targetPos[1]-240)*conFactor), 0)   # ist zur Zeit y movement
        if len(self.wormVelocities)<=3: 
            time.sleep(0.2)     
        x_pos = get_position_t()
        result = lib.get_position(self.device_id[0], byref(x_pos))
        yPos = repr(x_pos.Position)
        result2 = lib.get_position(self.device_id[1], byref(x_pos))
        xPos = repr(x_pos.Position) 
#        print(xPos, yPos)
#        if result == Result.Ok:
#            print("Position: " + repr(x_pos.Position))        
#        self.PosStageX.append(self.PosStageX[self.RecNumber]+self.PosWormX[self.RecNumber]-320) 
#        self.PosStageY.append(self.PosStageY[self.RecNumber]+self.PosWormY[self.RecNumber]-240)
        
    def start_Rec(self):
        '''Starts the recording of a video:
        records the time of the taken images, estimates position of the worms
        opens a canvas to show the velocity vs time, moves the stage according to worm position and
        saves the individual results before destroying all involvoed processes
        ''' 
        ret, frame = self.cap.read()
        if self.RecNumber>=1:
            self.timeActualFrame = time.time()-self.timeFirstFrame            
        else:
            self.timeActualFrame = 0
            self.timeFirstFrame = time.time()
            self.out = cv2.VideoWriter(self.filedir+'output.avi',self.fourcc, 20.0, (640,480), False)             
            self.traceWindow = tk.Toplevel()
            self.traceWindow.wm_title("Worm Trace")
            self.PosStageX.append(0)
            self.PosStageY.append(0)
            self.w = tk.Canvas(self.traceWindow, width=640, height=480)
            self.NumbRec = tk.Label(self.traceWindow, height=2, width = 20, text=self.RecNumber)
            self.NumbRec.pack()
            self.w.pack()
        if self.RecNumber <= self.numofFrames:
            ret1, img1 = cv2.threshold(frame[:,:,0],220,255,cv2.THRESH_BINARY)
            img2 = skimor.remove_small_holes(np.divide(img1,255).astype(bool), min_size=50, connectivity=1, in_place=False)
            img3 = skimor.remove_small_objects(img2, min_size=50, connectivity=1, in_place=False)
            for j in range(0,3):
                img3 = skimor.binary_dilation(img3)
            imgcnt = img3.astype(int)
            try:
               im,contours,hierarchy = cv2.findContours(imgcnt,cv2.RETR_FLOODFILL, cv2.CHAIN_APPROX_SIMPLE)#cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            except:
                pass           
            img4 = skimor.skeletonize(img3).astype(int)*255
            ellips = None    
            if len(contours) >= 1:
                foundWorm = True
                for e in contours:        
                    try:
                        ellips = cv2.fitEllipse(e)
                        cv2.circle(frame[:,:,0], (int(self.ellip[0][0]),int(self.ellip[0][1])), 5, color = ( 1, 1, 1 ), thickness = 2)   
    #                    cv2.ellipse(img4,ellip,(255,255,0),2)
                    except:
                        pass
            else:
                foundWorm = False
            self.PosWormX.append(ellips[0][0])
            self.PosWormY.append(ellips[0][1])
            self.out.write(frame[:,:,0])            
            if self.RecNumber>=1:
                self.NumbRec.configure(text=self.RecNumber+1)
                self.displayWormTrace()
            self.timeLastFrame = self.timeActualFrame
            self.RecNumber += 1
            print(self.RecNumber)
            self.panel.after(20, self.start_Rec)      
        else:
            self.out.release()         
            self.saveIndividualResults()            
            self.PosWormX,self.PosWormY = [],[]
            self.PosStageX, self.PosStageY = [],[]
            self.WormNumber += 1
            self.RecNumber = 0            
            cv2.destroyAllWindows()
            
    def saveIndividualResults (self):
        '''Saves the results of one recording
        '''
        # to be written
        
    def show_AllResults(self):
        '''Shows all results that have been recorded so far
        '''
        # to be written, should include a button for saveResults()
        
    def saveResults(self):
        '''Saves the results of all recordings in a file
        '''
        # to be written

    def centerStage(self):
        '''Center the stage
        '''
        x_pos = get_position_t()
        result = lib.get_position(self.device_id[0], byref(x_pos))
        yPos = repr(x_pos.Position)
        result2 = lib.get_position(self.device_id[1], byref(x_pos))
        xPos = repr(x_pos.Position)         
        lib.command_movr(self.device_id[1], -int(xPos), 0)
        time.sleep(1)                              
        lib.command_movr(self.device_id[0], -int(yPos), 0)   # ist zur Zeit y movement
        time.sleep(1)        
        
    def onClose(self):
        '''Saves the results, sets the stop event, cleans up the camera,
        and quits process
        '''
        self.centerStage()
        self.saveResults()                  # saves results automatically so no results are lost
        self.stopEvent.set()                # sets the stop event for the video loop
        self.cap.release()                  # releases the captured images
        self.root.quit()                    # quits the root window
        lib.close_device(byref(cast(self.device_id[0], POINTER(c_int))))
        lib.close_device(byref(cast(self.device_id[1], POINTER(c_int))))
        
#Needs to run on rPI3
#Start the app (class)
pba = USPokingTracker()
pba.root.mainloop()
pba.root.destroy()