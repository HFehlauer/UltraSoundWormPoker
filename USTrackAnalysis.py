# -*- coding: utf-8 -*-
"""
Created on Wed Nov 22 16:19:42 2017

@author: Holger
"""

import numpy as np
import math
import scipy.stats as st

class TrackAnalysis():
    '''Class to analyze individual tracks of the US tracker
    '''

    def __init__(self):
        '''Initialization if needed
        '''
        
    def calculateAngle(self,v1,v2):
        '''Calculates the clockwise angle between 2 vectors
           input: two 2D-vectors each as a list
           output: the clockwise angle between the two vectors in rad
        '''
        if (v1[0]*v2[1]-v1[1]*v2[0])<0:                                         # if the determine of the two vectors is smaller than 0 the angle is between 
            return np.arccos(np.clip(np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)), -1.0, 1.0))                    
        else:
            return -np.arccos(np.clip(np.dot(v1/np.linalg.norm(v1), v2/np.linalg.norm(v2)), -1.0, 1.0)) 
            
    def analyzeTrack(self, speedThresh, USPulseFrac, Times, PosX, PosY):
        '''Analyzes the worms movement based on 8 points (A,B,C,D,E,F,G,H):
           calculates the speed of the worm between point B&C, C&D, D&E, E&F, F&G, G&H            
           calculates the angle between (B-A)&(C-B), (C-B)&(D-C), (D-C)&(E-D), (E-D)&(F-E), (F-E)&(G-F), (G-F)&(G-H)
           output: booleans of change in speed, change in angle
        '''
        points, times, vectors, speeds, angles = [], [], [], [], []             # the points and times that the vectors, speed and angles are calculated from
        j=0                                                               
        for i in range (0,8):
            if math.isnan(PosX[10*i]):
                correctJ = 1
                while correctJ < 9:
                    if math.isnan(PosX[10*i-correctJ]): 
                        correctJ +=1
                    else:
                        points.append([(PosX[10*i-correctJ]+PosX[10*i+correctJ])/2,(PosY[10*i+correctJ]+PosY[10*i+correctJ])/2])  
                        break           
            else:
                points.append([PosX[10*i],PosY[10*i]])
            times.append(Times[10*i])
            if i == 0:
                vectors.append(points[0])
            else:
                vectors.append(np.subtract(points[i],points[i-1]))
                if i >= 2:
                    speeds.append((int((np.sqrt((vectors[i][0])**2+(vectors[i][1])**2))/(times[i]-times[i-1])*100)/100))
                    if vectors[i][0]==0 and vectors[i][1]==0 or vectors[i-1][0]==0 and vectors[i-1][1]==0 :
                        angles.append(0)
                    else:
                        angles.append((int(self.calculateAngle(vectors[i],vectors[i-1])*100)/100))           
                if i == (USPulseFrac-1):
          #          avgSpeed = np.average(speeds)
          #          stdSpeed = np.std(speeds)
          #          avgAngle = np.average(angles)
          #          stdAngle = np.std(angles)
                    stdSpeed = st.t.interval(0.95, len(speeds)-1, loc=np.mean(speeds), scale=st.sem(speeds))
                elif i == USPulseFrac:              
                    if stdSpeed[1]<=(speeds[i-2]):
                        SpeedUp = True
                        Pause = False
                    elif (speeds[i-2])<=stdSpeed[0]:
                        SpeedUp = False
                        Pause = True
                    else:
                        SpeedUp = False
                        Pause = False
                    if speeds[i-2]>speedThresh:
                        if np.absolute(angles[i-2])>=1/2*np.pi:
                             angChanged = True
                        else:
                            angChanged = False
                    else:
                        angChanged = False                     
                elif i >= USPulseFrac+1:  
                    if vectors[i][0]==0 and vectors[i][1]==0 or vectors[i-2-j][0]==0 and vectors[i-2-j][1]==0 :
                        angles.append(0)
                    else:                
                        angles[i-2]=(int(self.calculateAngle(vectors[i],vectors[i-2-j])*100)/100)
                    if stdSpeed[1]<=(speeds[i-2]):
                        SpeedUp = True
                    elif (speeds[i-2])<=stdSpeed[0]:
                        Pause = True                    
                    if speeds[i-2]>=speedThresh:                     
                        if np.absolute(angles[i-2])>=1/2*np.pi:
                             angChanged = True
                    j+=1    
        return SpeedUp, Pause, angChanged
    
    
