# -*- coding: utf-8 -*-
"""
Created on Tue Aug 29 15:18:18 2017

@author: Holger
"""


import numpy as np
from numpy import genfromtxt
import scipy.stats as st
from matplotlib import pyplot as pl
from matplotlib import gridspec
import seaborn as sns
import pandas as pd
import glob
import math
import matplotlib.patches as patches
from matplotlib import rcParams
import getpass
import USTrackAnalysis

sns.set_style("ticks")
sns.set_context("notebook", font_scale=1.4, rc={"lines.linewidth": 2.5})
   
rcParams['xtick.direction'] = 'out'
rcParams['ytick.direction'] = 'out'
rcParams['svg.fonttype'] = 'none'


def adjust_spines(ax, lay):
    if lay == 0:
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.spines['left'].set_position(('outward',5))
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks([])
    if lay == 1:
        ax.yaxis.set_ticks([])
        ax.xaxis.set_ticks([])
    if lay == 2:
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.yaxis.set_ticks_position('left')
        ax.xaxis.set_ticks_position('bottom')
        ax.spines['left'].set_position(('outward',5))
        ax.spines['bottom'].set_position(('outward',5))  
    if lay == 3:
        ax.spines['bottom'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['top'].set_visible(False)
        ax.spines['left'].set_visible(False)
        ax.yaxis.set_ticks([])  
        ax.xaxis.set_ticks([])        

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def calculateAngleClockwise(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))    

def plotAllTracksMiddlepoint(file, f, v):
    Times, PosX, PosY = getPositions(file)   
    PosX[:]-=PosX[40]
    PosY[:]-=PosY[40]
    if PosX[10]>=0:
        PosX = -PosX[:]
    m1 = np.cos(calculateAngleClockwise([PosX[10],PosY[10]],[0,-np.sqrt(PosX[10]**2+PosY[10]**2)]))
    m2 = np.sin(calculateAngleClockwise([PosX[10],PosY[10]],[0,-np.sqrt(PosX[10]**2+PosY[10]**2)]))
    PosX1 = np.subtract(m1*PosX,m2*PosY)+f*10
    PosY2 = np.add(m2*PosX,m1*PosY)
    if PosY2[0] >=0:
        PosY2=-PosY2
    ax = fig.add_subplot(gs[0])
    ax.plot(PosX1[10], PosY2[10] , 'o', color=(1,0.9,0.15), linewidth=1)          
    for i in range (10,71-1,1):           
        col = (1-((i-10)*256/(71-10)/256),0.9-((i-10)*230/(71-10)/256),(i-10)*200/(71-10)/256)            
        ax.plot(PosX1[i:i+1+1], PosY2[i:i+1+1] , 'k', color= col, linewidth=3)    
    ax.plot(PosX1[70], PosY2[70] , 'o', color=(0,0,0.75), linewidth=1)
    if v ==True:
        markercolor = 'black'
    else:
        markercolor = 'white'        
    ax.plot(PosX1[40], PosY2[40] , 'o', color=(0.5,0.5,0.5), linewidth=1, mec='black', mfc=markercolor, mew=1.5)
    '''
    ax.add_patch(
        patches.Rectangle(
            (f*10-3, 3),
            2,
            2,
            lw = 2,
            facecolor="#000000",
            edgecolor="#000000",
            fill=v      # remove background
        )
    )
    '''   
    adjust_spines(ax, 3)    

def getPositions(filename):
    myData = genfromtxt(filename, delimiter=',')
    Times = myData[1:,0]-myData[1,0]
    PosX = myData[1:,3]-myData[1,3]
    PosY = myData[1:,4]-myData[1,4]
    return Times,PosX, PosY

def areweRight(estimate,file):
    Times, PosX, PosY = getPositions(file)
    analyzed = UTA.analyzeTrack(0.05,4,Times, PosX, PosY)
#    analyzed = analyzeTrack(Times, PosX, PosY)
    answer = []
    for i in range (0,3):
        if estimate[i]==2 or estimate[i]==analyzed[i]:
            answer.append(True)
        else:
            answer.append(False)
    return answer

fig = pl.figure(figsize=(18, 9))
pl.clf()
gs = gridspec.GridSpec(1, 1) 
correct, false = 0, 0
estimate = [2,2,True]
UTA = USTrackAnalysis.TrackAnalysis()
f = 0    
end = 10
filedir = 'C:/Users/'+getpass.getuser()+'/OneDrive/Documents/python plots/wt/new/'     
for filename in glob.glob(filedir+'*.txt'):
    v = areweRight(estimate,filename)[2]
    if v == True:
        correct += 1
    else:
        false += 1
    plotAllTracksMiddlepoint(filename,f,v)    
    f += 1   
    if f == end:
        f += 3      
        end += 13
print(correct/(correct+false))
pl.show()