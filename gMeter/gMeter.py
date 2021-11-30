##############################################################
# Kunos Simulazioni
# AC Python tutorial 04 : Get data from AC
#
# To activate create a folder with the same name as this file
# in apps/python. Ex apps/python/tutorial01
# Then copy this file inside it and launch AC
#############################################################

import ac
import acsys
import math

appWindow=0
longitudinalGIndicator=0
lateralGIndicator=0
barLength=322
filter=0.2
maxG=2
triangleWidth=10
class GIndicator:
    def __init__(self,app,x,y,name):
         self.xPosition=x
         self.yPosition=y
         self.counter=0
         self.secondsToFade=3
         self.currentValue=0
         self.oldValue=0
         self.maxValue=0
		 
         ac.setPosition(ac.addLabel(appWindow,name),x,y)
         self.currentValueLabel=ac.addLabel(appWindow,"0.0g")
         ac.setPosition(self.currentValueLabel,x+50,y)
         self.indicatorWidth=10
         self.indicatorPosition=0
    
    def setCurrentValue(self,value):
        global maxG, barLength
        self.currentValue= min(max(value,-maxG),maxG)
		#filtering the values
        self.currentValue = self.oldValue*(filter) + self.currentValue*(1-filter)
        self.currentValue=round(self.currentValue*100)/100
        ac.setText(self.currentValueLabel,"{0}g".format(abs(self.currentValue)))
        if(abs(self.currentValue)<0.1):
             self.currentValue=0
             ac.setText(self.currentValueLabel,"0.0g")
			 
        self.indicatorPosition=self.currentValue/maxG
			 

		 

# This function gets called by AC when the Plugin is initialised
# The function has to return a string with the plugin name
def acMain(ac_version):
    global longitudinalGIndicator, appWindow, lateralGIndicator
    appWindow=ac.newApp("G Meter")
    ac.setSize(appWindow,333,173)
    ac.drawBorder(appWindow,0)
    ac.setBackgroundOpacity(appWindow,0)
    ac.setBackgroundTexture(appWindow,"apps/python/gMeter/gmeterBackground.png")
    lateralGIndicator = GIndicator(appWindow,22,62,"Lat.")
    longitudinalGIndicator = GIndicator(appWindow,22,136,"Lon.")
    ac.addRenderCallback(appWindow , onFormRender)
    return "G Meter"
	
def onFormRender(deltaT):
    global longitudinalGIndicator, lateralGIndicator, barLength
    drawHTriangleIn(167 + (lateralGIndicator.indicatorPosition*(barLength/2)))
    drawLTriangleIn(167 + (longitudinalGIndicator.indicatorPosition*(barLength/2)))
    x,y,z=ac.getCarState(0,acsys.CS.AccG)
    longitudinalGIndicator.setCurrentValue(z)
    lateralGIndicator.setCurrentValue(x)
	
def drawBar():
    ac.glColor4f(1,1,1,1)
    ac.glQuad(0,55,300,7)
    ac.glColor4f(1,1,1,1)
    ac.glQuad(148,45,4,27)

	
def drawLTriangleIn(x):
    global triangleWidth
    ac.glColor4f(1,0,0,1)
    ac.glBegin(acsys.GL.Triangles)
    ac.glVertex2f(x,109)
    ac.glVertex2f(x-(triangleWidth/2),109+triangleWidth)
    ac.glVertex2f(x+(triangleWidth/2),109+triangleWidth)
    ac.glEnd()
    ac.glQuad(x-(triangleWidth/2),109+triangleWidth,triangleWidth,triangleWidth/2)
	
def drawHTriangleIn(x):
    global triangleWidth
    ac.glColor4f(1,0,0,1)
    ac.glBegin(acsys.GL.Triangles)
    ac.glVertex2f(x,104)
    ac.glVertex2f(x-(triangleWidth/2),104-triangleWidth)
    ac.glVertex2f(x+(triangleWidth/2),104-triangleWidth)
    ac.glEnd()
    ac.glQuad(x-(triangleWidth/2),104-(triangleWidth + triangleWidth/2),triangleWidth,triangleWidth/2)
