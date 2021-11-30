##############################################################
# Kunos Simulazioni
# AC Python tutorial 03 : Add a control to the App
#
# To activate create a folder with the same name as this file
# in apps/python. Ex apps/python/tutorial01
# Then copy this file inside it and launch AC
#############################################################



import ac
import acsys

carTexture=0
i_y=20
space=50
FL=0
FR=0
RL=0
RR=0
carHeightFrontValue=0
carHeightBackValue=0

class CarInfoFront:
    def __init__(self,app,x,y):
         self.xPosition=x
         self.yPosition=y
		 		 
         self.labelTemperature=ac.addLabel(app,"Temp:")
         ac.setPosition(self.labelTemperature,x,y)
         self.labelTemperatureValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelTemperatureValue,x+space,y)
         y=y+i_y
		 
         self.labelCamber=ac.addLabel(app,"Camber:")
         ac.setPosition(self.labelCamber,x,y)
         self.labelCamberValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelCamberValue,x+space+20,y)
         y=y+i_y
		 
         self.labelCaster=ac.addLabel(app,"Caster:")
         ac.setPosition(self.labelCaster,x,y)
         self.labelCasterValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelCasterValue,x+space+20,y)
         y=y+i_y
		 
         self.labelPression=ac.addLabel(app,"Pressure:")
         ac.setPosition(self.labelPression,x,y)
         self.labelPressionValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelPressionValue,x+space+20,y)
         y=y+i_y

         self.labelToe=ac.addLabel(app,"TOE:")
         ac.setPosition(self.labelToe,x,y)
         self.labelToeValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelToeValue,x+space+20,y)
         y=y+i_y
		 
    def setValues(self,T1,T2,T3,Cam,Cas,P,TOE):
         ac.setText(self.labelTemperatureValue,"{0}/{1}/{2}".format(round(T1),round(T2),round(T3)))
         ac.setText(self.labelCamberValue,"{0}".format(round(Cam*100)/100))
         ac.setText(self.labelCasterValue,"{0}".format(round(Cas*100)/100))
         ac.setText(self.labelPressionValue,"{0}".format(round(P*100)/100))
         ac.setText(self.labelToeValue,"{0}".format(round(TOE*100)/100))
		 
class CarInfoRear:
    def __init__(self,app,x,y):
         self.xPosition=x
         self.yPosition=y
		 		 
         self.labelTemperature=ac.addLabel(app,"Temp:")
         ac.setPosition(self.labelTemperature,x,y)
         self.labelTemperatureValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelTemperatureValue,x+space,y)
         y=y+i_y
		 
         self.labelCamber=ac.addLabel(app,"Camber:")
         ac.setPosition(self.labelCamber,x,y)
         self.labelCamberValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelCamberValue,x+space+20,y)
         y=y+i_y
		 
         self.labelPression=ac.addLabel(app,"Pressure:")
         ac.setPosition(self.labelPression,x,y)
         self.labelPressionValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelPressionValue,x+space+20,y)
         y=y+i_y

         self.labelToe=ac.addLabel(app,"TOE:")
         ac.setPosition(self.labelToe,x,y)
         self.labelToeValue=ac.addLabel(app,"V")
         ac.setPosition(self.labelToeValue,x+space+20,y)
         y=y+i_y
		 
    def setValues(self,T1,T2,T3,Cam,P,TOE):
         ac.setText(self.labelTemperatureValue,"{0}/{1}/{2}".format(round(T1),round(T2),round(T3)))
         ac.setText(self.labelCamberValue,"{0}".format(round(Cam*100)/100))
         ac.setText(self.labelPressionValue,"{0}".format(round(P*100)/100))
         ac.setText(self.labelToeValue,"{0}".format(round(TOE*100)/100))
		 
# This function gets called by AC when the Plugin is initialised
# The function has to return a string with the plugin name
def acMain(ac_version):
    global carTexture,label, FR,FL,RR,RL, carHeightFrontValue, carHeightBackValue
    appWindow=ac.newApp("Car Status",1)
    ac.setSize(appWindow,400,570)
    carTexture=ac.newTexture("apps/python/system/setup/CarStatus/carView.png")
    ac.addRenderCallback(appWindow , onFormRender)
    FL=CarInfoFront(appWindow,10,	10)
    FR=CarInfoFront(appWindow,240,	10)
    RL=CarInfoRear(appWindow,10,	465)
    RR=CarInfoRear(appWindow,240,	465)

    carHeightFront=ac.addLabel(appWindow,"Front Height:")
    ac.setPosition(carHeightFront,100 ,120)
    carHeightFrontValue=ac.addLabel(appWindow,"V")
    ac.setPosition(carHeightFrontValue,220,120)
	
    carHeightBack=ac.addLabel(appWindow,"Rear Height:")
    ac.setPosition(carHeightBack,100 ,420)
    carHeightBackValue=ac.addLabel(appWindow,"V")
    ac.setPosition(carHeightBackValue,220,420)
	
    return "Car Status Setup APP"
	 
def onFormRender(deltaT):
    global FL,FR,RL,RR, carTexture, carHeightFrontValue, carHeightBackValue
    camFL,camFR,camRL,camRR=ac.getCarState(0,acsys.CS.CamberDeg)
    aCaster=ac.getCarState(0,acsys.CS.Caster)
    pFL,pFR,pRL,pRR = ac.getCarState(0,acsys.CS.DynamicPressure)


    toeFL=ac.getCarState(0,acsys.CS.ToeInDeg,0)
    toeFR=ac.getCarState(0,acsys.CS.ToeInDeg,1)
    toeRL=ac.getCarState(0,acsys.CS.ToeInDeg,2)
    toeRR=ac.getCarState(0,acsys.CS.ToeInDeg,3)
	
    temp1,temp2,temp3 = ac.getCarState(0,acsys.CS.LastTyresTemp,acsys.WHEELS.FL)
    FL.setValues(temp1,temp2,temp3,camFL,aCaster,pFL,toeFL)
    temp1,temp2,temp3 = ac.getCarState(0,acsys.CS.LastTyresTemp,acsys.WHEELS.FR)
    FR.setValues(temp1,temp2,temp3,camFR,aCaster,pFR,-toeFR)
    temp1,temp2,temp3 = ac.getCarState(0,acsys.CS.LastTyresTemp,acsys.WHEELS.RL)
    RL.setValues(temp1,temp2,temp3,camRL,pRL,toeRL)
    temp1,temp2,temp3 = ac.getCarState(0,acsys.CS.LastTyresTemp,acsys.WHEELS.RR)
    RR.setValues(temp1,temp2,temp3,camRR,pRR,-toeRR)
	
    fh,rh = ac.getCarState(0,acsys.CS.RideHeight)
    minHeight = ac.getCarMinHeight(0)

    if (fh<minHeight):
         ac.setFontColor(carHeightFrontValue,1,0,0,1)
    else:
         ac.setFontColor(carHeightFrontValue,1,1,1,1)

    if (rh<minHeight):
         ac.setFontColor(carHeightBackValue,1,0,0,1)
    else:
         ac.setFontColor(carHeightBackValue,1,1,1,1)
		 
    fh = round(fh*1000)
    rh = round(rh*1000)
    minHeight = round(minHeight*1000)
	
    if (rh==minHeight):
         ac.setFontColor(carHeightBackValue,1,1,0,1)
    if (fh==minHeight):
         ac.setFontColor(carHeightFrontValue,1,1,0,1)

    ac.setText(carHeightFrontValue,"~{0} mm - min : {1}".format(fh,minHeight))
    ac.setText(carHeightBackValue,"~{0} mm - min : {1}".format(rh,minHeight))

    ac.glColor4f(1,1,1,1)
    ac.glQuadTextured(75,136,208,288,carTexture)
