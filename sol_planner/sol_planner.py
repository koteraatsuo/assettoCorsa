# Sol Planner
version = "2.1"
'''
- alpha 2
Time shift to slot if Slotheader is pressed
- alpha 3
fixed Time shift swinging with low sun trajectories
- alpha 4
added rain timed changes (probability, variance)
added preset weather parameter
better time change when clicking Slot-header (Noon, Dusk....)
- alpha 5
added rain timed changes (probability, variance)
added preset weather parameter
better time change when clicking Slot-header (Noon, Dusk....)
- alpha 6
added humidity and fog parameter to weather slots
- alpha 7
changed rain probability method
- alpha 8
added timeline mode
added repeat control
added more button textures
added random weather logic
added multiselection edit for random weather (press random weather button again)
changed presets path
- alpha 9
check folders
fixed single weather behavior
slot time adjustment
- alpha 10
more bug fixes
- alpha 11
added import/export of preset-files
- alpha 12
added import/export of single weather plans
added sun and moon angle display
added date readout from csp debug
added reset button / fixed reset function
- beta 1
fixed loading road temperatures
empty app with replay mode or incompatible weatherFX controller
- RC 1
fixed visibility of Slot-editor when switching app pages
- RC4
added wiper control
- 1.2
fixed some bugs with UI
more space for elements
more practical arrangement of slot editor elements
better description if controller is not running
- 1.3
fixed wrong slot time calculation with very small sun angles in night times
- 1.4
proper handling of empty slots
- 1.5
preset button getting the names of exported or imported plans
- 1.6
fixed wiper control
Mist - lower fog value
- 1.7
fixed some bugs with saving presets
- 1.8
fixed day update
fixed plan position bug
added time stamp mode
- 1.9
proper reset if all slots are empty
changed UI / added time multiplier control
preset/plan stores/loads time multiplier
- 2.0
fixed wrong weather transition calculation
- 2.1
fixed calculation of day stamp times
'''

import ac, acsys
import platform, os, sys, math, time
import codecs, json
import configparser, re
import random

from datetime import datetime
from datetime import timedelta

if platform.architecture()[0] == "64bit":
    dllfolder = "stdlib64"
else:
    dllfolder = "stdlib"
cwd = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(cwd, dllfolder))
os.environ['PATH'] = os.environ['PATH'] + ";."
import ctypes
from ctypes import wintypes

def lerp(a,b,c):
    return (c*b) + ((1-c)*a)   

#from sim_info_lib.sim_info import info

weather_params = [
# name                   id         rain   probability   variance   wind   humidity  fog
[ "Light\nThunderstorm" ,  0   ,    0.1,   0.5          ,0.7       ,20.0  ,0.50     ,0.00    ],               
[ "Thunderstorm"        ,  1   ,    0.2,   0.7          ,0.5       ,40.0  ,0.60     ,0.00    ],           
[ "Heavy\nThunderstorm" ,  2   ,    0.4,   0.9          ,0.3       ,70.0  ,0.70     ,0.00    ],               
[ "Light\nDrizzle"      ,  3   ,    0.05,  0.8          ,0.6       ,5.0   ,0.60     ,0.00    ],           
[ "Drizzle"             ,  4   ,    0.15,  0.9          ,0.5       ,10.0  ,0.70     ,0.00    ],       
[ "Heavy\nDrizzle"      ,  5   ,    0.25,  1.0          ,0.4       ,15.0  ,0.80     ,0.00    ],           
[ "Light\nRain"         ,  6   ,    0.3,   0.8          ,0.8       ,10.0  ,0.60     ,0.00    ],       
[ "Rain"                ,  7   ,    0.6,   0.9          ,0.6       ,20.0  ,0.70     ,0.00    ],   
[ "Heavy\nRain"         ,  8   ,    1.0,   1.0          ,0.4       ,30.0  ,0.80     ,0.00    ],
[ "Light\nSnow"         ,  9   ,    0.0,   0.7          ,0.6       ,0.0   ,0.50     ,0.20    ],       
[ "Snow"                , 10   ,    0.0,   0.8          ,0.5       ,0.0   ,0.50     ,0.30    ],   
[ "Heavy\nSnow"         , 11   ,    0.0,   0.9          ,0.4       ,0.0   ,0.50     ,0.50    ],       
[ "Light\nSleet"        , 12   ,    0.2,   0.7          ,0.6       ,0.0   ,0.50     ,0.00    ],           
[ "Sleet"               , 13   ,    0.5,   0.8          ,0.5       ,0.0   ,0.50     ,0.00    ],   
[ "Heavy\nSleet"        , 14   ,    0.8,   0.9          ,0.4       ,0.0   ,0.50     ,0.00    ],           
[ "Clear"               , 15   ,    0.0,   0.0          ,0.0       ,5.0   ,0.50     ,0.00    ],   
[ "Few\nClouds"         , 16   ,    0.0,   0.0          ,0.0       ,15.0  ,0.50     ,0.00    ],       
[ "Scattered\nClouds"   , 17   ,    0.0,   0.2          ,0.8       ,20.0  ,0.50     ,0.00    ],               
[ "Broken\nClouds"      , 18   ,    0.0,   0.5          ,0.9       ,35.0  ,0.50     ,0.00    ],           
[ "Overcast\nClouds"    , 19   ,    0.0,   0.3          ,0.7       ,15.0  ,0.50     ,0.00    ],               
[ "Fog"                 , 20   ,    0.0,   0.0          ,0.0       ,0.0   ,1.00     ,0.80    ],   
[ "Mist"                , 21   ,    0.0,   0.0          ,0.0       ,0.0   ,0.95     ,0.55    ],   
[ "Smoke"               , 22   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],   
[ "Haze"                , 23   ,    0.0,   0.0          ,0.0       ,0.0   ,0.80     ,0.25    ],   
[ "Sand"                , 24   ,    0.0,   0.0          ,0.0       ,0.0   ,0.40     ,0.00    ],   
[ "Dust"                , 25   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],   
[ "Squalls"             , 26   ,    0.0,   0.5          ,0.9       ,70.0  ,0.70     ,0.00    ],       
[ "Tornado"             , 27   ,    0.3,   0.7          ,0.6       ,100.0 ,0.80     ,0.00    ],       
[ "Hurricane"           , 28   ,    0.5,   0.9          ,0.3       ,120.0 ,0.90     ,0.00    ],       
[ "Cold"                , 29   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],   
[ "Hot"                 , 30   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],   
[ "Windy"               , 31   ,    0.0,   0.2          ,0.9       ,50.0  ,0.60     ,0.00    ],   
[ "Hail"                , 32   ,    0.5,   0.9          ,0.0       ,0.0   ,0.50     ,0.00    ],   
[ "No\nClouds"          ,100   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ], 
[ "Random\nDry"         , 40   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],
[ "Random\nRainy"       , 41   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],
[ "Random\nBad"         , 42   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],
[ "Empty"               ,666   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],
[ "CM"                  , 50   ,    0.0,   0.0          ,0.0       ,0.0   ,0.50     ,0.00    ],
]
# do not change the sorting !!!
# always add at the end !!!

from sol_lib.sol_interface import *
from sol_lib.sol_UI import *
from sol_lib.sol_Dialogs import *
from sol_lib.sol_Stellar import Stellar

timer1 = 0.0
timer2 = 0.0
timer3 = 0.0
timer4 = 0.0
timerTimeMultiplier = 0
gTimeMultiplier = 1
gOriginalTimeMultiplier = 1
bTimeMultiControl = False
timerButtonWaitTime = 100.0
timerDateUpdate = 0
timerStart = 0.0
SolCom = SolInterface('plannerAPP')
SolUI = UI("Sol Planner", useMultiplePages=False)
SolUI_SlotEditor = UI("Sol Planner", useMultiplePages=False, usePageButtons=False)
editedPreset = -1
editedSlot = -1
bEditorMultiSelectionMode = False
gEditorSelectedWeather = -1
gInitialCMWeather = {}
initSlot = 4
slotsCount = 9
timeUpdateFromSolController = 0
dayPosition = 0
playPosition = 0
stampPosition = 0
gForceSlotStampTime = -1
ACCSPDate = datetime(1900, 1, 1, hour=0, minute=0, second=0, microsecond=0, tzinfo=None)
playSeconds = 0
lastCurrent = -1
lastNext = -1
lastTransition = 0.0
uiLockTime = 0.5
bUseAbsoluteDay = True
gTimeMode = 0 # 0=day cycle, 1=timespan, 2=timestamp
bPlaying = False
bPause = False
bRepeat = True
bSol21ControllerRunning = False
gWiperMode = -1
gWiperState = 0


if 'ext_setWeatherTimeMult' in dir(ac):
    bTimeMultiControl = True



def update_version_check(version):
    global initSlot, bSol21ControllerRunning

    version = round(math.floor(version*10), 0)*0.1
    ac.log('Sol controller version: %f'%version)
    if version>2:
        showSections = ['LoadSave', 'Stellar', 'Time', 'Date', 'TimMult', 'TimReset', 'PlanDuration', 'Preset', 'Load', 'Aslot', 'Rslot', 'RPlus1', 'RMinus1', 'RPlus2', 'RMinus2', 'Slot', 'Transport', 'Abs', 'Rel', 'Stamp', 'Pic', 'Wipers']
        for section in showSections:
            n = SolUI.getElementsOfSection(section)
            for tmp in n:
                tmp.show()

        n = SolUI.getElement('Label', 'version')
        if len(n)>0:
            n[0].hide()

        #if initSlot>=0:
            #gSlots[initSlot].press()

        bSol21ControllerRunning = True
        updateModeButtons()



class Slot:
    def __init__(self, number, **args):
        self.number = number

        self.tex = -1
        
        self.virginRain = True
        self.virginHumidity = True
        self.virginFog = True
        self.virginWind = True

        self.active = False
        self.editing = False
        self.weatherID = -1
        self.phantomWeatherID = -1 # here the real weatherID is stored if for example "CM"-weather is selected
        self.weatherList = []
        self.calculatedWeather=-1

        self.rain = 0
        self.rainProbability = 0
        self.rainVariance = 0

        self.humidity = 0
        self.fog = 0

        self.RainForce = 0
        self.RainDecay = 0.5
        self.RainCalculated = 0
        self.rainResult = 0

        self.daySeconds = 0
        self.playSeconds = 300 #5 min
        self.stampHour = 16 + number * 3
        if self.stampHour > 23:
            self.stampHour -= 24
        self.stampMinute = 0
        self.stampDateTime = datetime(1900, 1, 1, hour=0, minute=0, second=0, microsecond=0, tzinfo=None)

        self.wind_strength = 0
        self.wind_dir = 0
        self.temp_ambient = 20
        self.temp_road = 30

        self.slotButton = 0

    def reset(self):

        self.tex = -1

        self.virginRain = True
        self.virginHumidity = True
        self.virginFog = True
        self.virginWind = True

        self.rain = 0
        self.rainProbability = 0
        self.rainVariance = 0

        self.humidity = 0
        self.fog = 0

        self.RainForce = 0
        self.RainDecay = 0.5
        self.RainCalculated = 0
        self.rainResult = 0

        self.wind_strength = 0
        self.wind_dir = 0
        self.temp_ambient = 20
        self.temp_road = 30

        
    def setButton(self, element):
        self.slotButton = element
        if self.slotButton:
            self.setActive(self.active)

    def setActive(self, b):
        self.active = b
        if self.slotButton:
            if b:
                self.slotButton.setColor(0.8,0.8,0.8,1)
            else:
                self.slotButton.setColor(0.2,0.2,0.2,1)

    def setEditing(self, b):
        self.editing = b
        if self.slotButton:
            if b:
                self.slotButton.setBackgroundColor(0.5,1.0,0.0)
            else:
                self.slotButton.setBackgroundColor(0.1,0.1,0.1)

    def setWeather(self, wid, preset):
        global gInitialCMWeather
        if weather_params[wid][1]==666: #empty
            self.weatherID = -1
            self.phantomWeatherID = -1
            self.reset()
            if self.slotButton:
                self.slotButton.setText('%i'%(self.number+1))
            self.setActive(False)
            #ac.setText(self.tempLabel, "")
        else:

            self.weatherID = wid
            self.phantomWeatherID = -1

            # if "CM" is the selected weather, search for its real meaning
            if weather_params[self.weatherID][1]==50 and ('weatherID' in gInitialCMWeather) and int(gInitialCMWeather['weatherID'])>0:
                self.checkPhantomWeather()
                self.weatherID = self.phantomWeatherID

            if self.slotButton:
                self.slotButton.setText(weather_params[self.weatherID][0])
            self.active = True

            if preset:
                if self.virginRain:
                    self.rain = weather_params[self.weatherID][2]
                    self.rainProbability = weather_params[self.weatherID][3]
                    self.rainVariance = weather_params[self.weatherID][4]
                    self.RainForce = 1

                if self.virginHumidity:
                    self.humidity = weather_params[self.weatherID][6]
                    SolCom.init_value('current', 'humidity', gSlots[editedSlot].humidity)
                if self.virginFog:
                    self.fog = weather_params[self.weatherID][7]
                    SolCom.init_value('current', 'fog', gSlots[editedSlot].fog)

                if self.virginWind:
                    self.wind_strength = weather_params[self.weatherID][5]

            n = SolUI_SlotEditor.getElement('Weather', '%i'%wid)
            if len(n)>0:
                self.tex = n[0].OffTex
            self.setActive(self.active)
            self.weatherID = wid #set the real weatherID again

        self.calculatedWeather = -1


    def setWeatherList(self, widList):
        self.weatherList = []
        for tmp in widList:
            self.weatherList.append(tmp)

    def getWeatherList(self):
        return self.weatherList

    def getWeather(self):
        if self.phantomWeatherID>=0:
            return self.phantomWeatherID
        else:
            return self.weatherID

    def checkPhantomWeather(self):
        global gInitialCMWeather
        if self.weatherID >=0 and weather_params[self.weatherID][1] == 50: # CM weather
            if 'weatherID' in gInitialCMWeather:
                if int(gInitialCMWeather['weatherID'])>=0:
                    wid = int(gInitialCMWeather['weatherID'])
                    i=0
                    for tmp in weather_params:
                        if wid == tmp[1]:
                            #ac.log('CM Weather: %i = %s'%(tmp[1], tmp[0]))
                            self.phantomWeatherID = i
                            self.slotButton.setText(weather_params[self.phantomWeatherID][0])
                            break
                        i+=1

            
            self.setRain(weather_params[self.getWeather()][2]) #( float(order['value']) )
            self.setRainProbability(weather_params[self.getWeather()][3])
            self.setRainVariance(weather_params[self.getWeather()][4])
            #updateWipers(weather_params[self.getWeather()][2])

            if 'humidity' in gInitialCMWeather:
                if float(gInitialCMWeather['humidity']) < 0 and self.getWeather() >= 0:
                    self.humidity = weather_params[self.getWeather()][6]
                else:
                    self.humidity = ( float(gInitialCMWeather['humidity']) )

            if 'fog' in gInitialCMWeather:
                if float(gInitialCMWeather['fog']) < 0 and self.getWeather() >= 0:
                    self.fog = weather_params[self.getWeather()][7]
                else:
                    self.fog = float(gInitialCMWeather['fog'])

            if 'wind_strength' in gInitialCMWeather:
                self.wind_strength = int(gInitialCMWeather['wind_strength'])

            if 'wind_dir' in gInitialCMWeather:
                self.wind_dir = int(gInitialCMWeather['wind_dir'])

            if 'temp_ambient' in gInitialCMWeather:
                self.setTempAmbient(int(gInitialCMWeather['temp_ambient']))

            if 'temp_road' in gInitialCMWeather:
                self.temp_road = int(gInitialCMWeather['temp_road'])
            

    def calculateWeather(self):
        global editedSlot
        wid = self.getWeather()
        if wid >= 0:
            if weather_params[wid][1]>=40 and weather_params[wid][1]<=42: # random weather
                if len(self.weatherList)>0:
                    rnd = math.floor(random.random() * len(self.weatherList))
                    self.calculatedWeather = self.weatherList[rnd]
                    
                    self.setRain(weather_params[self.calculatedWeather][2])
                    self.setRainProbability(weather_params[self.calculatedWeather][3])
                    self.setRainVariance(weather_params[self.calculatedWeather][4])
                    self.wind_strength = weather_params[self.calculatedWeather][5]
                    self.setHumidity(weather_params[self.calculatedWeather][6])
                    self.setFog(weather_params[self.calculatedWeather][7])

                    if editedSlot == self.number:
                        unmarkWeatherInEditor()
                        markWeatherInEditor()
                else:
                    self.calculatedWeather = 33 #noCLouds
            else:
                self.calculatedWeather = wid
        return self.calculatedWeather

    def getCalculatedWeather(self):
        if self.calculatedWeather<0:
            self.calculateWeather()
        return self.calculatedWeather

    def setRain(self, amount):
        self.rain = amount
        self.RainCalculated = self.rain

    def setRainProbability(self, prob):
        self.rainProbability = prob
        self.RainCalculated = self.rain * self.rainProbability

    def setRainVariance(self, vari):
        self.rainVariance = vari

    def getRain(self):
        return self.rainResult

    def setHumidity(self, humid):
        self.humidity = humid

    def setFog(self, fog):
        self.fog = fog
    
    def setTempAmbient(self, temp):
        #ac.setText(self.tempLabel, "%i°"%temp)
        self.temp_ambient = temp

    def press(self):
        if self.slotButton:
            self.slotButton.press()

    def update(self, dt):
        #if self.rain>0:

        self.RainForce += 0.0167 * dt# math.pow(self.rainProbability, 3) * dt * 0.25 * random.random()
        if self.RainForce>1:
            prob = random.random() / max(0.01, self.rainProbability)
            if prob <= 1:
                self.RainDecay = 2
            else:
                if random.random() >= 0.5:
                    self.RainDecay = 0.5
                else:
                    self.RainDecay = 0.45
            self.RainForce = 0
        #self.RainDecay -= dt*(random.random()*60*(self.RainCalculated+(1-self.rainProbability)))
        #self.RainDecay = max(0.45, self.RainDecay)

        self.RainCalculated = self.RainCalculated + self.rain*(self.RainDecay-random.random())*dt*0.25 + (random.random()-0.5)*self.rainVariance*dt
        self.RainCalculated = max(0, min( self.rain, self.RainCalculated))
        
        #ac.log('force:%f, decay:%f'%(self.RainForce, self.RainDecay))
        #else:
        #   self.RainCalculated = 0
        
        self.rainResult = lerp(self.rainResult, self.RainCalculated, min(1, dt*0.5))

        


    def isCurrent(self):
        global lastCurrent
        return lastCurrent==self.number

    def isNext(self):
        global lastNext
        return lastNext==self.number

gSlots = []












settings_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "settings")
if not os.path.exists(settings_path):
    os.makedirs(settings_path)
settings_default_ini_path = os.path.join(settings_path, "settings_defaults.ini")
settings_ini_path = os.path.join(settings_path, "settings.ini")
settings_ini      = configparser.ConfigParser(comment_prefixes=('#', ';'))
if not os.path.isfile(settings_ini_path): # create empty if not there
    #with open(settings_default_ini_path, 'r', encoding='utf-8') as ini_default:
    with open(settings_ini_path, 'w', encoding='utf-8') as ini:
        ini.write('') #ini_default.read()
settings_ini.read(settings_ini_path)

presets_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "presets")
if not os.path.exists(presets_path):
    os.makedirs(presets_path)
presets_ini_defaultFile = os.path.join(presets_path, "presets.ini")
presets_ini      = configparser.ConfigParser(comment_prefixes=('#', ';'))
if not os.path.isfile(presets_ini_defaultFile): # create empty if not there
    with open(presets_ini_defaultFile, 'w', encoding='utf-8') as ini:
        ini.write('') #ini_default.read()

plans_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "plans")









def convert_seconds_to_hhmm_text(seconds):
    return time.strftime('%H:%M', time.gmtime(seconds))

def resetSlotTimeStamps():
    global ACCSPDate, gSlots, stampPosition
    
    if ACCSPDate.year > 1970:

        if stampPosition < 1: 
            # only reset the complete plan if its in the first slot
            gSlots[0].stampDateTime = gSlots[0].stampDateTime.replace(year=ACCSPDate.year, month=ACCSPDate.month, day=ACCSPDate.day, hour=gSlots[0].stampHour, minute=gSlots[0].stampMinute, second=0)

        for i in range(1,9):
            gSlots[i].stampDateTime = gSlots[i].stampDateTime.replace(year=gSlots[i-1].stampDateTime.year, month=gSlots[i-1].stampDateTime.month, day=gSlots[i-1].stampDateTime.day, hour=gSlots[i].stampHour, minute=gSlots[i].stampMinute, second=0)
            if (gSlots[i].stampDateTime - gSlots[i-1].stampDateTime).total_seconds() <= 0:
                gSlots[i].stampDateTime += timedelta(days=1)
            
        # update date text also
        for i in range(0,9):
            n = SolUI.getElement('SDate', '%i'%i)
            if len(n)>0:
                n[0].setText('%s.%s'%(gSlots[i].stampDateTime.day, gSlots[i].stampDateTime.month))


def resetACTimeToSlotStampTime(slot_number):
    global ACCSPDate, gSlots, gForceSlotStampTime

    if ACCSPDate.year <= 1970:
        gForceSlotStampTime = slot_number
        return

    #if slot_number >= 0 and slot_number < 9:
    delta = gSlots[slot_number].stampDateTime - ACCSPDate
    ac.ext_weatherTimeOffset( delta.total_seconds() )

    #ac.log('AC %i-%i-%i, %i:%i'%(ACCSPDate.year, ACCSPDate.month, ACCSPDate.day, ACCSPDate.hour, ACCSPDate.minute))
    #ac.log('ST %i-%i-%i, %i:%i'%(stampDate.year, stampDate.month, stampDate.day, stampDate.hour, stampDate.minute))
    #ac.log('## %i'%delta.total_seconds())


def setTimeMultiplier(v):
    global gTimeMultiplier, timerDateUpdate, SolUI, presets_ini_defaultFile, presets_ini, editedPreset
    gTimeMultiplier = v
    n = SolUI.getElement('TimMult', 'Label')
    if len(n)>0:
        n[0].setText('%ix'%gTimeMultiplier)
    timerDateUpdate = 0
    calcPlanDuration()

    if editedPreset>=0:
        # save time multi just once
        section = str(editedPreset)
        if not presets_ini.has_section(section):
            presets_ini.add_section(section)
        presets_ini.set(section, 'timeMultiplier', str(gTimeMultiplier))


def resetTimeDate():
    global gOriginalTimeMultiplier
    setTimeMultiplier(gOriginalTimeMultiplier)


def resetWiper():
    global gWiperState
    for i in range(0,4):
        ac.ext_rainParamsAdjust(0, -1)
    gWiperState = 0

def updateWipers(rainAmount):
    global gWiperMode, gWiperState
    if gWiperMode == -2:
        # no control
        pass
    elif gWiperMode == -1:
        # automatic control
        tmp = 0
        
        if rainAmount >= 0.75:
            tmp=4
        elif rainAmount >= 0.55:
            tmp=3
        elif rainAmount >= 0.3:
            tmp=2
        elif rainAmount >= 0.1:
            tmp=1
        
        ac.ext_rainParamsSet(0, tmp)
        '''
        if tmp > gWiperState:
            ac.ext_rainParamsAdjust(0, 1)
            gWiperState += 1
        elif tmp < gWiperState and gWiperState>0:
            ac.ext_rainParamsAdjust(0, -1)
            gWiperState -= 1
        '''
    else:
        ac.ext_rainParamsSet(0, gWiperMode)
        '''
        if gWiperMode > gWiperState:
            ac.ext_rainParamsAdjust(0, 1)
            gWiperState += 1
        elif gWiperMode < gWiperState and gWiperState>0:
            ac.ext_rainParamsAdjust(0, -1)
            gWiperState -= 1
        '''

def positioningSlotEditor():
    global SolUI, SolUI_SlotEditor
    scale = SolUI.getScale()
    pos = ac.getPosition(SolUI.window)
    ac.setPosition(SolUI_SlotEditor.window, pos[0]+(editedSlot/8)*((SolUI.width-120-SolUI_SlotEditor.width)*scale), pos[1] + SolUI.height*scale + 1)

def updatePresetButton(key):
    global SolUI, presets_ini
    for i in range(0,5):
        section = str(i)
        n = SolUI.getElement('Preset', '%i'%i)
        if len(n)>0:
            if n[0].key not in key:
                if presets_ini.has_section(section):
                    n[0].setBackgroundColor(0.0,0.33,0.75)
                else:
                    n[0].setBackgroundColor(0.1,0.1,0.1)
            else:
                n[0].setBackgroundColor(0.5,1.0,0.0)

        if presets_ini.has_option(section, 'name'):
            s = presets_ini.get(section,'name')
            if len(s) > 0:
                n[0].setText(s)
            else:
                n[0].setText('Preset ' + str(i+1))
        else:
            n[0].setText('Preset ' + str(i+1))


def updateAbsSlotTimes(stellarObj):
    # Night
    gSlots[0].daySeconds = 1800 
    # Dawn
    gSlots[1].daySeconds = stellarObj.getTimeFromSunangle(-9, 'am')*60
    if gSlots[1].daySeconds < 0: # if sun can't reach this angle
        gSlots[1].daySeconds = 3900 #2:30
    # Sunrise
    gSlots[2].daySeconds = stellarObj.getTimeFromSunangle(2, 'am')*60
    if gSlots[2].daySeconds < gSlots[1].daySeconds or gSlots[2].daySeconds < 0:
        gSlots[2].daySeconds = gSlots[1].daySeconds + 1800

    # Dusk
    gSlots[8].daySeconds = stellarObj.getTimeFromSunangle(-9, 'pm')*60
    if gSlots[8].daySeconds < 0: # if sun can't reach this angle
        gSlots[8].daySeconds = 84600 #23:30
    # Sunset
    gSlots[7].daySeconds = stellarObj.getTimeFromSunangle(2, 'pm')*60
    if gSlots[7].daySeconds > gSlots[8].daySeconds or gSlots[7].daySeconds < 0:
        gSlots[7].daySeconds = gSlots[8].daySeconds - 1800

    # Morning
    gSlots[3].daySeconds = 28800 # 8:00
    if gSlots[3].daySeconds < gSlots[2].daySeconds + 1800:
        gSlots[3].daySeconds = gSlots[2].daySeconds + 1800
    
    # Noon
    gSlots[4].daySeconds = 45000 # 12.30

    # Evening
    gSlots[6].daySeconds = 68400 # 19:00
    if gSlots[6].daySeconds > gSlots[7].daySeconds - 1800:
        gSlots[6].daySeconds = gSlots[7].daySeconds - 1800

    # Afternoon
    gSlots[5].daySeconds = 57600 # 16:00
    if gSlots[5].daySeconds > gSlots[6].daySeconds - 1800:
        gSlots[5].daySeconds = gSlots[6].daySeconds - 1800


    '''
    for i in range(0,9):
        n = SolUI.getElement('Aslot', '%i'%i)
        if len(n)>0:
            n[0].setText(time.strftime('%H:%M', time.gmtime(gSlots[i].daySeconds)))
    '''
   

def updateStellar(stellarObj):
    updateAbsSlotTimes(stellarObj)

SolStellar = Stellar(updateStellar) 


def calcPlanDuration():
    global SolUI, gTimeMode, gSlots, gTimeMultiplier

    sec = 0

    if gTimeMode<1: #day cycle
        sec = 86400
    elif gTimeMode<2: # time span
        for i in range(0,9):
            sec += gSlots[i].playSeconds
    else:
        sec = (gSlots[8].stampDateTime - gSlots[0].stampDateTime).total_seconds()

    n = SolUI.getElement('PlanDuration', 'Text')
    if len(n)>0:
        if gTimeMultiplier>0.01:
            sec /= gTimeMultiplier
            sec = math.floor(sec)
            n[0].setText('%s'%str(timedelta(seconds=sec)))
        elif gTimeMultiplier<-0.01:
            sec /= -gTimeMultiplier
            sec = math.floor(sec)
            n[0].setText('%s'%str(timedelta(seconds=sec)))
        else:
            n[0].setText('forever and ever')

    
def calcPlayPositionDay(stellarObj):
    global gSlots, dayPosition
    sec = stellarObj.daySeconds

    if sec < gSlots[0].daySeconds or sec > gSlots[8].daySeconds:
        if sec < gSlots[0].daySeconds:
            sec+=86400
        dayPosition = 8+(sec-gSlots[8].daySeconds) / max(1, (gSlots[0].daySeconds+86400) - gSlots[8].daySeconds)
    else:
        for i in range(0,8):
            if sec>=gSlots[i].daySeconds and sec<gSlots[i+1].daySeconds:
                dayPosition = i+(sec-gSlots[i].daySeconds) / max(1, gSlots[i+1].daySeconds - gSlots[i].daySeconds)
                break
        
    if dayPosition >= 8.5:
        dayPosition-=9 


def shiftPlayPositionTime(slot_number):
    global playSeconds, gSlots, playPosition

    playPosition = slot_number
    playSeconds = 0
    for i in range(0,slot_number):
        playSeconds += gSlots[i].playSeconds

    updatePlayPosition(True)
    #resetWeatherFX()
           

def calcPlayPositionTime():
    global playSeconds, bRepeat, gSlots, playPosition

    playPosition = 0
    tmp = 0
    reachedEnd = True
    for i in range(0,9):
        if playSeconds >= tmp and playSeconds < (tmp + gSlots[i].playSeconds):
            playPosition = i + (playSeconds-tmp) / max(1, gSlots[i].playSeconds)
            reachedEnd = False
            break
        tmp += gSlots[i].playSeconds
    
    if playPosition>=9 or reachedEnd:
        if bRepeat:
            playPosition-=9 
            playSeconds=0
        else:
            playPosition=9
            playSeconds=tmp
            #bPlaying = False
            pause()
    #ac.log('playPosition: %f'%playPosition)


def calcPlayPositionStamp():
    global stampPosition, gSlots

    sec=0
    sec2=0
    stampPosition = 0

    sec = (ACCSPDate - gSlots[0].stampDateTime).total_seconds()
    if sec < 0:
        return

    sec2 = (ACCSPDate - gSlots[1].stampDateTime).total_seconds()
    for i in range(0,8):
        
        if sec >= 0 and sec2 < 0:
            stampPosition = i + min(1, sec / max(1, (gSlots[i+1].stampDateTime - gSlots[i].stampDateTime).total_seconds()))
            break

        if i==7:
            stampPosition=8.0
            break

        sec = sec2
        if i < 7:
            sec2 = (ACCSPDate - gSlots[i+2].stampDateTime).total_seconds()
        

def updatePlayPosition(bInit):

    global lastTransition, lastCurrent, lastNext, gTimeMode, bUseAbsoluteDay, dayPosition, playPosition, gSlots, SolCom

    pos = 0
    if gTimeMode < 1: # day cycle
        pos = dayPosition
        bInit = True
    elif gTimeMode < 2: # timespan
        pos = playPosition
        #bInit = True
    else: # time stamp
        pos = stampPosition
        bInit = True

    
    if pos>=9: return #do nothing if timed weather plan is not repeating and reaches its end

    currentW = math.floor(pos)

    nextW = currentW+1
    transition = 0 

    widC = -1
    tranC = currentW
    for i in range(currentW, -1, -1):
        widC = gSlots[i].getWeather()
        if widC>=0:
            tranC = i
            break
    if widC<0:
        for i in range(8, -1, -1):
            widC = gSlots[i].getWeather()
            if widC>=0:
                tranC = i
                break
    if widC>=0:
        if lastCurrent<0:
            lastCurrent = tranC
            gSlots[tranC].calculateWeather()
        widC = gSlots[tranC].getCalculatedWeather()
        if widC>=0:
            SolCom.set_value('current', 'weatherID', weather_params[widC][1])

    widN = -1
    tranN = nextW
    for i in range(nextW, 9):
        widN = gSlots[i].getWeather()
        if widN>=0:
            tranN = i
            break 
    if widN < 0: #try again from Slot 0 on
        if not ((gTimeMode==1 and not bRepeat) or gTimeMode==2):
            for i in range(0, 9):
                widN = gSlots[i].getWeather()
                if widN>=0:
                    tranN = i
                    break 
        else:
           tranN = tranC
           widN  = widC

    if widN>=0:
        if tranN!=lastNext: #calculate weather if next is changing
            lastNext = tranN
            gSlots[tranN].calculateWeather()
        widN = gSlots[tranN].getCalculatedWeather()
        if widN>=0:
            SolCom.set_value('next', 'weatherID', weather_params[widN][1])

    #ac.log('current %i, next %i'%(widC, widN))
    #ac.log('curr: %i, next: %i'%(tranC,tranN))
        
    if widC<0 and widN<0:

        SolCom.set_value('current', 'weatherID', 100) #noClouds
        SolCom.set_value('next', 'weatherID', 100) #noClouds
        SolCom.send_command(['TransisionInit', '100', '100', '0']) 

        SolCom.set_value('current', 'rain', 0)
        SolCom.set_value('current', 'humidity',      0  ) 
        SolCom.set_value('current', 'fog',           0  ) 
        SolCom.set_value('current', 'wind_strength', 0  )  
        SolCom.set_value('current', 'temp_ambient',  20 ) 
        SolCom.set_value('current', 'temp_road',     20 )

        updateWipers(0)

    elif tranN < 0 or tranC==tranN:
        if tranC < 0:# no slot has a weather

            SolCom.set_value('next', 'weatherID', 100) #noClouds

            SolCom.set_value('current', 'rain', 0)
            updateWipers(0)
        else:
            SolCom.set_value('next', 'weatherID', weather_params[widC][1])
            SolCom.set_value('current', 'rain', gSlots[tranC].getRain())
            SolCom.set_value('current', 'humidity',      gSlots[tranC].humidity      ) 
            SolCom.set_value('current', 'fog',           gSlots[tranC].fog           ) 
            SolCom.set_value('current', 'wind_strength', gSlots[tranC].wind_strength ) 
            SolCom.set_value('current', 'wind_dir',      gSlots[tranC].wind_dir      ) 
            SolCom.set_value('current', 'temp_ambient',  gSlots[tranC].temp_ambient  ) 
            SolCom.set_value('current', 'temp_road',     gSlots[tranC].temp_road     ) 

            updateWipers(gSlots[tranC].getRain())
        
        #SolCom.init_value('current', 'transition', 0)  
        SolCom.send_command(['TransisionInit', str(weather_params[widC][1]), str(weather_params[widC][1]), '0']) 
    else:

        C = tranC
        N = tranN

        if ((gTimeMode==1 and not bRepeat) or gTimeMode==2) and tranC>tranN:
            C = N
            transition = 0
        else:
            if tranC>tranN:
                tranN+=9
            if pos<0 or tranC>pos:
                pos+=9
            transition = (pos-tranC) / (tranN-tranC)

        tmp_rain = lerp(gSlots[C].getRain(),     gSlots[N].getRain(),     transition)

        SolCom.set_value('current', 'rain',          tmp_rain )
        SolCom.set_value('current', 'humidity',      lerp(gSlots[C].humidity,      gSlots[N].humidity,      transition)) 
        SolCom.set_value('current', 'fog',           lerp(gSlots[C].fog,           gSlots[N].fog,           transition))
        SolCom.set_value('current', 'wind_strength', lerp(gSlots[C].wind_strength, gSlots[N].wind_strength, transition)) 
        SolCom.set_value('current', 'wind_dir',      lerp(gSlots[C].wind_dir,      gSlots[N].wind_dir,      transition)) 
        SolCom.set_value('current', 'temp_ambient',  lerp(gSlots[C].temp_ambient,  gSlots[N].temp_ambient,  transition)) 
        SolCom.set_value('current', 'temp_road',     lerp(gSlots[C].temp_road,     gSlots[N].temp_road,     transition)) 

        updateWipers(tmp_rain)

        if abs(transition - lastTransition) > 0.5:
            bInit = True

        if bInit:
            #SolCom.init_value('current', 'transition', transition)
            #ac.log('Transition Init %s, %s, %s'%(str(weather_params[widC][1]), str(weather_params[widN][1]), str(transition)))
            SolCom.send_command(['TransisionInit', str(weather_params[widC][1]), str(weather_params[widN][1]), str(transition)])
        else:   
            SolCom.set_value('current', 'transition', transition) 

        lastTransition = transition

def updateTime(stellarObj):
    global SolUI
    n = SolUI.getElement('Time', 'Label')
    if len(n)>0:
        n[0].setText(convert_seconds_to_hhmm_text(stellarObj.daySeconds))

def updateDate(date):
    n = SolUI.getElement('Date', 'Label')
    if len(n)>0:
        n[0].setText(date)

def shiftTimeToSlot(slot):
    global SolStellar
    step = gSlots[slot].daySeconds - SolStellar.daySeconds
    ac.ext_weatherTimeOffset( step )

def markWeatherInEditor():
    global SolUI_SlotEditor

    wid = gSlots[editedSlot].weatherID 
    n = SolUI_SlotEditor.getElement('Weather', str(wid))
    if len(n)>0:
        if bEditorMultiSelectionMode:
            n[0].setBackgroundColor(1,0.5,0)
            n[0].setBackgroundOpacity(0.75)
        else:
            n[0].setBackgroundColor(0.5,1,0)
            n[0].setBackgroundOpacity(0.25)

    CMweather=False

    if weather_params[wid][1] >=40 and weather_params[wid][1] <= 42: # random buttons
    # show multiselection if random weather is selected
        data = gSlots[editedSlot].getWeatherList()
        if len(data)>0:
            for tmp in data:
                nn = SolUI_SlotEditor.getElement('Weather', str(tmp))
                if len(nn)>0:
                    nn[0].setBackgroundColor(0,0.5,1)

    elif weather_params[wid][1] == 50:
        if gSlots[editedSlot].phantomWeatherID >= 0:
            CMweather=True
            nn = SolUI_SlotEditor.getElement('Weather', str(gSlots[editedSlot].phantomWeatherID))
            if len(nn)>0:
                nn[0].setBackgroundColor(1.0,0.0,0.0)


    n = SolUI_SlotEditor.getElement('Slot', 'Properties')
    if len(n)>0:
        n[0].setText("Slot %i properties"%(editedSlot+1))
    n = SolUI_SlotEditor.getElement('Slot', 'Weather')
    if len(n)>0:
        text=''
        if gSlots[editedSlot].getWeather() >= 0:
            if CMweather:
                text='CM | '
            text = '%s%s'%(text, weather_params[gSlots[editedSlot].getWeather()][0])
        text = text.replace('\n', ' ')
        n[0].setText("%s"%text)


    n = SolUI_SlotEditor.getElement('Rain', 'amount')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].rain)
    n = SolUI_SlotEditor.getElement('Rain', 'probability')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].rainProbability)
    n = SolUI_SlotEditor.getElement('Rain', 'variance')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].rainVariance)
    
    n = SolUI_SlotEditor.getElement('Humidity', 'amount')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].humidity)
    n = SolUI_SlotEditor.getElement('Fog', 'amount')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].fog)

    n = SolUI_SlotEditor.getElement('Wind', 'strength')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].wind_strength)

    n = SolUI_SlotEditor.getElement('Wind', 'direction')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].wind_dir)

    n = SolUI_SlotEditor.getElement('Temp', 'ambient')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].temp_ambient)

    n = SolUI_SlotEditor.getElement('Temp', 'road')
    if len(n)>0:
        n[0].initValue(gSlots[editedSlot].temp_road)
        
def unmarkWeatherInEditor():
    global SolUI_SlotEditor

    for i in range(0,len(weather_params)):
        n = SolUI_SlotEditor.getElement('Weather', str(i))
        if len(n)>0:
            n[0].setBackgroundColor(0.10,0.10,0.10)


def updateMultiSelection():
    global SolUI_SlotEditor, editedSlot

    data = gSlots[editedSlot].getWeatherList()
    for i in range(0,len(weather_params)):
        if (weather_params[i][1] >= 0 and weather_params[i][1] <= 32) or weather_params[i][1]==50 or weather_params[i][1]==100:
            n = SolUI_SlotEditor.getElement('Weather', str(i))
            if len(n)>0:
                if i in data:
                    n[0].setBackgroundColor(0,0.5,1)
                else:
                    n[0].setBackgroundColor(0.10,0.10,0.10)
                

def updateTimeSlotButtons():
    global SolUI, gTimeMode, bUseAbsoluteDay, gSlots

    for i in range(0,9):
        n = SolUI.getElement('Aslot', '%i'%i)
        if len(n)>0:
            if gTimeMode==0: # day cycle
                n[0].use()
                n[0].setColor(1,1,1,1)
            elif gTimeMode==1: # time span mode
                n[0].use()
                n[0].setColor(0.6,0.6,0.2,1) #do not make too dark, it can be still used for time span mode
            else: # time stamp
                n[0].setColor(0.1,0.1,0.1,1)
                n[0].unuse() # prevent usage
        n = SolUI.getElement('Rslot', '%i'%i)
        if len(n)>0:
            if gTimeMode<1:
                n[0].setColor(0.1,0.1,0.1,1)
            elif gTimeMode<2: #time span
                n[0].setColor(1,1,1,1)
                n[0].setText('%i'%(gSlots[i].playSeconds/60))
            else: # stamp mode
                n[0].setColor(1,1,1,1)
                n[0].setText(convert_seconds_to_hhmm_text(gSlots[i].stampHour*3600+gSlots[i].stampMinute*60))

    if gTimeMode==2:
        resetSlotTimeStamps()

    calcPlanDuration()
    
    updatePlayPosition(True)
    showRepeat()

def resetWeatherFX():
    global SolCom
    ac.log('resetting Sol...')
    try:
        SolCom.restart()
        with codecs.open('extension/weather/sol/reset_dummy.lua', 'w', 'utf_8', errors='ignore') as f:# write
            f.write('\n')
    except:
        ac.log("SolPlannerApp error: " + sol_dummy + "\n" + traceback.format_exc())
        ac.console("SolPlannerApp error: " + sol_dummy + "\n" + traceback.format_exc())



def savePreset(parser, file, n):
    global gSlots, gTimeMode, bUseAbsoluteDay, bRepeat, bPlaying, weather_params

    section = str(n)
    if n < 0:
        section = 'plan'
        if not parser.has_section(section):
            parser.add_section(section)
        parser.set(section, 'timeMultiplier', str(gTimeMultiplier))

    if not parser.has_section(section):
        parser.add_section(section)

    nn = 0
    for i in range(0,9):
        wid = gSlots[i].weatherID
        if wid >=0 and wid != 37: # not empty
            nn+=1
            parser.set(section, '%i-weather'%i, '%i'%wid)

            if weather_params[wid][1] <40 or weather_params[wid][1] > 42: # random buttons
                # clear data in weatherlists
                gSlots[i].setWeatherList([])

            if len(gSlots[i].getWeatherList())>0:
                tmp_str = ''
                for tmp in gSlots[i].getWeatherList():
                    if len(tmp_str)>0:
                        tmp_str += ','
                    tmp_str += str(tmp)
                parser.set(section, '%i-weather_list'%i, tmp_str)
            else:
                parser.remove_option(section, '%i-weather_list'%i)


            parser.set(section, '%i-rain'%i, '%.2f'%(gSlots[i].rain))
            parser.set(section, '%i-rainprobability'%i, '%.2f'%(gSlots[i].rainProbability))
            parser.set(section, '%i-rainvariance'%i, '%.2f'%(gSlots[i].rainVariance))

            parser.set(section, '%i-humidity'%i, '%.2f'%(gSlots[i].humidity))
            parser.set(section, '%i-fog'%i, '%.2f'%(gSlots[i].fog))

            parser.set(section, '%i-tempambient'%i, '%i'%(gSlots[i].temp_ambient))
            parser.set(section, '%i-temproad'%i, '%i'%(gSlots[i].temp_road))

            parser.set(section, '%i-wind'%i, '%i'%(gSlots[i].wind_strength))
            parser.set(section, '%i-winddirection'%i, '%i'%(gSlots[i].wind_dir))
        else:
            key = '%i-weather'%i
            parser.remove_option(section, key)
            key = '%i-weather_list'%i
            parser.remove_option(section, key)
            key = '%i-rain'%i
            parser.remove_option(section, key)
            key = '%i-rainprobability'%i
            parser.remove_option(section, key)
            key = '%i-rainvariance'%i
            parser.remove_option(section, key)
            key = '%i-humidity'%i
            parser.remove_option(section, key)
            key = '%i-fog'%i
            parser.remove_option(section, key)
            key = '%i-tempambient'%i
            parser.remove_option(section, key)
            key = '%i-temproad'%i
            parser.remove_option(section, key)
            key = '%i-wind'%i
            parser.remove_option(section, key)
            key = '%i-winddirection'%i
            parser.remove_option(section, key)

        
        parser.set(section, '%i-playseconds'%i, '%i'%(gSlots[i].playSeconds))
        
        parser.set(section, '%i-stamphour'%i, '%i'%(gSlots[i].stampHour))
        parser.set(section, '%i-stampminute'%i, '%i'%(gSlots[i].stampMinute))

    if nn==0:
        parser.remove_section(section)
    else:
        #parser.set(section, 'useFullDay', str(bUseAbsoluteDay))
        parser.set(section, 'timeMode', str(gTimeMode))
        #parser.set(section, 'timeMultiplier', str(gTimeMultiplier))
        parser.set(section, 'repeat', str(bRepeat))
        parser.set(section, 'playing', str(bPlaying))

    if len(file)>0: 
        with open(file, 'w') as ini:   
            parser.write(ini)


def loadPreset(n):

    random.seed(time.time())

    global presets_ini, gSlots, gTimeMode, bUseAbsoluteDay, bRepeat, bPlaying, SolUI, gTimeMultiplier

    section = str(n)
    if presets_ini.has_section(section):
        for i in range(0,9):
            key = '%i-weather'%i
            if presets_ini.has_option(section, key):
                wid = presets_ini.getint(section,key)
                if wid >= 0 and wid != 37:
                    gSlots[i].setWeather(wid, False)
                    
                    key = '%i-weather_list'%i
                    if presets_ini.has_option(section, key):
                        tmp = (presets_ini.get(section,key)).split(',')
                        data=[]
                        for str_tmp in tmp:
                            data.append(int(str_tmp))
                        gSlots[i].setWeatherList(data)

                    key = '%i-rain'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].setRain(presets_ini.getfloat(section,key))
                    key = '%i-rainProbability'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].setRainProbability(presets_ini.getfloat(section,key))
                    key = '%i-rainVariance'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].setRainVariance(presets_ini.getfloat(section,key))

                    key = '%i-humidity'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].humidity = presets_ini.getfloat(section,key)
                    key = '%i-fog'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].fog = presets_ini.getfloat(section,key)

                    key = '%i-tempAmbient'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].temp_ambient = presets_ini.getfloat(section,key)
                    key = '%i-tempRoad'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].temp_road = presets_ini.getfloat(section,key)

                    key = '%i-wind'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].wind_strength = presets_ini.getfloat(section,key)
                    key = '%i-windDirection'%i
                    if presets_ini.has_option(section, key):
                        gSlots[i].wind_dir = presets_ini.getfloat(section,key)
                else:
                    gSlots[i].setWeather(37,False)
            else:
                gSlots[i].setWeather(37,False)

            key = '%i-playseconds'%i
            if presets_ini.has_option(section, key):
                gSlots[i].playSeconds = presets_ini.getint(section,key)

            key = '%i-stamphour'%i
            if presets_ini.has_option(section, key):
                gSlots[i].stampHour = presets_ini.getint(section,key)
            key = '%i-stampMinute'%i
            if presets_ini.has_option(section, key):
                gSlots[i].stampMinute = presets_ini.getint(section,key)

        
        if presets_ini.has_option(section, 'useFullDay'):
            if presets_ini.getboolean(section, 'useFullDay'):
                gTimeMode = 0
        if presets_ini.has_option(section, 'timeMode'):
            gTimeMode = presets_ini.getint(section, 'timeMode')
        if presets_ini.has_option(section, 'timeMultiplier'):
            setTimeMultiplier(presets_ini.getfloat(section, 'timeMultiplier'))
        if presets_ini.has_option(section, 'repeat'):
            bRepeat = presets_ini.getboolean(section, 'repeat')
        if presets_ini.has_option(section, 'playing'):
            if gTimeMode == 1: # time span
                bPlaying = presets_ini.getboolean(section, 'playing')
                if bPlaying:
                    playSeconds = 0
                    play()
            if gTimeMode == 2: # time stamp
                pass

        unmarkWeatherInEditor()
        markWeatherInEditor()

        lastCurrent = -1
        lastNext = -1
        
        if gTimeMode < 1: # day cycle
            n_e = SolUI.getElement('Abs', '')
            if len(n_e)>0:
                n_e[0].press()
        elif gTimeMode < 2: # time span
            n_e = SolUI.getElement('Rel', '')
            if len(n_e)>0:
                n_e[0].press()
        else: # time stamp
            n_e = SolUI.getElement('Stamp', '')
            if len(n_e)>0:
                n_e[0].press()

        #resetWeatherFX()


def importPreset(filename):
    global presets_path, presets_ini, editedPreset, SolUI

    if os.path.isfile(filename):
        # clear the parser
        for section in presets_ini.sections():
            presets_ini.remove_section(section)
        presets_ini.read(filename)
        editedPreset=-1
        updatePresetButton('')

        # reset autoload checkboxes
        for i in range(0,5):
            n = SolUI.getElement('Load', '%i'%i)
            if len(n)>0:
                n[0].initValue(False)

        if presets_ini.has_option("general", "loadWithStart"):
            preset = presets_ini.getint("general", "loadWithStart")
            n = SolUI.getElement('Preset', '%i'%preset)
            if len(n)>0:
                n[0].press()
            n = SolUI.getElement('Load', '%i'%preset)
            if len(n)>0:
                n[0].initValue(True)
                   
            return True
        else:
            return False


def exportPreset(filename):
    global presets_path, presets_ini, presets_ini_defaultFile, editedPreset, SolUI

    if editedPreset>=0:
        savePreset(presets_ini, '', editedPreset)

    if presets_ini.has_section('general'):
        if presets_ini.has_option('general', 'loadWithStart'):
            presets_ini.remove_option('general', 'loadWithStart')
    else:
        presets_ini.add_section('general')

    for i in range(0,5):
        n = SolUI.getElement('Load', '%i'%i)
        if len(n)>0:
            if n[0].getValue():
                presets_ini.set('general', 'loadWithStart', str(i))
                #break
        section = str(n)
        n = SolUI.getElement('Preset', '%i'%i)
        if len(n)>0:
            s = n[0].getText()
            if s.find('Preset ') < 0:
                if presets_ini.has_section(section):
                    presets_ini.set(section, 'name', s)
            else:
                if presets_ini.has_option(section, 'name'):
                    presets_ini.remove_option(section, 'name')


    with open(filename, 'w') as ini:   
        presets_ini.write(ini)


def exportPlan(filename, n):
    if n>=0:
        plan_ini = configparser.ConfigParser(comment_prefixes=('#', ';'))
        savePreset(plan_ini, filename, -1)

        n = SolUI.getElement('Preset', '%i'%n)
        if len(n)>0:
            s = os.path.splitext(os.path.basename(filename))[0]
            n[0].setText(s)
    
def importPlan(filename, n):
    global presets_ini
    if n>=0:
        plan_ini = configparser.ConfigParser(comment_prefixes=('#', ';'))
        plan_ini.read(filename)

        if plan_ini.has_section('plan'):
            plan = plan_ini.items('plan')
        
            section = str(n)
            if presets_ini.has_section(section):
                presets_ini.remove_section(section)
            presets_ini.add_section(section)

            for tmp in plan:
                presets_ini.set(section, tmp[0], tmp[1])

            loadPreset(n)

            name = os.path.splitext(os.path.basename(filename))[0]
            n = SolUI.getElement('Preset', '%i'%n)
            if len(n)>0:
                n[0].setText(name)

            if not presets_ini.has_section(section):
                presets_ini.add_section(section)
            presets_ini.set(section, 'name', name)












def execute_SETVALUE_from_Sol(order):
    
    if 'show' in order['section']:
        if SolUI_SlotEditor.isVisible():
            if 'humidity' in order['key']:
                n = SolUI_SlotEditor.getElement("Humidity", "amount")
                if len(n)>0:
                    n[0].setForcedValue(float(order['value']), order['relative']) 
            elif 'fog' in order['key']:
                n = SolUI_SlotEditor.getElement("Fog", "amount")
                if len(n)>0:
                    n[0].setForcedValue(float(order['value']), order['relative'])

def execute_INITVALUE_from_Sol(order):
    
    global SolUI, SolUI_SlotEditor, dayPosition, SolStellar, gTimeMode, bUseAbsoluteDay, gSlots, timeUpdateFromSolController, gInitialCMWeather, timerDateUpdate

    if 'base' in order['section']:
        gInitialCMWeather[order['key']] = order['value']
        for i in range(0,9):
            gSlots[i].checkPhantomWeather()
                   
    elif 'stellar' in order['section']:
        
        if 'dayPos' in order['key']:
            pass
       
    
        elif 'DayOfTheYear' in order['key']:
            SolStellar.setDayOfTheYear(int(order['value']))
            timerDateUpdate = 0
        elif 'DaySeconds' in order['key']:
            timeUpdateFromSolController+=1
            SolStellar.setTime(int(order['value']))
            if gTimeMode==0:
                calcPlayPositionDay(SolStellar)
            elif gTimeMode==2:
                calcPlayPositionStamp()
            updateTime(SolStellar)
            timerDateUpdate = 0
        elif 'TimeZoneOffset' in order['key']:
            SolStellar.setTimeZoneOffset(int(order['value']))
            #ac.log('TimeZoneOffset: %.2f'%(SolStellar.timeZoneOffset))
        elif 'TrackCoordinatesLong' in order['key']:
            SolStellar.setLongitude(float(order['value']))
            #ac.log('TrackCoordinatesLong: %.2f'%(SolStellar.trackCoordinates[0]))
        elif 'TrackCoordinatesLat' in order['key']:
            SolStellar.setLatitude(float(order['value']))
            #ac.log('TrackCoordinatesLat: %.2f'%(SolStellar.trackCoordinates[1]))

        elif 'sunangle' in order['key']:
            n = SolUI.getElement('Stellar', 'Sun')
            if len(n)>0:
                n[0].setText('%.1f°'%order['value'])
            
        elif 'moonangle' in order['key']:
            n = SolUI.getElement('Stellar', 'Moon')
            if len(n)>0:
                n[0].setText('%.1f°'%order['value'])

        elif 'moonphase' in order['key']:
            n = SolUI.getElement('Pic', 'Moon')
            if len(n)>0:
                n[0].setTexRelative(order['value'])

    elif 'system' in order['section']:
        if 'version' in order['key']:
            update_version_check(float(order['value']))
    
    else:
        n = SolUI.getElement(order['section'], order['key'])
        for tmp in n:
            #ac.log('%s.%s, %f'%(order['section'], order['key'], order['value']))
            tmp.initValue(order['value'])
    

def execute_CMD_from_Sol(cmd):
    global SolUI, timer3
    if 'CMD1' in cmd:
        if 'ctrl' in cmd['CMD1']:
            pass

        elif 'system' in cmd['CMD1']:
            if 'CMD2' in cmd:
                if 'console' in cmd['CMD2']:
                    n = SolUI.getElement('system', 'console')
                    for tmp in n:
                        timer3 = 0.0
                        tmp.setColor(0.2,0.9,0.4,1)
                        tmp.setText("# "+cmd['CMD3'])
                elif 'clearDirty' in cmd['CMD2']:
                    SolUI.setDirty(False)
                elif 'ResetWeatherFX' in cmd['CMD2']:
                    resetWeatherFX()
                elif 'run' in cmd['CMD2']:
                    pass
        elif 'UI' in cmd['CMD1']:
            if 'CMD2' in cmd:
                if 'LockAfterUse' in cmd['CMD2']:
                    #ac.log('%s,%s'%(cmd['CMD3'], cmd['CMD4']))
                    if 'CMD3' in cmd and 'CMD4' in cmd :
                        n = SolUI.getElement(cmd['CMD3'], cmd['CMD4'])
                        for tmp in n:
                            tmp.lockUIafterUsage = True
        
    
order_executor_list = {'SET_VALUE':execute_SETVALUE_from_Sol,
                       'INIT_VALUE':execute_INITVALUE_from_Sol,
                       'CMD':execute_CMD_from_Sol,}



















def resetTransportButtons():
    global SolUI

    if gTimeMode == 1: # only in Time span mode
        n = SolUI.getElement('Transport', 'Stop')
        if len(n)>0:
            n[0].setBackgroundColor(0.5,0.1,0.1)
            n[0].setBackgroundOpacity(0.75)

        n = SolUI.getElement('Transport', 'Pause')
        if len(n)>0:
            n[0].setFlashing(False)
            n[0].setBackgroundColor(0.4,0.3,0.1)
            n[0].setBackgroundOpacity(0.75)
            
        n = SolUI.getElement('Transport', 'Play')
        if len(n)>0:
            n[0].setBackgroundColor(0.1,0.5,0.1)
            n[0].setBackgroundOpacity(0.75)
    else:
        n = SolUI.getElement('Transport', 'Stop')
        if len(n)>0:
            n[0].setBackgroundColor(0.5,0.1,0.1)
            n[0].setBackgroundOpacity(0.25)

        n = SolUI.getElement('Transport', 'Pause')
        if len(n)>0:
            n[0].setFlashing(False)
            n[0].setBackgroundColor(0.4,0.3,0.1)
            n[0].setBackgroundOpacity(0.25)
            
        n = SolUI.getElement('Transport', 'Play')
        if len(n)>0:
            n[0].setBackgroundColor(0.1,0.5,0.1)
            n[0].setBackgroundOpacity(0.25)

def stop():
    global bPlaying, bPause, playSeconds
    resetTransportButtons()
    bPlaying = False
    bPause = False
    playSeconds = 0
    if gTimeMode==1:
        calcPlayPositionTime()
        updatePlayPosition(True)

def pause():
    global bPlaying, bPause, SolUI
    resetTransportButtons()
    bPause = True
    n = SolUI.getElement('Transport', 'Pause')
    if len(n)>0:
        n[0].setBackgroundColor(0.6,0.5,0.1)
        n[0].setFlashing(True)

def play():
    global bPlaying, bPause, SolUI
    resetTransportButtons()
    n = SolUI.getElement('Transport', 'Play')
    if len(n)>0:
        n[0].setBackgroundColor(0.1,1,0.1)
    bPlaying = True
    bPause = False

def showRepeat():
    global bRepeat, gTimeMode, bUseAbsoluteDay, SolUI
    n = SolUI.getElement('Transport', 'Repeat')
    if len(n)>0:
        if bRepeat and gTimeMode==1: # only in time span mode
            n[0].setBackgroundColor(0.75,0.3,0.1)
            n[0].setBackgroundOpacity(0.75)
        else:
            n[0].setBackgroundColor(0.1,0.1,0.1)
            n[0].setBackgroundOpacity(0.25)

def updateModeButtons():
    global gTimeMode, bUseAbsoluteDay
    
    if bSol21ControllerRunning:

        n = SolUI.getElement('Abs', '')
        if len(n)>0:
            if gTimeMode==0:
                n[0].setBackgroundColor(0.9,0.1,0.1)
                n[0].setBackgroundOpacity(0.75)
            else:
                n[0].setBackgroundColor(0.1,0.1,0.1)
                n[0].setBackgroundOpacity(0.25)

        n = SolUI.getElement('Rel', '')
        if len(n)>0:
            if gTimeMode==1:
                n[0].setBackgroundColor(0.9,0.9,0.1)
                n[0].setBackgroundOpacity(0.75)
            else:
                n[0].setBackgroundColor(0.1,0.1,0.1)
                n[0].setBackgroundOpacity(0.25)

        n = SolUI.getElement('Stamp', '')
        if len(n)>0:
            if gTimeMode==2:
                n[0].setBackgroundColor(0.9,0.1,0.9)
                n[0].setBackgroundOpacity(0.75)
            else:
                n[0].setBackgroundColor(0.1,0.1,0.1)
                n[0].setBackgroundOpacity(0.25)

        if gTimeMode<1: # day cycle
            for i in range(0,9):
                n = SolUI.getElement('RPlus1', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('RMinus1', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('RPlus2', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('RMinus2', '%i'%i)
                if len(n)>0:
                    n[0].hide()
        elif gTimeMode<2: # time span
            for i in range(0,9):
                n = SolUI.getElement('RPlus1', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('RMinus1', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('RPlus2', '%i'%i)
                if len(n)>0:
                    n[0].show()
                n = SolUI.getElement('RMinus2', '%i'%i)
                if len(n)>0:
                    n[0].show()
        else: # time stamp
            for i in range(0,9):
                n = SolUI.getElement('RPlus1', '%i'%i)
                if len(n)>0:
                    n[0].show()
                n = SolUI.getElement('RMinus1', '%i'%i)
                if len(n)>0:
                    n[0].show()
                n = SolUI.getElement('RPlus2', '%i'%i)
                if len(n)>0:
                    n[0].show()
                n = SolUI.getElement('RMinus2', '%i'%i)
                if len(n)>0:
                    n[0].show()


        if gTimeMode<2: # day cycle + time span mode
            for i in range(0,9):
                n = SolUI.getElement('SDate', '%i'%i)
                if len(n)>0:
                    n[0].hide()
                n = SolUI.getElement('Aslot', '%i'%i)
                if len(n)>0:
                    n[0].show()
        else: # time stamp mode
            for i in range(0,9):
                n = SolUI.getElement('SDate', '%i'%i)
                if len(n)>0:
                    n[0].show()
                n = SolUI.getElement('Aslot', '%i'%i)
                if len(n)>0:
                    n[0].hide()
















# UI Handle function

def spinnerHandle(spinner):
    global gWiperMode
    if 'Wipers' in spinner.section:
        gWiperMode = spinner.getValue()
        if gWiperMode == -2:
            # no control
            spinner.setText("external Wiper control")
            resetWiper()
        elif gWiperMode == -1:
            # automatic control
            spinner.setText("automatic Wiper control")
            updateWipers(0)
        else:
            spinner.setText("manual Wiper control")
            for i in range(0,4):
                # to be sure to have always the right wiper speed, repeat for maximum steps
                updateWipers(0)
    
def checkboxHandle(checkbox):
    #SolCom.set_value(checkbox.section, checkbox.key, checkbox.getValue())
    global presets_ini
    if 'Load' in checkbox.section:
        #if presets_ini.has_section(checkbox.key):
        for i in range(0,5):
            n = SolUI.getElement('Load', '%i'%i)
            if len(n)>0:
                if n[0].key not in checkbox.key:
                    n[0].initValue(0)
    
def buttonHandle(button):
    global SolCom, SolUI_SlotEditor, editedSlot, editedPreset, gTimeMode, bUseAbsoluteDay, bRepeat, gSlots, bPlaying, bPause, playSeconds
    global presets_path, presets_ini, presets_ini_defaultFile, plans_path, timerDateUpdate, gTimeMultiplier
    #t = [button.section]
    #if button.key:
    #    t.append(button.key)
        #ac.log('%s'%button.key)
    #SolCom.send_command(t)
    
    if 'ResetSol' in button.section:
        resetWeatherFX()

    elif 'Slot' in button.section:
        slot = int(button.key)
        if editedSlot >= 0:
            gSlots[editedSlot].setEditing(False)
        if editedSlot == slot and SolUI_SlotEditor.isVisible():
            SolUI_SlotEditor.forceVisible(False)
            SolUI_SlotEditor.stop()
            editedSlot = -1
        else:
            editedSlot = slot
            positioningSlotEditor()
            SolUI_SlotEditor.start()
            SolUI_SlotEditor.forceVisible(True)
            gSlots[editedSlot].setEditing(True)

    elif 'Abs' in button.section:
        gTimeMode = 0
        updateTimeSlotButtons()
        resetTransportButtons()
        updateModeButtons()
        if bPlaying:
            pause()
    elif 'Rel' in button.section:
        gTimeMode = 1
        updateTimeSlotButtons()
        resetTransportButtons()
        updateModeButtons()
        if bPause:
            pause() #recall pause to update buttons
        elif bPlaying:
            play() #recall play to update buttons
    elif 'Stamp' in button.section:
        gTimeMode = 2
        updateTimeSlotButtons()
        resetTransportButtons()
        updateModeButtons()
        if bPlaying:
            pause()
        # reset AC-time to first slot
        resetACTimeToSlotStampTime(0)
        
    elif 'Preset' in button.section:
        updatePresetButton(button.key)
        if editedPreset>=0:
            # save time multi just once
            savePreset(presets_ini, presets_ini_defaultFile, editedPreset)
        stop()
        editedPreset = int(button.key)
        loadPreset(editedPreset)
        #ac.log('edited preset: %i'%editedPreset)

    elif 'Aslot' in button.section:
        shiftTimeToSlot(int(button.key))
    elif 'Rslot' in button.section:
        if gTimeMode==1:
            shiftPlayPositionTime(int(button.key))
        elif gTimeMode==2:
            resetACTimeToSlotStampTime(int(button.key))

    elif 'RPlus1' in button.section:
        if gTimeMode==2:
            slot = int(button.key)
            if gSlots[slot].stampHour<23:
                gSlots[slot].stampHour+=1
            else:
                gSlots[slot].stampHour=0
            updateTimeSlotButtons()
    elif 'RMinus1' in button.section:
        if gTimeMode==2:
            slot = int(button.key)
            if gSlots[slot].stampHour>0:
                gSlots[slot].stampHour-=1
            else:
                gSlots[slot].stampHour=23
            updateTimeSlotButtons()

    elif 'RPlus2' in button.section:
        if gTimeMode==1:
            slot = int(button.key)
            if gSlots[slot].playSeconds<900:
                gSlots[slot].playSeconds+=60
            else:
                gSlots[slot].playSeconds+=300
            updateTimeSlotButtons()
        elif gTimeMode==2:
            slot = int(button.key)
            if gSlots[slot].stampMinute<55:
                gSlots[slot].stampMinute+=5
            else:
                gSlots[slot].stampMinute=0
                gSlots[slot].stampHour+=1
                if gSlots[slot].stampHour>23:
                    gSlots[slot].stampHour=0
            updateTimeSlotButtons()
            
    elif 'RMinus2' in button.section:
        if gTimeMode==1:
            slot = int(button.key)
            if gSlots[slot].playSeconds>60:
                if gSlots[slot].playSeconds<=900:
                    gSlots[slot].playSeconds-=60
                else:
                    gSlots[slot].playSeconds-=300
                updateTimeSlotButtons()
        elif gTimeMode==2:
            slot = int(button.key)
            if gSlots[slot].stampMinute>0:
                gSlots[slot].stampMinute-=5
            else:
                gSlots[slot].stampMinute=55
                gSlots[slot].stampHour-=1
                if gSlots[slot].stampHour<0:
                    gSlots[slot].stampHour=23
            updateTimeSlotButtons()
    
    elif 'Time' in button.section:
        offset = int(button.key) * 60
        ac.ext_weatherTimeOffset(offset)
        timerDateUpdate = 0
        
    elif 'Date' in button.section:
        offset = int(button.key) 
        ac.ext_weatherTimeOffset(offset*86400)
        timerDateUpdate = 0

    elif 'TimMult' in button.section:
        offset = int(button.key) 
        gTimeMultiplier += offset
        setTimeMultiplier(gTimeMultiplier)

    elif 'TimReset' in button.section:
        resetTimeDate()

    elif 'Transport' in button.section:
        if gTimeMode > 0:
            if 'Stop' in button.key:
                stop()
            elif 'Pause' in button.key:
                pause()
            elif 'Play' in button.key:
                play()
            elif 'Repeat' in button.key and gTimeMode==1:
                bRepeat = not bRepeat
                showRepeat()
                updatePlayPosition(True)

    elif 'LoadSave' in button.section:
        
        tmp = ac.getPosition(SolUI.window)
        pos = [max(0, tmp[0]+button.x - 150), tmp[1]+button.y+button.height]

        if SolUI_SlotEditor.isVisible():
            SolUI_SlotEditor.forceVisible(False)
            SolUI_SlotEditor.stop()
            editedSlot = -1
        
        if 'ImportPreset' in button.key:
            Sol_LoadSave_Dialog_Open(presets_path,
            'Load',
            pos,
            importPresetCallback,
            Titel='Import presets', DoButtonText='Import', ChangeRoot=False)

        if 'ExportPreset' in button.key:
            Sol_LoadSave_Dialog_Open(presets_path,
            'Save',
            pos,
            exportPresetCallback,
            Titel='Export presets', DoButtonText='Export', Suffix='ini', CheckOverwrite=True, ChangeRoot=False)
        
        if 'ImportPlan' in button.key:
            if editedPreset>=0:
                Sol_LoadSave_Dialog_Open(plans_path,
                'Load',
                pos,
                importPlanCallback,
                Titel='Import single plan', DoButtonText='Import', ChangeRoot=False)
            else:
                Sol_Confirm_Dialog_Open('No preset slot selected!', '', SingleMessage=True, Position=pos)


        if 'ExportPlan' in button.key:
            if editedPreset>=0:
                Sol_LoadSave_Dialog_Open(plans_path,
                'Save',
                pos,
                exportPlanCallback,
                Titel='Export single plan', DoButtonText='Export', Suffix='ini', CheckOverwrite=True, ChangeRoot=False)
            else:
                Sol_Confirm_Dialog_Open('No preset slot selected!', '', SingleMessage=True, Position=pos)


def EditorspinnerHandle(spinner):
    global SolCom, editedSlot, gSlots
    if 'Rain' in spinner.section:
        gSlots[editedSlot].virginRain = False
        if 'amount' in spinner.key:
            gSlots[editedSlot].setRain(spinner.getValue())
        if 'probability' in spinner.key:
            gSlots[editedSlot].setRainProbability(spinner.getValue())
        if 'variance' in spinner.key:
            gSlots[editedSlot].setRainVariance(spinner.getValue())
    
    if 'Humidity' in spinner.section:
        gSlots[editedSlot].virginHumidity = False
        if 'amount' in spinner.key:
            gSlots[editedSlot].humidity = spinner.getValue()
            if gSlots[editedSlot].isCurrent():
                SolCom.init_value('current', 'humidity', gSlots[editedSlot].humidity)
    if 'Fog' in spinner.section:
        gSlots[editedSlot].virginFog = False
        if 'amount' in spinner.key:
            gSlots[editedSlot].fog = spinner.getValue()
            if gSlots[editedSlot].isCurrent():
                SolCom.init_value('current', 'fog', gSlots[editedSlot].fog)

    elif 'Wind' in spinner.section:
        gSlots[editedSlot].virginWind = False
        if 'strength' in spinner.key:
            gSlots[editedSlot].wind_strength = spinner.getValue()
        elif 'direction' in spinner.key:
            gSlots[editedSlot].wind_dir = spinner.getValue() 

    elif 'Temp' in spinner.section:
        if 'ambient' in spinner.key:
            gSlots[editedSlot].setTempAmbient(spinner.getValue())
        elif 'road' in spinner.key:
            gSlots[editedSlot].temp_road = spinner.getValue()
                
    updatePlayPosition(False)

def EditorbuttonHandle(button):
    global bEditorMultiSelectionMode, gEditorSelectedWeather, gSlots
    global weather_params, SolUI_SlotEditor, editedSlot, lastNext, lastCurrent
    #t = [button.section]
    #if button.key:
    #    t.append(button.key)
        #ac.log('%s'%button.key)
    #SolCom.send_command(t)
    if 'Weather' in button.section:
        wid = int(button.key)

        if weather_params[wid][1] >=40 and weather_params[wid][1] <= 42: # random buttons
            if wid == gEditorSelectedWeather: # if pressed twice
                bEditorMultiSelectionMode = not bEditorMultiSelectionMode
            else:
                gSlots[editedSlot].setWeather(wid, True)
                gEditorSelectedWeather = wid
                bEditorMultiSelectionMode = False
                n = SolUI_SlotEditor.getElement('Weather', str(wid))
                if len(n)>0:
                    #if len(gSlots[editedSlot].getWeatherList())==0:
                    data = n[0].getBundledData()
                    if len(data)>0:
                        gSlots[editedSlot].setWeatherList(data)

            if not bEditorMultiSelectionMode: #leaving Multiselect
                # force recalculation of random Weather
                if gSlots[editedSlot].isCurrent():
                    lastCurrent=-1
                if gSlots[editedSlot].isNext():
                    lastNext=-1

            unmarkWeatherInEditor()
            markWeatherInEditor()  
        else:
            if bEditorMultiSelectionMode:
                if (weather_params[wid][1] >= 0 and weather_params[wid][1] <= 32) or weather_params[wid][1]==50 or weather_params[wid][1]==100:
                    data = gSlots[editedSlot].getWeatherList()
                    if wid in data:
                        data.remove(wid)
                    else:
                        data.append(wid)
                    updateMultiSelection()

                    # make a copy of the data in the buttons bundle data to recover it for the next usage
                    n = SolUI_SlotEditor.getElement('Weather', str(gEditorSelectedWeather))
                    if len(n)>0:
                        n[0].setBundledData(data)
                else:
                    unmarkWeatherInEditor()
                    bEditorMultiSelectionMode = False

            if not bEditorMultiSelectionMode:
                gSlots[editedSlot].setWeather(wid, True)
                
                gEditorSelectedWeather = -1
                unmarkWeatherInEditor()
                markWeatherInEditor()
            
            if weather_params[wid][1]==666: #empty
                bEditorMultiSelectionMode = False
                gEditorSelectedWeather = -1
                gSlots[editedSlot].setWeather(37, False)
                gSlots[editedSlot].press() #trigger closing editor

            if gSlots[editedSlot].isCurrent():
                lastCurrent=-1
            if gSlots[editedSlot].isNext():
                lastNext=-1

        updatePlayPosition(True)    
        #ac.log('%i, %i'%(bEditorMultiSelectionMode, gEditorSelectedWeather))

def pageHandle(page):
    pass

def dirtyHandle(state):
    # will be called once if UI is changing its dirty flag
    n = SolUI.getElement('SaveStandard', '')
    for tmp in n:
        tmp.setFlashing(state)

def lockingHandle(UI):
    global timerButtonWaitTime
    if UI.isLocked():
        timerButtonWaitTime = 0.0  

def visibleHandle(UI):
    global SolCom, editedSlot, SolUI_SlotEditor, gTimeMode, bUseAbsoluteDay, bPlaying

    if UI.isVisible():
        SolCom.run()
        if editedSlot >= 0:
            SolUI_SlotEditor.forceVisible(True)
        else:
            SolUI_SlotEditor.forceVisible(False)
    else:
        if gTimeMode==1 and not bPlaying:
            SolCom.stop()
        SolUI_SlotEditor.forceVisible(False)

def EditorvisibleHandle(UI):
    global editedSlot, gSlots, weather_params, gEditorSelectedWeather, bEditorMultiSelectionMode
    if UI.isVisible():
        if editedSlot >= 0:
            positioningSlotEditor()
            
            if gSlots[editedSlot].getWeather() >= 0:
                if weather_params[gSlots[editedSlot].getWeather()][1] >=40 and weather_params[gSlots[editedSlot].getWeather()][1] <= 42:
                    gEditorSelectedWeather = gSlots[editedSlot].getWeather()
                    bEditorMultiSelectionMode = False
                else:
                    gEditorSelectedWeather = -1
            else:
                gEditorSelectedWeather = -1
            
            unmarkWeatherInEditor()
            markWeatherInEditor()
        else:
            UI.forceVisible(False)
            

def scaleHandle(UI):
    global SolUI_SlotEditor, Sol_LoadSave_UI
    #clone the scale
    SolUI_SlotEditor.setScale(UI.getScale())
    Sol_LoadSave_UI.setScale(UI.getScale())




def importPresetCallback(**args):
    if 'value' in args:
        if len(args['value'])>0:
            importPreset(args['value'])
        else: #Cancel
            pass
    
def exportPresetCallback(**args):
    if 'value' in args:
        if len(args['value'])>0:
            exportPreset(args['value'])
        else: #Cancel
            pass
    
def importPlanCallback(**args):
    global SolUI, editedPreset
    if 'value' in args:
        if len(args['value'])>0:
            if editedPreset>=0:
                importPlan(args['value'], editedPreset)
        else: #Cancel
            pass

def exportPlanCallback(**args):
    global SolUI, editedPreset
    if 'value' in args:
        if len(args['value'])>0:
            if editedPreset>=0:
                exportPlan(args['value'], editedPreset)
        else: #Cancel
            pass


















def acMain(ac_version):

    global cwd, appWindow, dialogWindow, slotEditorWindow, order_executor_list, SolCom, SolUI, settings_ini, settings_ini_path, initSlot
    global gSlots, slotsCount, gWiperMode, gTimeMultiplier, gOriginalTimeMultiplier, bTimeMultiControl


    
    SolCom.set_order_executor(order_executor_list)
    # Always start communication to initialize to UI
    SolCom.run()

    #Initialize font
    ac.initFont(0, "Roboto", 1, 1)

    if bTimeMultiControl:
        gOriginalTimeMultiplier = ac.ext_getWeatherTimeMult()
    else:
        gOriginalTimeMultiplier = 1
    gTimeMultiplier = gOriginalTimeMultiplier

    #App window
    appWindow = ac.newApp("Sol Planner")
    ac.setTitle(appWindow, "Sol Planner " + version)
    ac.setTitlePosition(appWindow,-15,0)
    ac.drawBorder(appWindow, 1)
    ac.setIconPosition(appWindow, 0, -10000)
    SolUI.setWindow(appWindow, 950, 255, settings_ini)


    x=0; y=25
    SolUI.newTitel(Section='Label', Key='version', Page=1, Text='\nSol 2.1 controller not running!\n\nPlease select \"Sol 2.1\" as controller script in\nCM: Settings -> Custom Shaders Patch -> weatherFX\n\nController scripts do not work in replays or online sessions!', FontSize=20, Color=[1.0,0.3,0.3,1], customPos=[x,y])


    x=130; y=10
    SolUI.newTexButton('LoadSave', Key='ImportPreset', Hide=True, Page=1,  OffTex='%s/tex/importpreset.png'%cwd, FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[40,40])
    x=175
    SolUI.newTexButton('LoadSave', Key='ExportPreset', Hide=True, Page=1,  OffTex='%s/tex/exportpreset.png'%cwd, FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[40,40])

    x=240
    SolUI.newTexButton('LoadSave', Key='ImportPlan', Hide=True, Page=1,  OffTex='%s/tex/importplan.png'%cwd, FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[40,40])
    x=285
    SolUI.newTexButton('LoadSave', Key='ExportPlan', Hide=True, Page=1,  OffTex='%s/tex/exportplan.png'%cwd, FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[40,40])


    x=413; y=30
    SolUI.newLabel(Section='Stellar', Key='Sun', Hide=True,  Page=1, Text='', FontSize=16, customPos=[x,y], customDim=[40,24])

    x=498
    SolUI.newLabel(Section='Stellar', Key='Moon', Hide=True, Page=1, Text='', FontSize=16, customPos=[x,y], customDim=[40,24])
    
    x=380; y=31
    SolUI.newPictureSeq(['%s/tex/sun.png'%cwd], Section='Pic', Key='Sun', Hide=True, Page=1, customPos=[x,y], customDim=[23,23])
    
    x=465
    moon_list = []
    for i in range(0,7):
        moon_list.append('%s/tex/moon%i.png'%(cwd,i))
    SolUI.newPictureSeq(moon_list, Section='Pic', Key='Moon', Hide=True, Page=1, customPos=[x,y], customDim=[23,23])
    
    
    
    #Wiper control
    x=580; y=10
    wiper = SolUI.newSpinner("Wipers", "control", "", [-2,4], False, 0, Page=1, Hide=True, customPos=[x,y], customDim=[150,32])
    #wiper.initValue(gWiperMode)



    # TIME
    x=780; y=38
    SolUI.newPictureSeq(['%s/tex/time-minus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x,y], customDim=[48,24])
    keys=[360,60,10,1]
    for i in range(0,4):
        SolUI.newButton('Time', Key=str(keys[i]*-1),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[12,24])
        x+=12
    x+=34
    SolUI.newLabel(Section='Time', Key='Label', Page=1, Hide=True, Text='23:59', FontSize=20, FontAlignment='center', customPos=[x,y-4], customDim=[80,24])
    x+=34
    SolUI.newPictureSeq(['%s/tex/time-plus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x,y], customDim=[48,24])
    for i in range(3,-1,-1):
        SolUI.newButton('Time', Key=str(keys[i]),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[12,24])
        x+=12

    # DATE
    x=780; y=68
    SolUI.newPictureSeq(['%s/tex/time-minus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x+6,y], customDim=[42,24])
    keys=[365,30,1]
    for i in range(0,3):
        SolUI.newButton('Date', Key=str(keys[i]*-1),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[16,24])
        x+=16
    x+=34
    SolUI.newLabel(Section='Date', Key='Label', Page=1, Hide=True, Text='01.01.\n2000', FontSize=16, FontAlignment='center', customPos=[x,y-8], customDim=[80,24])
    x+=34
    SolUI.newPictureSeq(['%s/tex/time-plus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x,y], customDim=[42,24])
    for i in range(2,-1,-1):
        SolUI.newButton('Date', Key=str(keys[i]),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[16,24])
        x+=16

        
    #TIME MULTIPLIER
    if bTimeMultiControl:
        x=780; y=105
        SolUI.newButton('TimReset', Key='TimReset', Text='R  E  S  E  T', FontSize=14,  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[164,20])
    
        x=780; y=130
        SolUI.newPictureSeq(['%s/tex/time-minus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x+6,y], customDim=[42,24])
        keys=[100,10,1]
        for i in range(0,3):
            SolUI.newButton('TimMult', Key=str(keys[i]*-1),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[16,24])
            x+=16
        x+=34
        SolUI.newLabel(Section='TimMult', Key='Label', Page=1, Hide=True, Text='%ix'%gTimeMultiplier, FontSize=20, FontAlignment='center', customPos=[x,y-4], customDim=[80,24])
        x+=34
        SolUI.newPictureSeq(['%s/tex/time-plus.png'%cwd], Section='Pic', Page=1, Hide=True, Opacity=0.75, customPos=[x,y], customDim=[42,24])
        for i in range(2,-1,-1):
            SolUI.newButton('TimMult', Key=str(keys[i]),  Page=1, Hide=True, BackgroundOpacity=0.0, customPos=[x,y], customDim=[16,24])
            x+=16

        x=860; y+=32
        SolUI.newLabel(Section='PlanDuration', Key='Label', Hide=True,  Page=1, Text='Plan Duration', FontSize=14, FontAlignment='center', customPos=[x,y], customDim=[120,20])
        y+=21
        SolUI.newLabel(Section='PlanDuration', Key='Text', Hide=True,  Page=1, Text='', FontSize=16, FontAlignment='center', customPos=[x,y], customDim=[120,20])

    
    # initialize the slots after UI-window is ready 
    for i in range(0,slotsCount):
        gSlots.append(Slot(i))

    x=-8; y=68; b=148
    for i in range(0,5):
        SolUI.newCheckbox('Load', '%i'%i, '', False, Page=1, Hide=True, FontSize=18, Color=[0.8,0.8,0.8,1], customPos=[x+i*b,y], customDim=[25,25], showValueComparisonSystem=False)
        SolUI.newButton('Preset', Key='%i'%i,  Page=1, Hide=True, Text='Preset %i'%(i+1), MaxShownCharacters=15, FontSize=14, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x+42+i*b,y], customDim=[b-35,24]) 
        

    ANames = ['Night','Dawn','Sunrise','Morning','Noon','Afternoon','Evening','Sunset','Dusk']
    for i in range(0,slotsCount):
        SolUI.newButton('Aslot', Key='%i'%i,  Page=1, Hide=True, Text=ANames[i], FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], customPos=[25+i*80,105], customDim=[80,25])
        SolUI.newLabel(Section='SDate', Key='%i'%i,  Page=1, Hide=True, FontSize=16, FontAlignment='center', Color=[0.6,0.6,0.6,1], BackgroundColor=[0.10,0.10,0.10], customPos=[65+i*80,105], customDim=[80,25])
        
        SolUI.newButton('Rslot', Key='%i'%i,  Page=1, Hide=True, Text='', FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], customPos=[40+i*80,130], customDim=[50,25])
        SolUI.newTexButton('RPlus1', Key='%i'%i,  OffTex='%s/tex/plus.png'%cwd,  Page=1, Hide=True, customPos=[25+i*80,130], customDim=[15,13])
        SolUI.newTexButton('RMinus1', Key='%i'%i, OffTex='%s/tex/minus.png'%cwd, Page=1, Hide=True, customPos=[25+i*80,142], customDim=[15,13])
        SolUI.newTexButton('RPlus2', Key='%i'%i,  OffTex='%s/tex/plus.png'%cwd,  Page=1, Hide=True, customPos=[90+i*80,130], customDim=[15,13])
        SolUI.newTexButton('RMinus2', Key='%i'%i, OffTex='%s/tex/minus.png'%cwd, Page=1, Hide=True, customPos=[90+i*80,142], customDim=[15,13])
        SolUI.newButton('Slot',  Key='%i'%i,  Page=1, Hide=True, Text='%i'%(i+1), FontSize=12, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], customPos=[25+i*80,155], customDim=[80,100])

    SolUI.newTexButton('Transport', Key='Repeat',  Page=1, Hide=True, OffTex='%s/tex/repeat.png'%cwd,  customPos=[745,130], customDim=[25,25])

    SolUI.newTexButton('Transport', Key='Stop',  Page=1, Hide=True, OffTex='%s/tex/stop.png'%cwd,  customPos=[780,215], customDim=[55,40])
    SolUI.newTexButton('Transport', Key='Pause', Page=1, Hide=True, OffTex='%s/tex/pause.png'%cwd, customPos=[835,215], customDim=[55,40])
    SolUI.newTexButton('Transport', Key='Play',  Page=1, Hide=True, OffTex='%s/tex/play.png'%cwd,  customPos=[890,215], customDim=[55,40])
    resetTransportButtons()

    absButton = SolUI.newTexButton('Abs', Page=1, Hide=True, OffTex='%s/tex/fullday.png'%cwd,  customPos=[0,105], customDim=[25,25])
    SolUI.newTexButton('Rel', Page=1, Hide=True, OffTex='%s/tex/timed.png'%cwd,  customPos=[0,130], customDim=[25,25])
    SolUI.newTexButton('Stamp', Page=1, Hide=True, OffTex='%s/tex/stamped.png'%cwd,  customPos=[0,155], customDim=[25,25])

    x=780; y=2
    SolUI.newButton('ResetSol', Key='',  Page=1, Text='reset Sol', FontSize=12, Color=[1.0,0.2,0.2,1], BackgroundColor=[0.50,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[60,18]) 
        


    SolUI.setSpinnersCallback(spinnerHandle)
    SolUI.setCheckboxesCallback(checkboxHandle)
    SolUI.setButtonsCallback(buttonHandle)
    SolUI.setPagesCallback(pageHandle)
    #SolUI.setDirtyCallback(dirtyHandle)
    SolUI.setLockingCallback(lockingHandle)
    SolUI.setVisibleCallback(visibleHandle)
    SolUI.setScaleCallback(scaleHandle)

    SolUI.start()

    
    slotEditorWindow = ac.newApp("Sol SlotEditor")
    ac.setTitle(slotEditorWindow, "")
    ac.drawBorder(slotEditorWindow, 1)
    ac.setIconPosition(slotEditorWindow, 0, -10000)
    ac.setPosition(slotEditorWindow, 0,0)
    SolUI_SlotEditor.setWindow(slotEditorWindow, 730, 370, 0)
    SolUI_SlotEditor.setScale(SolUI.getScale())

    y=5; 
    w_width = 60
    seq = [ 
    [33, 34, 35, 36, 38, 37], #functional
    [15, 16, 17, 31, 18, 19], #norain
    [23, 21, 20, 25, 24, 22], #fogs
    [3, 4, 5, 6, 7, 8], #rain
    [0, 1, 2, 26, 27, 28], #thunder
    [9, 10, 11, 12, 13, 14], #snow
    ]
    for tmp in seq:
        x=0
        for i in tmp:
            SolUI_SlotEditor.newTexButton('Weather', Key='%i'%i, Page=1, OffTex='%s/tex/%i.png'%(cwd,weather_params[i][1]), Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], customPos=[x,y], customDim=[w_width,w_width])
            x+=w_width
        y+=w_width

    n = SolUI_SlotEditor.getElement('Weather', '34') #random dry
    if len(n)>0:
        n[0].setBundledData([15,16,17,18,19,31,3,6,23,25,26])

    n = SolUI_SlotEditor.getElement('Weather', '35') #random rainy
    if len(n)>0:
        n[0].setBundledData([18,19,26,3,4,5,6,7,8])

    n = SolUI_SlotEditor.getElement('Weather', '36') #random bad
    if len(n)>0:
        n[0].setBundledData([0,1,2,26,27,28,18,29])
    
    for i in range(0,slotsCount):
        n = SolUI.getElement('Slot', str(i))
        if len(n)>0:
            gSlots[i].setButton(n[0])

    
    SolUI_SlotEditor.newSeparator(customPos=[364,5],   customDim=[2,360])
    
    x=370; y=5
    SolUI_SlotEditor.newLabel(Section="Slot", Key="Properties", Text="", FontSize=14, Color=[0.75,0.75,0.75,1], customPos=[x+0,y])
    y+=20
    SolUI_SlotEditor.newLabel(Section="Slot", Key="Weather", Text="", FontSize=18, Color=[1,1,1,1], customPos=[x+0,y])

    x=370; y=60
    y+=10
    SolUI_SlotEditor.newSpinner("Rain","amount",     "rain amount",[0,1],True,0, customPos=[x+0,y],   customDim=[360,0])
    y+=50
    SolUI_SlotEditor.newSpinner("Rain","probability","probability",[0,1],True,0, customPos=[x+0,y],   customDim=[175,0])
    SolUI_SlotEditor.newSpinner("Rain","variance",   "variance",   [0,1],True,0, customPos=[x+185,y], customDim=[170,0])
    
    y+=50
    SolUI_SlotEditor.newSeparator(customPos=[x+0,y],   customDim=[360,2])

    y+=10
    SolUI_SlotEditor.newSpinner("Humidity","amount", "humidity",[0,1],True,0, customPos=[x+0,y],   customDim=[175,0])
    SolUI_SlotEditor.newSpinner("Fog",     "amount", "fog",     [0,1],True,0, customPos=[x+185,y], customDim=[170,0])
    
    y+=50
    SolUI_SlotEditor.newSeparator(customPos=[x+0,y],   customDim=[360,2])

    y+=15
    SolUI_SlotEditor.newSpinner("Wind","strength","wind strength",  [0,120], False, 0, customPos=[x+0,y],   customDim=[175,0])
    SolUI_SlotEditor.newSpinner("Wind","direction","wind direction",[0,360], False, 0, customPos=[x+185,y], customDim=[175,0])

    y+=50
    SolUI_SlotEditor.newSpinner("Temp","ambient","ambient temp.",  [0,40], False, 0, customPos=[x+0,y],   customDim=[175,0])
    SolUI_SlotEditor.newSpinner("Temp","road","road temp.",        [0,80], False, 0, customPos=[x+185,y], customDim=[175,0])


    SolUI_SlotEditor.setSpinnersCallback(EditorspinnerHandle)
    SolUI_SlotEditor.setButtonsCallback(EditorbuttonHandle)
    SolUI_SlotEditor.setVisibleCallback(EditorvisibleHandle)

    SolUI_SlotEditor.forceVisible(0)

    absButton.press()
    updatePresetButton('')

    if importPreset(presets_ini_defaultFile):
        # a preset was set to autoload
        initSlot = -1
    else:
        gSlots[initSlot].setWeather(38, True) # set "CM" as weather


    #Initialize Dialogs
    Sol_Dialogs_setMasterUI(SolUI)
    Sol_LoadSave_Dialog_INIT(settings_ini)

    if settings_ini.has_section("Wipers"):
        if settings_ini.has_option("Wipers", "Mode"): 
            gWiperMode = settings_ini.getint("Wipers", "Mode")
    wiper.initValue(gWiperMode, CallHandle=True)


    #Render callback
    ac.addRenderCallback(appWindow, onFormRender)
    ac.addRenderCallback(slotEditorWindow, onEditorFormRender)

    #force to load the config
    #SolCom.send_command('LoadStandard')

    
    # support live app reloading
    # force the controller to send its version !
    SolCom.send_command(['system', "check_version"])













def acUpdate(deltaT):

    global appWindow, SolCom, SolUI, SolStellar

    global app, timer1, timer2, timer3, timer4, timerDateUpdate, timerStart, timerButtonWaitTime, uiLockTime, dayPosition, timeUpdateFromSolController
    global bPlaying, bPause, playSeconds, ACCSPDate, gForceSlotStampTime, gTimeMultiplier, bTimeMultiControl, SolStellar

    #ac.ext_setWeatherTimeMult(360)

    if bPlaying and not bPause:
        playSeconds += deltaT
        #ac.log('playSeconds: %f'%playSeconds)
    
    if timer1 < 100: timer1 += deltaT
    if timer2 < 100: timer2 += deltaT
    if timer3 < 100: timer3 += deltaT
    if timer4 < 100: timer4 += deltaT
    if timerButtonWaitTime < 100: timerButtonWaitTime += deltaT
    if timerDateUpdate < 10: timerDateUpdate += deltaT

    if timer1 > 0.25:
        timer1=0.0

        #if SolUI.isVisible() or timerStart < 5:
        SolCom.update()

        if bPlaying:
            pass


        # check if something is in the temporary config memory
        # It means, some UI Element was used and changed
        # This is needed to get the dirty flag after a wfx reset
        # But we are not doing this every cycle - thats not needed
        if not SolUI.getDirty():
            msg = ac.ext_loadLua("sol.TempPlanner__")
            if msg and len(msg)>0:
                SolUI.setDirty(True)

        if editedSlot >= 0:
            positioningSlotEditor()
            n = SolUI_SlotEditor.getElement("Rain","amount")
            if len(n)>0:
                n[0].setForcedValue( gSlots[editedSlot].rainResult, True)

    if timer2 > 1:
        timer2=0.0
 
        # constantly check if UI, to react on app page cycles (visibility changes)
        SolUI.update()
        SolUI_SlotEditor.update()
        if editedSlot<0:
            if SolUI_SlotEditor.isVisible():
                SolUI_SlotEditor.forceVisible(False)
        else:
            if SolUI.isVisible() and not SolUI_SlotEditor.isVisible():
                SolUI_SlotEditor.forceVisible(True)
        
        if gTimeMode==1:
            calcPlayPositionTime()

    if timerDateUpdate < 1:

        #timerDateUpdate = 0.0
        if bTimeMultiControl:
            ac.ext_setWeatherTimeMult(gTimeMultiplier)

        # taken from sol_weather by leBluem
        sDebug = ac.ext_weatherDebugText()
        vdatetime = re.findall(r"[\d]{1,4}-[\d]{1,2}-[\d]{2} [\d]{2}:[\d]{2}:[\d]{2}",sDebug)
        if len(vdatetime)>0:
            sdatetime = str(vdatetime).replace("['", "").replace("']", "")
            # current day: 2020-05-06 16:31:25
            f = sdatetime.split(' ')
            # ac.log(str(f))
            sdate = f[0]
            stime = f[1]
            tmp = sdate.split('-')
            ACCSPDate = ACCSPDate.replace(year=int(tmp[0]), month=int(tmp[1]), day=int(tmp[2]))
            updateDate('%s.%s.\n%s'%(tmp[2],tmp[1],tmp[0]))
            tmp = stime.split(':')
            ACCSPDate = ACCSPDate.replace(hour=int(tmp[0]), minute=int(tmp[1]), second=int(tmp[2]))

            timerDateUpdate += 1

            if gForceSlotStampTime >= 0:
                resetSlotTimeStamps()
                resetACTimeToSlotStampTime(gForceSlotStampTime)
                gForceSlotStampTime = -1


    if timer3 > 10 and timer3 < 12:
        n = SolUI.getElement('system', 'console')
        for tmp in n:
            x = (timer3-10)*0.5
            tmp.setColor(0.2+0.2*x,0.9-0.5*x,0.4, 1-0.5*x)


    if timer4 > 3:
        #SolCom.update()
        timer4=0.0
        
    if timeUpdateFromSolController >= 2:
        timeUpdateFromSolController = 0 
        updatePlayPosition(False)


    if timerButtonWaitTime > uiLockTime:
        if SolUI.isLocked():
            SolUI.unlock()


    '''
    slot_pos1 = math.floor(dayPosition)
    if slot_pos1 > 8:
        slot_pos1 = 0
    slot_pos2 = slot_pos1+1
    if slot_pos2 > 8:
        slot_pos2 = 0

    gSlots[slot_pos1].update(deltaT)
    gSlots[slot_pos2].update(deltaT)
    '''
    for i in range(0,9):
        gSlots[i].update(deltaT)


    if SolUI.isVisible():
        ac.setBackgroundColor(appWindow, 0.12,0.12,0.12)
        ac.setBackgroundOpacity(appWindow, 0.85)
        
    if SolUI_SlotEditor.isVisible():
        ac.setBackgroundColor(SolUI_SlotEditor.window, 0.12,0.12,0.12)
        ac.setBackgroundOpacity(SolUI_SlotEditor.window, 0.85)

    Sol_LoadSave_Dialog_UPDATE(deltaT) 

#GL Drawing		
def onFormRender(deltaT):
    global dayPosition, playPosition, stampPosition
    global SolUI, gSlots, gTimeMode, bUseAbsoluteDay, bSol21ControllerRunning

    if SolUI.isVisible() and bSol21ControllerRunning:
        SolUI.render(deltaT)
        scale = SolUI.getScale()

        if gTimeMode<1: # day cycle
            ac.glColor4f(1,0,0,1)
            ac.glQuad((63 + 80 * dayPosition) * scale, (SolUI.height-150) * scale, 4 * scale, 25 * scale)
        elif gTimeMode<2: # time span
            ac.glColor4f(1,1,0,1)
            ac.glQuad((23 + 80 * playPosition) * scale, (SolUI.height-125) * scale, 4 * scale, 25 * scale)
        else: # time stamp
            ac.glColor4f(1,0,1,1)
            ac.glQuad((23 + 80 * stampPosition) * scale, (SolUI.height-125) * scale, 4 * scale, 25 * scale)
        
        for i in range(0,9):
            wid = gSlots[i].getWeather()
            if wid>=0:
                ac.glColor4f(1,1,1,1)
                if gSlots[i].tex>0:
                    ac.glQuadTextured(
                    (25+i*80) * scale,
                    (SolUI.height-70) * scale,
                    70 * scale,
                    70 * scale,
                    gSlots[i].tex)
                ac.glColor4f(0,0.5,1,1)
                rain = gSlots[i].getRain()
                ac.glQuad((95+i*80)*scale, (SolUI.height-60*rain)*scale, 10*scale, rain*60*scale)


def onEditorFormRender(deltaT):
    SolUI_SlotEditor.render(deltaT)

#Do on AC shutdown
def acShutdown():
    global SolUI, editedPreset, presets_ini, presets_ini_defaultFile, gWiperMode, gTimeMultiplier

    if not settings_ini.has_section("Wipers"):
        settings_ini.add_section("Wipers")

    settings_ini.set("Wipers", "Mode", "%i"%gWiperMode)

    SolUI.close()
    with open(settings_ini_path, 'w') as ini:
        settings_ini.write(ini)

    exportPreset(presets_ini_defaultFile)

    
