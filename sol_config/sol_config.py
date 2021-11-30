# Sol Config App
version = "2.0"
'''
changelog
2.0
using Sol's unified UI
added shared memory communication with Sol
added load/save
added displaying of custom config's parameter manipulation
changed to new config system
'''

import ac, acsys
import platform, os, sys
import codecs, json
import configparser
import random

if platform.architecture()[0] == "64bit":
    dllfolder = "stdlib64"
else:
    dllfolder = "stdlib"
cwd = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(cwd, dllfolder))
os.environ['PATH'] = os.environ['PATH'] + ";."
import ctypes
from ctypes import wintypes


#from sim_info_lib.sim_info import info

from sol_lib.sol_interface import *
from sol_lib.sol_UI import *

timer1 = 0.0
timer2 = 0.0
timerUpConsole = 0.0
timerBottomConsole = 0.0
timerButtonWaitTime = 100.0
timerStart = 0.0
app_size = 1.00
test = 0
SolCom = SolInterface('configAPP')
SolUI = UI("Sol Config")
uiLockTime = 1.5
debug_label = 0
sol_dummy = 'extension/weather/sol/reset_dummy.lua'
trackID = ac.getTrackName(0)
trackID = trackID.replace("-", "%-")


# SETTINGS
#
settings_default_ini_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "settings", "settings_defaults.ini")
settings_ini_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "settings", "settings.ini")
settings_ini      = configparser.ConfigParser(comment_prefixes=('#', ';'))

#read settings
if not os.path.isfile(settings_ini_path): # create empty if not there
    #with open(settings_default_ini_path, 'r', encoding='utf-8') as ini_default:
    with open(settings_ini_path, 'w', encoding='utf-8') as ini:
        ini.write('') #ini_default.read()
settings_ini.read(settings_ini_path)



# TRACK ADAPTIONS
#
track_adaptions_ini_path = "extension/weather/sol/track adaptions/track_adaption.ini"
track_adaptions_ini = configparser.ConfigParser(comment_prefixes=('#', ';'))

#read track adaptions
if not os.path.isfile(track_adaptions_ini_path): # create empty if not there
    #with open(settings_default_ini_path, 'r', encoding='utf-8') as ini_default:
    with open(track_adaptions_ini_path, 'w', encoding='utf-8') as ini:
        ini.write('') #ini_default.read()
track_adaptions_ini.read(track_adaptions_ini_path)

def saveTA(key, value):

    global track_adaptions_ini, track_adaptions_ini_path, trackID

    if not track_adaptions_ini.has_section(trackID):
        track_adaptions_ini.add_section(trackID)

    track_adaptions_ini.set(trackID, key, str(value))

    with open(track_adaptions_ini_path, 'w') as ini:
        track_adaptions_ini.write(ini)
    
    resetWeatherFX()




def resetWeatherFX():
    global sol_dummy, SolCom
    try:
        SolCom.restart()
        with codecs.open(sol_dummy, 'w', 'utf_8', errors='ignore') as f:# write
            f.write('\n')
    except:
        ac.log("SolConfigApp error: " + sol_dummy + "\n" + traceback.format_exc())
        ac.console("SolConfigApp error: " + sol_dummy + "\n" + traceback.format_exc())


def execute_SETVALUE_from_Sol(order):
    global SolUI
    n = SolUI.getElement(order['section'], order['key'])
    for tmp in n:
        tmp.setForcedValue(order['value'], order['relative'])

def execute_INITVALUE_from_Sol(order):
    global SolUI
    n = SolUI.getElement(order['section'], order['key'])
    for tmp in n:
        #ac.log('%s.%s, %f'%(order['section'], order['key'], order['value']))
        tmp.initValue(order['value'])

def execute_CMD_from_Sol(cmd):
    global SolUI, timerUpConsole, timerBottomConsole

    if 'CMD1' in cmd:
        if 'system' in cmd['CMD1']:
            if 'CMD2' in cmd:
                #ac.log('%s'%cmd['CMD2'])
                if 'UpConsole' in cmd['CMD2']:
                    n = SolUI.getElement('system', 'UpConsole')
                    for tmp in n:
                        timerUpConsole = 0.0
                        if timerBottomConsole < 10: timerBottomConsole = 10
                        tmp.setColor(0.2,0.9,0.4,1)
                        tmp.setText("# "+cmd['CMD3'])
                elif 'BottomConsole' in cmd['CMD2']:
                    n = SolUI.getElement('system', 'BottomConsole')
                    for tmp in n:
                        timerBottomConsole = 0.0
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


# UI Handle function

def spinnerHandle(spinner):
    global SolCom
    if "TA" in spinner.section and len(spinner.section) == 2:
        if not spinner.forcedValueRelative:
            pass
        else:
            saveTA(spinner.key, spinner.getValue());
    else:
        SolCom.set_value(spinner.section, spinner.key, spinner.getValue())
    
def checkboxHandle(checkbox):
    global SolCom
    SolCom.set_value(checkbox.section, checkbox.key, checkbox.getValue())
    
def buttonHandle(button):

    global SolUI, SolCom, timerButtonWaitTime, uiLockTime

    t = [button.section]
    if button.key:
        t.append(button.key)
        #ac.log('%s'%button.key)
    if timerButtonWaitTime > uiLockTime:
        SolCom.send_command(t)

def pageHandle(page):
    pass

def dirtyHandle(state):
    global SolUI
    # will be called once if UI is changing its dirty flag
    n = SolUI.getElement('SaveStandard', '')
    for tmp in n:
        tmp.setFlashing(state)

def lockingHandle(UI):
    global timerButtonWaitTime
    #ac.log('%i'%UI.isLocked())
    if UI.isLocked():
        timerButtonWaitTime = 0.0  

def visibleHandle(UI):
    if UI.isVisible():
        SolCom.run()
    else:
        SolCom.stop()

def acMain(ac_version):

    global appWindow, debug_label, order_executor_list, SolCom, SolUI, settings_ini, settings_ini_path


    








    SolCom.set_order_executor(order_executor_list)
    # Always start communication to initialize to UI
    SolCom.run()

    #Initialize font
    #ac.initFont(0, "Roboto", 1, 1)

    #App window
    appWindow = ac.newApp("Sol Config")
    ac.setTitle(appWindow, "Sol Config " + version)
    ac.drawBorder(appWindow, 1)
    ac.setIconPosition(appWindow, 0, -10000)
    

    SolUI.setWindow(appWindow, 400, 750, settings_ini)
    SolUI.newPage()
  
    
    y = 70
    SolUI.newTitel(Text='Performance Presets', FontSize=20, customPos=[ 0,y])
    y += 40
    SolUI.newButton('Performance', Key='1', Text='Low',     FontSize=16, Color=[0.5,0.8,1.0,1], BackgroundColor=[0.37,0.45,0.50], customPos=[ 15,y], customDim=[85,30])
    SolUI.newButton('Performance', Key='2', Text='Medium',  FontSize=16, Color=[0.6,1.0,0.6,1], BackgroundColor=[0.40,0.50,0.40], customPos=[110,y], customDim=[85,30])
    SolUI.newButton('Performance', Key='3', Text='High',    FontSize=16, Color=[0.8,0.8,0.6,1], BackgroundColor=[0.50,0.50,0.40], customPos=[205,y], customDim=[85,30])
    SolUI.newButton('Performance', Key='4', Text='Ultra',   FontSize=16, Color=[1.0,0.6,0.6,1], BackgroundColor=[0.50,0.40,0.40], customPos=[300,y], customDim=[85,30])
    y += 60

    y = 300
    SolUI.newTitel(Text='Nightlook Presets', FontSize=20, customPos=[ 0,y])
    y += 40
    SolUI.newButton('Night', Key='1', Text='Dark',    FontSize=16, Color=[0.4,0.4,0.4,1], BackgroundColor=[0.20,0.20,0.20], customPos=[ 15,y], customDim=[85,30])
    SolUI.newButton('Night', Key='2', Text='Medium',  FontSize=16, Color=[0.6,0.6,0.6,1], BackgroundColor=[0.30,0.30,0.30], customPos=[110,y], customDim=[85,30])
    SolUI.newButton('Night', Key='3', Text='Bright',  FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.40,0.40,0.40], customPos=[205,y], customDim=[85,30])
    SolUI.newButton('Night', Key='4', Text='Intense', FontSize=16, Color=[1.0,1.0,1.0,1], BackgroundColor=[0.50,0.50,0.50], customPos=[300,y], customDim=[85,30])
    y += 35
    #SolUI.newSeparator(customPos=[0,y])


    SolUI.newSeparator(customPos=[0,525])
    SolUI.newLabel(Section='system', Key='UpConsole', Text='#', FontSize=18, Color=[0.4,0.4,0.4,0.5], customPos=[2,530])
    SolUI.newLabel(Section='system', Key='BottomConsole', Text='#', FontSize=18, Color=[0.4,0.4,0.4,0.5], customPos=[2,555])
    SolUI.newSeparator(customPos=[0,580])


    SolUI.newButton('SaveStandard',     Text='Save Config',         LockUIafterUsage=True, FontSize=20, Color=[0.7,1.0,0.2,1], BackgroundColor=[0.40,0.50,0.30], customPos=[30,600], customDim=[340,32])
    SolUI.newButton('LoadStandard',     Text='Load Config',         LockUIafterUsage=True, FontSize=20, Color=[1.0,1.0,0.0,1], BackgroundColor=[0.50,0.50,0.25], customPos=[30,650], customDim=[340,32])
    SolUI.newButton('ResetToDefaults',  Text='reset to defaults',   LockUIafterUsage=True, FontSize=20, Color=[1.0,0.0,0.0,1], BackgroundColor=[0.50,0.25,0.25], customPos=[30,705], customDim=[340,32])
    SolUI.newPage()

    SolUI.readLayout('extension/weather/sol/config/sol_config_design.txt')

    SolUI.newTitel(Section='TA', Key='TrackName', Page=21, Text=ac.getTrackName(0), FontSize=18, Color=[0.4,1.0,0.2,1.0], customPos=[0,115])



    # draw some clones of elements
    y = 170
    n = SolUI.getElement('performance', 'use_cpu_split')
    if n: SolUI.newCheckbox(n[0].section,n[0].key,n[0].getText(),n[0].getValue(), Page=1, customPos=[10,y+18], customDim=[180,32])
   
    n = SolUI.getElement('clouds', 'distance_multiplier')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[210,y], customDim=[180,32])
    
    y += 55
    n = SolUI.getElement('clouds', 'quality')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[10,y], customDim=[180,32])

    n = SolUI.getElement('clouds', 'render_per_frame')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[210,y], customDim=[180,32])

    y += 55
    SolUI.newSeparator(Page=1, customPos=[0,y])

    y = 400
    n = SolUI.getElement('night', 'brightness_adjust')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[10,y], customDim=[380,32])
    
    y += 55
    n = SolUI.getElement('night', 'moonlight_multiplier')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[10,y], customDim=[180,32])
    
    n = SolUI.getElement('night', 'starlight_multiplier')
    if n: SolUI.newSpinner(n[0].section,n[0].key,n[0].getText(),n[0].getSpan(),n[0].bFloat,n[0].getValue(), Page=1, customPos=[210,y], customDim=[180,32])


    SolUI.setSpinnersCallback(spinnerHandle)
    SolUI.setCheckboxesCallback(checkboxHandle)
    SolUI.setButtonsCallback(buttonHandle)
    SolUI.setPagesCallback(pageHandle)
    SolUI.setDirtyCallback(dirtyHandle)
    SolUI.setLockingCallback(lockingHandle)
    SolUI.setVisibleCallback(visibleHandle)
    

    SolUI.start()
    

    ac.setBackgroundOpacity(appWindow, 0.85)

    #Render callback
    ac.addRenderCallback(appWindow, onFormRender)

    #force to load the config
    #SolCom.send_command('LoadStandard')

    

def acUpdate(deltaT):

    global appWindow, debug_label, test, SolCom, SolUI
    global app, timer1, timer2, timerButtonWaitTime, uiLockTime, timerUpConsole, timerBottomConsole, timerStart
    
    if timer1 < 100: timer1 += deltaT
    if timer2 < 100: timer2 += deltaT
    if timerUpConsole < 100: timerUpConsole += deltaT
    if timerBottomConsole < 100: timerBottomConsole += deltaT
    if timerButtonWaitTime < 100: timerButtonWaitTime += deltaT
    if timerStart < 100: timerStart += deltaT

    if timer1 > 0.25:
        timer1=0.0

        if SolUI.isVisible() or timerStart < 5:
            SolCom.update()
        
        # check if something is in the temporary config memory
        # It means, some UI Element was used and changed
        # This is needed to get the dirty flag after a wfx reset
        # But we are not doing this every cycle - thats not needed
        if not SolUI.getDirty():
            msg = ac.ext_loadLua("sol.TempConfig")
            if msg and len(msg)>0:
                SolUI.setDirty(True)
        
    if timer2 > 1:
        timer2=0.0
 
        # constantly check if UI, to react on app page cycles (visibility changes)
        SolUI.update()


    if timerButtonWaitTime > uiLockTime:
        if SolUI.isLocked():
            SolUI.unlock()

    if timerUpConsole > 10 and timerUpConsole < 12:
        n = SolUI.getElement('system', 'UpConsole')
        for tmp in n:
            x = (timerUpConsole-10)*0.5
            tmp.setColor(0.2+0.2*x,0.9-0.5*x,0.4, 1-0.5*x)

    if timerBottomConsole > 10 and timerBottomConsole < 12:
        n = SolUI.getElement('system', 'BottomConsole')
        for tmp in n:
            x = (timerBottomConsole-10)*0.5
            tmp.setColor(0.2+0.2*x,0.9-0.5*x,0.4, 1-0.5*x)


    ac.setBackgroundOpacity(appWindow, 0.85)    


#GL Drawing		
def onFormRender(deltaT):
    SolUI.render(deltaT)

#Do on AC shutdown
def acShutdown():
    SolUI.close()
    with open(settings_ini_path, 'w') as ini:
        settings_ini.write(ini)
    
