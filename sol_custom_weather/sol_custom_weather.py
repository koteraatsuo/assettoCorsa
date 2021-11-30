# Sol Custom Weather App
version = "2.0 alpha 2"
'''
alpha 2
- integrating newest Sol libs



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
timer3 = 0.0
timerButtonWaitTime = 100.0
SolCom = SolInterface('customWeatherAPP')
SolUI = UI("Sol Custom Weather")
uiLockTime = 0.5


settings_default_ini_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "settings", "settings_defaults.ini")
settings_ini_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), "settings", "settings.ini")
settings_ini      = configparser.ConfigParser(comment_prefixes=('#', ';'))


def resetWeatherFX():
    global sol_dummy, SolCom
    try:
        SolCom.restart()
        with codecs.open(sol_dummy, 'w', 'utf_8', errors='ignore') as f:# write
            f.write('\n')
    except:
        ac.log("SolCustomWeatherApp error: " + sol_dummy + "\n" + traceback.format_exc())
        ac.console("SolCustomWeatherApp error: " + sol_dummy + "\n" + traceback.format_exc())

        


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
    global SolUI, timer3
    if 'CMD1' in cmd:
        if 'system' in cmd['CMD1']:
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


# UI Handle function

def spinnerHandle(spinner):
    SolCom.set_value(spinner.section, spinner.key, spinner.getValue())
    
def checkboxHandle(checkbox):
    SolCom.set_value(checkbox.section, checkbox.key, checkbox.getValue())
    
def buttonHandle(button):
    t = [button.section]
    if button.key:
        t.append(button.key)
        #ac.log('%s'%button.key)
    SolCom.send_command(t)

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
    if UI.isVisible():
        SolCom.run()
    else:
        SolCom.stop()



def acMain(ac_version):

    global appWindow, order_executor_list, SolCom, SolUI, settings_ini, settings_ini_path


    #read settings
    if not os.path.isfile(settings_ini_path): # create empty if not there
        #with open(settings_default_ini_path, 'r', encoding='utf-8') as ini_default:
        with open(settings_ini_path, 'w', encoding='utf-8') as ini:
            ini.write('') #ini_default.read()
    settings_ini.read(settings_ini_path)




    SolCom.set_order_executor(order_executor_list)
    # Always start communication to initialize to UI
    SolCom.run()

    #Initialize font
    ac.initFont(0, "Roboto", 1, 1)

    

    #App window
    appWindow = ac.newApp("Sol Custom Weather")
    ac.setTitle(appWindow, "Sol Custom Weather " + version)
    ac.drawBorder(appWindow, 1)
    ac.setIconPosition(appWindow, 0, -10000)



    SolUI.setWindow(appWindow, 400, 750, settings_ini)
    '''
    SolUI.addSpacer(Space=75)
    SolUI.newTitel(Text='Sol Config 2.0 alpha', FontSize=30)
    SolUI.newTitel(Text='Test App of LUA<->Python Com', FontSize=20)
    SolUI.newSeparator()
    SolUI.newLabel(Text='If parameters are not set or modulated in a custom config,', FontSize=14)
    SolUI.newLabel(Text='the UI elements have standard AC look.', FontSize=14)
    SolUI.addSpacer(Space=25)
    SolUI.newLabel(Text='Modulated parameters (they are set relatively) via:', FontSize=14)
    SolUI.newLabel(Text='e.g. SOL__set_config(\'ambient\', \'sun_color_balance\', 0.5, true)', FontSize=14)
    SolUI.newLabel(Text='are drawn in blue color', FontSize=16, Color=[0.25, 0.75, 1.0, 1])
    SolUI.addSpacer(Space=25)
    SolUI.newLabel(Text='Absolutely set parameters via:', FontSize=14)
    SolUI.newLabel(Text='e.g. SOL__set_config(\'ambient\', \'sun_color_balance\', 0.5, false)', FontSize=14)
    SolUI.newLabel(Text='or', FontSize=14)
    SolUI.newLabel(Text='e.g. SOL__set_config(\'ambient\', \'sun_color_balance\', 0.5)', FontSize=14)
    SolUI.newLabel(Text='are drawn in orange color', FontSize=16, Color=[1.0, 0.5, 0, 1])
    '''

    


    

    SolUI.readLayout('extension/weather/sol/custom weather/sol_custom_weather_design.txt')

    y = 655
    SolUI.newSeparator(Page=1, customPos=[0,y])
    SolUI.newLabel(Section='system', Key='console', Text='#', Page=1, FontSize=18, Color=[0.4,0.4,0.4,0.5], customPos=[2,y+5])
    SolUI.newSeparator(Page=1, customPos=[0,y+30])


    #SolUI.newButton('SaveStandard',     Page=1, Text='Save Config',         FontSize=20, Color=[0.7,1.0,0.2,1], BackgroundColor=[0.40,0.50,0.30], customPos=[30,600], customDim=[340,32])
    #SolUI.newButton('LoadStandard',     Page=1, Text='Load Config',         FontSize=20, Color=[1.0,1.0,0.0,1], BackgroundColor=[0.50,0.50,0.25], customPos=[30,650], customDim=[340,32])
    SolUI.newButton('ResetToDefaults',  Page=1, Text='reset to defaults',   LockUIafterUsage=True, FontSize=20, Color=[1.0,0.0,0.0,1], BackgroundColor=[0.50,0.25,0.25], customPos=[30,705], customDim=[340,32])


    SolUI.setSpinnersCallback(spinnerHandle)
    SolUI.setCheckboxesCallback(checkboxHandle)
    SolUI.setButtonsCallback(buttonHandle)
    SolUI.setPagesCallback(pageHandle)
    #SolUI.setDirtyCallback(dirtyHandle)
    SolUI.setLockingCallback(lockingHandle)
    SolUI.setVisibleCallback(visibleHandle)

    SolUI.start()


    #Render callback
    ac.addRenderCallback(appWindow, onFormRender)

    #force to load the config
    #SolCom.send_command('LoadStandard')

    

def acUpdate(deltaT):

    global appWindow, SolCom, SolUI

    global app, timer1, timer2, timer3, timerButtonWaitTime, uiLockTime
    
    if timer1 < 100: timer1 += deltaT
    if timer2 < 100: timer2 += deltaT
    if timer3 < 100: timer3 += deltaT
    if timerButtonWaitTime < 100: timerButtonWaitTime += deltaT

    if timer1 > 0.25:
        timer1=0.0

        if SolUI.isVisible():
            SolCom.update()


        # check if something is in the temporary config memory
        # It means, some UI Element was used and changed
        # This is needed to get the dirty flag after a wfx reset
        # But we are not doing this every cycle - thats not needed
        if not SolUI.getDirty():
            msg = ac.ext_loadLua("sol.TempCustomWeather__")
            if msg and len(msg)>0:
                SolUI.setDirty(True)

        
    if timer2 > 1:
        timer2=0.0
 
        # constantly check if UI, to react on app page cycles (visibility changes)
        SolUI.update()


    if timer3 > 10 and timer3 < 12:
        n = SolUI.getElement('system', 'console')
        for tmp in n:
            x = (timer3-10)*0.5
            tmp.setColor(0.2+0.2*x,0.9-0.5*x,0.4, 1-0.5*x)


    if timerButtonWaitTime > uiLockTime:
        if SolUI.isLocked():
            SolUI.unlock()


    ac.setBackgroundOpacity(appWindow, 0.85)


#GL Drawing		
def onFormRender(deltaT):
    SolUI.render(deltaT)

#Do on AC shutdown
def acShutdown():
    SolUI.close()
    with open(settings_ini_path, 'w') as ini:
        settings_ini.write(ini)
