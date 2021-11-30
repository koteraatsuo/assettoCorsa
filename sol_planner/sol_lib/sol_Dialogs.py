import ac, acsys
import platform, os, sys, math, time
import codecs, json
import configparser

from .sol_UI import *

gSelfPath = os.path.dirname(os.path.realpath(__file__))
os.environ['PATH'] = os.environ['PATH'] + ";."



def Sol_Dialogs_setMasterUI(ui):
    global gSol_Dialogs_MasterUI
    gSol_Dialogs_MasterUI = ui









gConfirm_SendValue = 0

def Sol_Confirm_close(retValue):
    global gSol_Dialogs_MasterUI, Sol_Confirm_UI, gCallback

    Sol_Confirm_UI.forceVisible(0)
    Sol_Confirm_UI.stop()

    gSol_Dialogs_MasterUI.start()

    if gCallback:
        gCallback(value=retValue)


def Sol_Confirm_buttonHandle(button):
    global gConfirm_SendValue

    if 'Cancel' in button.section:
        Sol_Confirm_close('')
        
    elif 'OK' in button.section:
        Sol_Confirm_close(gConfirm_SendValue)



Sol_Confirm_UI = UI("Confirm", useMultiplePages=False, usePageButtons=False)
def Sol_Confirm_Dialog_INIT(settings_ini):

    global Sol_Confirm_appWindow, gCallback

    Sol_Confirm_appWindow = ac.newApp("Sol Confirm Dialog")
    ac.setTitle(Sol_Confirm_appWindow, "")
    ac.drawBorder(Sol_Confirm_appWindow, 1)
    ac.setIconPosition(Sol_Confirm_appWindow, 0, -10000)
    Sol_Confirm_UI.setWindow(Sol_Confirm_appWindow, 400, 150, settings_ini)
    Sol_Confirm_UI.forceVisible(False)

    x=50; y=100
    Sol_Confirm_UI.newButton('OK', Page=1, Text='OK', FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[80,24]) 
    x=270
    Sol_Confirm_UI.newButton('Cancel', Page=1, Text='Cancel', FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[80,24]) 

    x=200; y=30
    Sol_Confirm_UI.newTitel( Section='Titel',  Page=1, Text='Message', FontSize=16, customPos=[x,y], customDim=[400,50])
    

    Sol_Confirm_UI.setButtonsCallback(Sol_Confirm_buttonHandle)


def Sol_Confirm_Dialog_Open(message, retValue, **args):
    global Sol_Confirm_UI, gConfirm_SendValue, Sol_LoadSave_UI, Sol_Dialog_X, Sol_Dialog_Y

    if gSol_Dialogs_MasterUI:
        Sol_Confirm_UI.setScale(gSol_Dialogs_MasterUI.getScale())
        gSol_Dialogs_MasterUI.stop()

    singleMessage = False
    if 'SingleMessage' in args and args['SingleMessage']:
        singleMessage = True

    
    n = Sol_Confirm_UI.getElement('OK', '')
    if len(n)>0:
        if 'OKButtonText' in args:
            n[0].setText(args['OKButtonText'])
        else:
            n[0].setText('OK')
        if singleMessage:
            n[0].setPosition(160,100)
        else:
            n[0].setPosition(50,100)

    n = Sol_Confirm_UI.getElement('Cancel', '')
    if len(n)>0:
        if singleMessage:
            n[0].hide()
        else:
            n[0].show()

    if 'Callback' in args:
        gCallback = args['Callback']

    if 'Position' in args:
        Sol_Dialog_X = args['Position'][0]
        Sol_Dialog_Y = args['Position'][1]

    gConfirm_SendValue = retValue

    n = Sol_Confirm_UI.getElement('Titel', '')
    if len(n)>0:
        n[0].setText(message)

    Sol_Confirm_UI.start()
    Sol_Confirm_UI.forceVisible(True)




def Sol_Confirm_Dialog_UPDATE(deltaT):
    global Sol_Confirm_UI, Sol_Dialog_X, Sol_Dialog_Y

    if Sol_Confirm_UI.isVisible():
        ac.setPosition(Sol_Confirm_UI.window, Sol_Dialog_X, Sol_Dialog_Y)
        ac.setBackgroundColor(Sol_Confirm_UI.window, 0.0,0.0,0.0)
        ac.setBackgroundOpacity(Sol_Confirm_UI.window, 0.95)

















gCallback = 0
gN_LoadSave_List = 12
gCurrent_LoadSave_List = 0
gLoadSave_FileList = []
gLoadSavePath = ''
gSavingForcedSuffix =''
gLoadSave_FolderPrefix = ''
gCheckOverwrite = False
gChangeRoot = False
gScroll_Size = 1
gScroll_Pos = 0

gDoubleClickTime = 0.5
gLastClickTime = 100
gLastClickedElement = 0



def Sol_LoadSave_close(retValue):
    global gLoadSavePath, gSavingForcedSuffix, gCheckOverwrite, gSol_Dialogs_MasterUI, Sol_LoadSave_UI, gCallback

    ac.setTitle(Sol_LoadSave_appWindow, "")
    Sol_LoadSave_UI.forceVisible(0)
    Sol_LoadSave_UI.stop()

    gSol_Dialogs_MasterUI.start()

    if len(retValue)>0:
        if len(gSavingForcedSuffix)>0:
            if not retValue.endswith(gSavingForcedSuffix):
                retValue += gSavingForcedSuffix
        
        fullPath = os.path.join(gLoadSavePath, retValue)

        if gCheckOverwrite and os.path.isfile(fullPath):
            Sol_Confirm_Dialog_Open('Overwrite\n%s'%retValue, fullPath, OKButtonText='Yes')
        else:
            if gCallback:
                gCallback(value=fullPath)
    else:
        if gCallback:
            gCallback(value='')


def Sol_LoadSave_updateList():
    global gCurrent_LoadSave_List, gN_LoadSave_List, gLoadSave_FileList, gScroll_Size, gScroll_Pos
    global gLoadSavePath, gChangeRoot

    gLoadSave_FileList = []

    for filename in os.listdir(gLoadSavePath):
        #for filename in files:
        if os.path.isdir(os.path.join(gLoadSavePath, filename)):
            if gChangeRoot:
                gLoadSave_FileList.append('%s%s'%(gLoadSave_FolderPrefix,filename))
        else:
            gLoadSave_FileList.append(filename)


    if len(gLoadSave_FileList)>0:
        for i in range(0, gN_LoadSave_List):
            n = Sol_LoadSave_UI.getElement('ListLabel', '%i'%i)
            if len(n)>0:
                if i+gCurrent_LoadSave_List < len(gLoadSave_FileList):
                    tmp = gLoadSave_FileList[i+gCurrent_LoadSave_List]
                    tmp = (tmp[:50] + '..') if len(tmp) > 50 else tmp
                    n[0].setText(tmp)
                else:
                    n[0].setText('')

        gScroll_Size = 230 * min(1, gN_LoadSave_List/len(gLoadSave_FileList)) - 4
        gScroll_Pos = gCurrent_LoadSave_List * 230/len(gLoadSave_FileList) + 2


def Sol_LoadSave_buttonHandle(button):
    global gN_LoadSave_List, gLoadSave_FileList, gCurrent_LoadSave_List, Sol_LoadSave_UI
    global gDoubleClickTime, gLastClickTime, gLastClickedElement

    if gLastClickTime>gDoubleClickTime:
        gLastClickedElement = 0

    if 'Cancel' in button.section:
        Sol_LoadSave_close('')

    elif 'ListButton' in button.section:
        pos = int(button.key)
        if pos+gCurrent_LoadSave_List < len(gLoadSave_FileList):
            tmp = gLoadSave_FileList[pos+gCurrent_LoadSave_List]
            n = Sol_LoadSave_UI.getElement('filename', '')
            if len(n)>0:
                n[0].setValue(tmp)
                if gLastClickedElement==button: #Doubleclick
                    Sol_LoadSave_close(n[0].getValue())
        
    elif 'DO' in button.section:
        n = Sol_LoadSave_UI.getElement('filename', '')
        if len(n)>0:
            Sol_LoadSave_close(n[0].getValue())

    elif 'Scroll' in button.section:
        if 'Up' in button.key:
            if gCurrent_LoadSave_List>0:
                gCurrent_LoadSave_List-=1
        elif 'Down' in button.key:
            if gCurrent_LoadSave_List+gN_LoadSave_List < len(gLoadSave_FileList):
                gCurrent_LoadSave_List+=1
        Sol_LoadSave_updateList()


    gLastClickedElement = button
    gLastClickTime = 0

def Sol_LoadSave_textfieldHandle(listbox):
    if 'filename' in listbox.section:
        if len(listbox.getValue())>0:
            Sol_LoadSave_close(listbox.getValue())


Sol_LoadSave_UI = UI("LoadSave", useMultiplePages=False, usePageButtons=False)
def Sol_LoadSave_Dialog_INIT(settings_ini):

    global Sol_LoadSave_appWindow, Sol_LoadSave_List, gN_LoadSave_List, gSelfPath

    Sol_LoadSave_appWindow = ac.newApp("Sol Load Save Dialog")
    ac.setTitle(Sol_LoadSave_appWindow, "")
    ac.drawBorder(Sol_LoadSave_appWindow, 1)
    ac.setIconPosition(Sol_LoadSave_appWindow, 0, -10000)
    Sol_LoadSave_UI.setWindow(Sol_LoadSave_appWindow, 400, 400, settings_ini)
    Sol_LoadSave_UI.forceVisible(False)

    x=5; y=30
    for i in range(0, gN_LoadSave_List):
        Sol_LoadSave_UI.newButton('ListButton', Key='%i'%i,  Page=1, Text='', FontSize=14, noBorder=True, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y+i*26], customDim=[355,22]) 
        Sol_LoadSave_UI.newLabel( Section='ListLabel',  Key='%i'%i,  Page=1, Text='', FontSize=14, customPos=[x,y+i*26], customDim=[355,22])
    
    x=5; y=340
    Sol_LoadSave_UI.newTextField('filename', '', Text='export.ini', Page=1, FontSize=14, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[355,20])


    x=5; y=370
    button = Sol_LoadSave_UI.newButton('DO', Page=1, Text='', FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[150,24]) 
    #button.set
    x=245
    Sol_LoadSave_UI.newButton('Cancel', Page=1, Text='Cancel', FontSize=16, Color=[0.8,0.8,0.8,1], BackgroundColor=[0.10,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[150,24]) 


    x=375; y=30
    Sol_LoadSave_UI.newTexButton('Scroll', Key='Up',  OffTex='%s/up.png'%gSelfPath,  Page=1, BackgroundColor=[0.75,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[20,40])
    y=300
    Sol_LoadSave_UI.newTexButton('Scroll', Key='Down',  OffTex='%s/down.png'%gSelfPath,  Page=1, BackgroundColor=[0.75,0.10,0.10], BackgroundOpacity=0.5, customPos=[x,y], customDim=[20,40])
   
    Sol_LoadSave_UI.setButtonsCallback(Sol_LoadSave_buttonHandle)
    Sol_LoadSave_UI.setTextFieldsCallback(Sol_LoadSave_textfieldHandle)

    ac.addRenderCallback(Sol_LoadSave_appWindow, Sol_LoadSave_Dialog_FormRender)


    Sol_Confirm_Dialog_INIT(settings_ini)


def Sol_LoadSave_Dialog_Open(path, func, pos, callback, **args):

    global gCallback, gLoadSave_FileList, gCurrent_LoadSave_List, gSelfPath, gSol_Dialogs_MasterUI
    global Sol_Dialog_X, Sol_Dialog_Y
    global gLoadSavePath, gSavingForcedSuffix, gCheckOverwrite

    if len(path)>0:

        gLoadSavePath = path

        if gSol_Dialogs_MasterUI:
            Sol_LoadSave_UI.setScale(gSol_Dialogs_MasterUI.getScale())
            gSol_Dialogs_MasterUI.stop()

        gCallback = callback

        Sol_LoadSave_UI.start()
        Sol_LoadSave_UI.forceVisible(True)
        
        Sol_Dialog_X = pos[0]; Sol_Dialog_Y = pos[1]

        if 'Titel' in args:
            ac.setTitle(Sol_LoadSave_UI.window, args['Titel'])
        if 'DoButtonText' in args:
            n = Sol_LoadSave_UI.getElement('DO', '')
            if len(n)>0:
                n[0].setText(args['DoButtonText'])

        gSavingForcedSuffix = ''
        if 'Suffix' in args:
            gSavingForcedSuffix = '.%s'%args['Suffix']

        gCheckOverwrite = False
        if 'CheckOverwrite' in args:
            gCheckOverwrite = (args['CheckOverwrite'])

        gChangeRoot = False
        if 'ChangeRoot' in args:
            gChangeRoot = args['ChangeRoot']


        if 'Load' in func:
            pass

        elif 'Save' in func:
            pass

        
        gCurrent_LoadSave_List = 0
        Sol_LoadSave_updateList()


def Sol_LoadSave_Dialog_UPDATE(deltaT):
    global Sol_LoadSave_UI, Sol_Dialog_X, Sol_Dialog_Y, gLastClickTime

    if gLastClickTime < 100:
        gLastClickTime += deltaT

    if Sol_LoadSave_UI.isVisible():
        ac.setPosition(Sol_LoadSave_UI.window, Sol_Dialog_X, Sol_Dialog_Y)
        ac.setBackgroundColor(Sol_LoadSave_UI.window, 0.0,0.0,0.0)
        ac.setBackgroundOpacity(Sol_LoadSave_UI.window, 0.95)

    Sol_Confirm_Dialog_UPDATE(deltaT)


def Sol_LoadSave_Dialog_FormRender(deltaT):
    global Sol_LoadSave_UI, gScroll_Size, gScroll_Pos
    if Sol_LoadSave_UI.isVisible():
        Sol_LoadSave_UI.render(deltaT)
        ac.glColor4f(0.4,0.4,0.4,0.5)
        ac.glQuad(
        375 * Sol_LoadSave_UI.scale, 
        (70 + gScroll_Pos) * Sol_LoadSave_UI.scale,
        20 * Sol_LoadSave_UI.scale,
        gScroll_Size * Sol_LoadSave_UI.scale)
        