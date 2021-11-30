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
textInput=0
lastMessageLabel=0

# This function gets called by AC when the Plugin is initialised
# The function has to return a string with the plugin name
def acMain(ac_version):
    global textInput, appWindow, lastMessageLabel
    appWindow=ac.newApp("Python Chat")
    ac.setSize(appWindow,400,100)
    ac.drawBorder(appWindow,0)
    ac.setBackgroundOpacity(appWindow,0)
    textInput = ac.addTextInput(appWindow,"TEXT_INPUT")
    ac.setPosition(textInput,15,60)
    ac.setSize(textInput,360,30)
    ac.addOnValidateListener(textInput,onValidateListener)
    ac.addOnChatMessageListener(appWindow,onChatMessage)
    author = "server"
    message = "Welcome to the Python demo chat"
    finalMessage = author + " : " + message
    lastMessageLabel = ac.addLabel(appWindow,finalMessage)
    ac.setPosition(lastMessageLabel,15,30)
    return "G Meter"
	
def onChatMessage(message, author) :
    global lastMessageLabel
    completeMessage = author + " : " + message
    ac.setText(lastMessageLabel,completeMessage)
    #ac.setText(lastMessageLabel, '{}'.format(ac.getCarEngineBrakeCount(0)))

def onValidateListener(string):
    global textInput, lastMessageLabel
    text = ac.getText(textInput)
    ac.setText(textInput,"")
    ac.setFocus(textInput,1)
    ac.sendChatMessage(text)
    #ac.setText(lastMessageLabel, '{}'.format(ac.getCarState(0, acsys.CS.AccG)))
    #ac.setText(lastMessageLabel, '{}'.format(ac.getCarEngineBrakeCount(0)))
    