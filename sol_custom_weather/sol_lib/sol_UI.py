# Version 29-08-2021
import ac, functools, configparser


'''
   Label = 1
   Spinner = 2
   Checkbox = 3
   Separator = 4
   Button = 5
   TexButton = 6
   TextField = 7
   Picture = 8
'''


class UI_Element:

    def __init__(self, parent, ui_type, ui_id, section, key, **args):
        
        self.page = 0
        self.pageless = False
        self.customPage = False
        self.parent = parent
        self.window = parent.window
        self.type = ui_type
        self.id = ui_id
        self.section = section
        self.key = key
        self.pythonUIID = 0
        self.CallbackFunc = 0
        self.saveable = False
        self.visible = True
        self.hidden = False

        self.x = 0
        self.y = 0
        self.useCustomPos = False
        if 'customPos' in args:
            self.x = args['customPos'][0]
            self.y = args['customPos'][1]
            self.useCustomPos = True
        
        self.margin = [0,0,0,0] 
        self.width = 0
        self.height = 0
        self.useCustomDim = False
        if 'customDim' in args:
            self.width  = args['customDim'][0]
            self.height = args['customDim'][1]
            self.useCustomDim = True

        if 'Page' in args:
            if args['Page'] <= len(parent.pages) and args['Page']>0:
                self.customPage = True
                self.page = self.parent.pages[int(args['Page']-1)]
                if not self.useCustomPos:
                    self.y = self.page.insert_position
                    page.expand(self.getHeight())
            elif args['Page'] < 0:
                self.pageless = True

        if 'Save' in args:
            self.saveable = args['Save']

        if 'Hide' in args:
            self.hidden = args['Hide']

    def update(self):
        pass

    def updatePosition(self):
        if self.pythonUIID:
            ac.setPosition(self.pythonUIID,
            (self.x + self.margin[0]) * self.parent.scale,
            (self.y+self.margin[1]) * self.parent.scale
            )

    def setPosition(self, x, y):
        self.x = x
        self.y = y
        self.updatePosition()

    def getHeight(self):
        return self.height + self.margin[1] + self.margin[3]

    def setPage(self, page):
        self.page = page
        if not self.useCustomPos:
            self.y = self.page.insert_position
            page.expand(self.getHeight())
        self.updatePosition()
        

    def setVisible(self, v):
        if self.pythonUIID:
            self.visible = v
            if v and not self.hidden:
                ac.setVisible(self.pythonUIID, True)
            else:
                ac.setVisible(self.pythonUIID, False)

    def setCallback(self, func):
        self.CallbackFunc = func

    def show(self):
        self.hidden = False
        if self.visible:
            if self.pythonUIID:
                ac.setVisible(self.pythonUIID, True)
                self.update()
        
    def hide(self):
        self.hidden = True
        if self.pythonUIID:
            ac.setVisible(self.pythonUIID, False)

    def render(self):
        pass

    def close(self):
        return True

    def __del__(self):
        self.close()




class UI_Separator(UI_Element):
    def __init__(self, parent, ui_id, **args):
        section = ''
        key = ''
        if 'Section' in args: section = args['Section']
        if 'Key' in args: key = args['Key']
        super().__init__(parent, 4, ui_id, section, key, **args)

        if not self.useCustomDim:
            self.width = parent.width
            self.margin = [0, 2, 0, 2]
            self.height = 2

    def render(self):
        if not self.hidden:
            ac.glColor4f(0.5, 0.5, 0.5, 0.5)
            ac.glQuad(
            (self.x+self.margin[0]) * self.parent.scale,
            (self.y+self.margin[1]) * self.parent.scale,
            self.width * self.parent.scale,
            self.height * self.parent.scale
            )


class UI_Picture(UI_Element):
    def __init__(self, parent, ui_id, tex_list, **args):
        section = ''
        key = ''
        if 'Section' in args: section = args['Section']
        if 'Key' in args: key = args['Key']
        super().__init__(parent, 8, ui_id, section, key, **args)
    
        self.opacity = 1
        if 'Opacity' in args:
            self.opacity = float(args['Opacity'])

        self.TexPos = 0
        self.TexList = []
        for tmp in tex_list:
            self.TexList.append(ac.newTexture(tmp))

    def setOpacity(self, o):
        self.opacity = o

    def setTex(self, n):
        if n < len(self.TexList):
            self.TexPos = n
        
    def setTexRelative(self, v):
        v = max(0, min(1, v))
        self.TexPos = round(v * (len(self.TexList)-1))

    def render(self):
        if len(self.TexList)>0 and self.TexPos>=0 and self.visible and not self.hidden:
            ac.glColor4f(1,1,1,self.opacity)
            ac.glQuadTextured(
            self.x * self.parent.scale,
            self.y * self.parent.scale,
            self.width * self.parent.scale,
            self.height * self.parent.scale,
            self.TexList[self.TexPos])



class UI_ElementWithText(UI_Element):
    def __init__(self, parent, ui_type, ui_id, section, key, **args):
        super().__init__(parent, ui_type, ui_id, section, key, **args)

        self.text = ' '
        self.textLimit = -1
        if 'Text' in args: self.text = args['Text']
        if 'MaxShownCharacters' in args: self.textLimit = args['MaxShownCharacters']
        self.fontSize = 14
        if 'FontSize' in args: self.fontSize = args['FontSize']
        self.color = [0.75, 0.75, 0.75, 1]
        if 'Color' in args: self.color = args['Color']

        self.noBorder = False
        if 'noBorder' in args:
            self.noBorder = args['noBorder']

        self.uiTitel = 0

    def setText(self, text):
        self.text = text
        if not self.hidden:
            self.update()
    def getText(self):
        return self.text

    def setFontSize(self, *args):
        if args:
            self.fontSize = args
        ui = self.uiTitel or self.pythonUIID
        if ui:
            ac.setFontSize(ui, self.fontSize * self.parent.scale)

    def setColor(self, *args):
        if args:
            self.color = args
        ui = self.uiTitel or self.pythonUIID
        if ui:
            ac.setFontColor(ui, self.color[0], self.color[1], self.color[2], self.color[3])

    def updatePosition(self):
        self.setFontSize()
        super().updatePosition()

    def update(self):
        ui = self.uiTitel or self.pythonUIID
        if ui:
            s = self.text
            if self.textLimit >= 0 and self.textLimit <= len(s):
                s = (s[:self.textLimit] + '..') if len(s) > self.textLimit else s
            ac.setText(ui, s)
            #ac.log('sec:%s, key:%s, ui:%i, text:%s'%(self.section,self.key,ui,s))
            self.setColor()
            self.updatePosition()
            if self.noBorder:
                ac.drawBorder(ui, 0)
        


class UI_InteractionElement(UI_ElementWithText):

    def __init__(self, parent, ui_type, ui_id, section, key, **args):
        super().__init__(parent, ui_type, ui_id, section, key, **args)

        self.bUse = True

        self.lockUIafterUsage = False
        if 'LockUIafterUsage' in args: self.lockUIafterUsage = args['LockUIafterUsage']

        self.showValueComparisonSystem = True
        if 'showValueComparisonSystem' in args: self.showValueComparisonSystem = args['showValueComparisonSystem']

    def handle(self, *args):
        #check for clones and set their values
        v = self.getValue()
        n = self.parent.getElement(self.section, self.key)
        for tmp in n:
            if tmp != self:
                tmp.initValue(v)

        if self.lockUIafterUsage:
            self.parent.lock()

    def use(self):
        self.bUse = True

    def unuse(self):
        self.bUse = False

class UI_Label(UI_ElementWithText):
    
    def __init__(self, parent, ui_id, **args):
        section = ''
        key = ''
        if 'Section' in args: section = args['Section']
        if 'Key' in args: key = args['Key']
        super().__init__(parent, 1, ui_id, section, key, **args)
        
        self.pythonUIID = ac.addLabel(self.window, self.text)

        if 'FontAlignment' in args:
            ac.setFontAlignment(self.pythonUIID, args['FontAlignment'])
        else:
            ac.setFontAlignment(self.pythonUIID, 'left')
        
        if 'customDim' not in args:
            self.width = parent.width * 0.96
            self.margin = [parent.width*0.02, 0, 0, 0]
        self.height = self.fontSize

        if 'Margin' in args:
            self.margin = args['Margin']

        self.update()

    def update(self):
        super().update()


class UI_Titel(UI_Label):
    def __init__(self, parent, ui_id, **args):
        super().__init__(parent, ui_id, **args)

        if 'Color' not in args: self.setColor(0.9, 0.9, 0.9, 1)

        ac.setFontAlignment(self.pythonUIID, 'center')
        
        self.margin = [0, 0, 0, 0]
        self.x = parent.width*0.5


class UI_Spinner(UI_InteractionElement):
    
    def __init__(self, parent, ui_id, section, key, text, span, bFloat, value, **args):
        self.saveable = True
        if 'FontSize' not in args:
            args['FontSize'] = 14
        if 'Color' not in args:
            args['Color'] = [1,1,1,1]
        super().__init__(parent, 2, ui_id, section, key, Text=text, **args)
        self.span = span
        self.value = value
        self.bFloat = bFloat
        self.forcedValue = value
        self.forcedValueRelative = -1
        self.forcedValue__GUI_X = 0
        self.forcedValue__GUI_Y = 0
        self.forcedValue__GUI_V = 0
        self.forcedValue__GUI_Color = [0.5, 0.5, 0.5, 0.75]
        self.stdHandleFunc = 0
        self.setValueHandleBlocker = False

        if self.bFloat:
            self.span[0] *= 100
            self.span[1] *= 100
            self.value *= 100
            self.text = self.text + ', %'

        self.pythonUIID = ac.addSpinner(self.window, ' ')
        self.uiTitel = ac.addLabel(self.window, self.text)
        ac.setFontAlignment(self.uiTitel, 'center')

        self.forcedValueRightLabel = ac.addLabel(self.window, '')
        
        ac.setFontAlignment(self.forcedValueRightLabel, 'right')
        self.forcedValueLeftLabel = ac.addLabel(self.window, '')
        ac.setFontAlignment(self.forcedValueLeftLabel, 'left')

        if not self.useCustomDim:
            self.width = parent.width
        self.height = self.fontSize * 1.5
        self.margin = [self.width*0.02, self.fontSize+1, self.width*0.02, self.fontSize*0.4]

        self.update()

        self.stdHandleFunc = functools.partial(self.handle)
        ac.addOnValueChangeListener(self.pythonUIID, self.stdHandleFunc)


    def getValue(self):

        if self.bFloat:
            return float(self.value*0.01)
        else:
            return self.value

    def getSpan(self):
        if self.bFloat:
            return [float(self.span[0]*0.01), float(self.span[1]*0.01)]
        else:
            return self.span

    def getText(self):
        if self.bFloat:
            return self.text.rstrip(', %')
        else:
            return self.text

    def handle(self, *args):
        if self.parent.isRunning:
            #ac.log("handle")
            if not self.forcedValueRelative:
                if ac.getValue(self.pythonUIID) != self.forcedValue:
                    ac.setValue(self.pythonUIID, self.forcedValue)
            else:
                if self.parent.isLocked():
                    if not self.setValueHandleBlocker:
                        self.setValueHandleBlocker = True
                        ac.setValue(self.pythonUIID, self.value)
                    else:
                        self.setValueHandleBlocker = False
                else:
                    if ac.getValue(self.pythonUIID) != self.value and self.saveable and not self.setValueHandleBlocker: self.parent.setDirty(True)
                    self.value = ac.getValue(self.pythonUIID)
                    if not self.setValueHandleBlocker:
                        if self.CallbackFunc:
                            self.CallbackFunc(self)
                            super().handle(*args)
                    else:
                        #ac.log("handle blocked")
                        self.setValueHandleBlocker = False

    def initValue(self, value, **args):
        if self.bFloat:
            value = int(value*100)
        
        self.forcedValue = value
        self.forcedValueRelative = -1
        self.forcedValue__GUI_V = 0
        self.forcedValue__GUI_Color = [0.5, 0.5, 0.5, 0.75]
        ac.setText(self.forcedValueLeftLabel, ' ')
        ac.setText(self.forcedValueRightLabel, ' ')

        # use the Spinner's handle function to set self.value
        # we have to set the dirty flag also!
        self.setValueHandleBlocker = True
        if "CallHandle" in args and args["CallHandle"]:
            self.setValueHandleBlocker = False
        ac.setValue(self.pythonUIID, value)
        self.setValueHandleBlocker = False

    def setForcedValue(self, value, bRelative):
        if self.bFloat:
            value *= 100

        self.forcedValue = min(self.span[1], max(self.span[0], int(value)))
        
        span = (self.span[1] - self.span[0])
        w = (self.width - (self.margin[0] + self.margin[2]) - (self.height*2+4))
        steps = w
        if span > 0:
            steps = w / span
        self.forcedValue__GUI_V = max(0, self.forcedValue - self.span[0]) * steps
        self.forcedValue__GUI_X = self.x + self.margin[0] + self.height + 1
        self.forcedValue__GUI_Y = self.y + self.height + self.margin[1] - 6

        if self.bFloat:
            ac.setText(self.forcedValueRightLabel, '%.2f'%(self.forcedValue*0.01))
        else:
            ac.setText(self.forcedValueRightLabel, '%i'%value)

        self.setForcedValueType(bRelative)
        if not self.forcedValueRelative:
            ac.setValue(self.pythonUIID, self.forcedValue)

        

    def setForcedValueType(self, bRelative):
        if self.forcedValueRelative != bRelative:
            self.forcedValueRelative = bRelative
            if self.forcedValueRelative:
                self.setForcedValueColor(0.25, 0.75, 1.0, 1)
                ac.setText(self.forcedValueLeftLabel, 'modulated')
            else:
                self.setForcedValueColor(1.0, 0.5, 0, 1)
                ac.setText(self.forcedValueLeftLabel, 'set')


    def setForcedValueColor(self, r,g,b,a):
        self.forcedValue__GUI_Color = [r,g,b,a]
        ac.setFontColor(self.forcedValueRightLabel, self.forcedValue__GUI_Color[0],
        self.forcedValue__GUI_Color[1],
        self.forcedValue__GUI_Color[2],
        self.forcedValue__GUI_Color[3])
        ac.setFontColor(self.forcedValueLeftLabel, self.forcedValue__GUI_Color[0],
        self.forcedValue__GUI_Color[1],
        self.forcedValue__GUI_Color[2],
        self.forcedValue__GUI_Color[3])

    def update(self):
        super().update()
        ac.setRange(self.pythonUIID, self.span[0], self.span[1])
        ac.setStep(self.pythonUIID, 1)
        self.setValueHandleBlocker = True
        ac.setValue(self.pythonUIID, self.value)
        self.setValueHandleBlocker = False
        
    def updatePosition(self):
        ac.setPosition(self.uiTitel,
        (self.x + self.width*0.5) * self.parent.scale,
        (self.y - self.fontSize - 6 + self.margin[1]) * self.parent.scale
        )
        ac.setPosition(self.forcedValueRightLabel,
        (self.x + self.margin[0] + (self.width - (self.height + 5 + self.width*0.02) - self.margin[2])) * self.parent.scale,
        (self.y + self.margin[1]) * self.parent.scale
        )
        ac.setPosition(self.forcedValueLeftLabel,
        (self.x + self.margin[0] + self.height + 5) * self.parent.scale,
        (self.y + self.margin[1]) * self.parent.scale
        )
        ac.setSize(self.pythonUIID,
        (self.width - (self.margin[0] + self.margin[2])) * self.parent.scale,
        (self.height) * self.parent.scale
        )
        super().updatePosition() #call the parent also

    def setFontSize(self, *args):
        if args:
            self.fontSize = args
        ac.setFontSize(self.pythonUIID, self.fontSize * self.parent.scale) #trigger spinner inside text size
        ac.setFontSize(self.forcedValueRightLabel, 10 * self.parent.scale)
        ac.setFontSize(self.forcedValueLeftLabel, 10 * self.parent.scale)
        super().setFontSize()

    def setVisible(self, v):
        ac.setVisible(self.uiTitel, v)
        ac.setVisible(self.forcedValueRightLabel, v)
        ac.setVisible(self.forcedValueLeftLabel, v)
        super().setVisible(v)

    def render(self):
        
        if self.showValueComparisonSystem:
            ac.glColor4f(self.forcedValue__GUI_Color[0],
            self.forcedValue__GUI_Color[1],
            self.forcedValue__GUI_Color[2],
            self.forcedValue__GUI_Color[3]*0.67)
            ac.glQuad(
            self.forcedValue__GUI_X * self.parent.scale, 
            self.forcedValue__GUI_Y * self.parent.scale,
            self.forcedValue__GUI_V * self.parent.scale,
            5 * self.parent.scale)






class UI_Checkbox(UI_InteractionElement):
    
    def __init__(self, parent, ui_id, section, key, text, value, **args):
        self.saveable = True
        if 'FontSize' not in args:
            args['FontSize'] = 14
        if 'Color' not in args:
            args['Color'] = [1,1,1,1]
        super().__init__(parent, 3, ui_id, section, key, Text=text, **args)
        self.value = value
        self.forcedValue = value
        self.forcedValue__GUI_Color = [0.5, 0.5, 0.5, 0.75]
        self.forcedValueRelative = -1
        self.stdHandleFunc = 0
        self.setValueHandleBlocker = False

        self.pythonUIID = ac.addCheckBox(self.window, '')

        self.uiTitel = ac.addLabel(self.window, self.text)
        ac.setFontAlignment(self.uiTitel, 'left')

        self.margin = [self.parent.width*0.02, 0, 0, 0]

        self.width = parent.width*0.96
        self.height = self.fontSize

        self.update()

        self.stdHandleFunc = functools.partial(self.handle)
        ac.addOnCheckBoxChanged(self.pythonUIID, self.stdHandleFunc)

    def getValue(self):
        if self.value:
            return True
        else:
            return False

    def update(self):
        super().update()
        self.setValueHandleBlocker = True
        if self.pythonUIID:
            ac.setValue(self.pythonUIID, self.value)
        self.setValueHandleBlocker = False
    
    def updatePosition(self):
        if self.pythonUIID:
            ac.setPosition(self.uiTitel,
            (self.x + self.margin[0] + self.height * 2.5) * self.parent.scale,
            (self.y + self.margin[1] - 1) * self.parent.scale
            )
            ac.setSize(self.pythonUIID, self.width * self.parent.scale, (self.height+1) * 1.25 * self.parent.scale)
            super().updatePosition()

    def setVisible(self, v):
        ac.setVisible(self.uiTitel, v)
        super().setVisible(v)

    def handle(self, *args):
        #ac.log('handle %s, %i'%(self.key,int(args[1])))
        if self.parent.isRunning and self.bUse:
            if not self.forcedValueRelative:
                if args[1] != self.forcedValue:
                    ac.setValue(self.pythonUIID, self.forcedValue)
            else:
                if self.parent.isLocked():
                    if not self.setValueHandleBlocker:
                        self.setValueHandleBlocker = True
                        ac.setValue(self.pythonUIID, self.value)
                    else:
                        self.setValueHandleBlocker = False
                else:
                    if int(args[1]) != self.value and self.saveable and not self.setValueHandleBlocker: self.parent.setDirty(True)
                    self.value = int(args[1])
                    if not self.setValueHandleBlocker:
                        if self.CallbackFunc:
                            self.CallbackFunc(self)
                            super().handle(*args)
                    else:
                        self.setValueHandleBlocker = False
        
    def initValue(self, value):
        value = int(value)
        self.forcedValue = value
        self.forcedValue__GUI_Color = [0.5, 0.5, 0.5, 0.75]
        self.forcedValueRelative = -1
        #ac.log('init %s:%i'%(self.key,value))
        #if self.stdHandleFunc == 0:
        #    ac.log('no handle func set!')
        # use the Checkbox's handle function to set self.value
        # we have to set the dirty flag also!
        self.setValueHandleBlocker = True
        ac.setValue(self.pythonUIID, value)
        self.handle(self.pythonUIID, value)
        self.setValueHandleBlocker = False
        
    def setForcedValue(self, value, bRelative):
        self.forcedValue = int(value)
        
        self.setForcedValueType(bRelative)
        if not self.forcedValueRelative:
            ac.setValue(self.pythonUIID, self.forcedValue)

    def setForcedValueType(self, bRelative):
        if self.forcedValueRelative != bRelative:
            self.forcedValueRelative = bRelative
            if self.forcedValueRelative:
                self.setForcedValueColor(0.25, 0.75, 1.0, 1)
            else:
                self.setForcedValueColor(1.0, 0.5, 0, 1)

    def setForcedValueColor(self, r,g,b,a):
        self.forcedValue__GUI_Color = [r,g,b,a]
    
    def render(self):
        
        if self.showValueComparisonSystem:
            ac.glColor4f(self.forcedValue__GUI_Color[0],
            self.forcedValue__GUI_Color[1],
            self.forcedValue__GUI_Color[2],
            self.forcedValue__GUI_Color[3])
            if self.forcedValue:
                ac.glQuad(
                (self.x + self.margin[0] + 4 + self.height*1.25) * self.parent.scale,
                (self.y + self.margin[1]) * self.parent.scale,
                self.height*0.75 * self.parent.scale,
                (self.height*1.25 + 1) * self.parent.scale
                )
            else:
                ac.glQuad(
                (self.x + self.margin[0] + 4 + self.height*1.25) * self.parent.scale,
                (self.y + self.margin[1]) * self.parent.scale,
                self.height*0.75 * self.parent.scale,
                (self.height*0.75 - 1) * self.parent.scale
                )








class UI_Button(UI_InteractionElement):
    
    def __init__(self, parent, ui_id, section, **args):
        key = ''
        if 'Key' in args: key = args['Key']
        super().__init__(parent, 5, ui_id, section, key, **args)

        self.pythonUIID = ac.addButton(self.window, self.text)

        self.backgroundColor = [0,0,0]
        if 'BackgroundColor' in args: self.backgroundColor = args['BackgroundColor']

        self.backgroundOpacity = 0.25
        if 'BackgroundOpacity' in args: self.backgroundOpacity = args['BackgroundOpacity']
        
        self.flashing = False
        self.flashingTimer = 0

        self.bundledData = []

        if not self.useCustomDim:
            self.margin = [self.parent.width*0.02, 0, 0, 0]
            self.width = parent.width*0.96
            self.height = self.fontSize * 1.5

        self.stdHandleFunc = functools.partial(self.handle)
        ac.addOnClickedListener(self.pythonUIID, self.stdHandleFunc)

        self.update()

    # fake some functions to fit the logic
    def getValue(self):
        return False
    def setValue(self):
        pass

    def setBackgroundColor(self, *args):
        self.backgroundColor = args
        self.update()

    def setBackgroundOpacity(self, opacity):
        self.backgroundOpacity = opacity
        self.update()

    def setFlashing(self, state):
        if state:
            self.flashing = True
            self.flashingTimer = 0
        else:
            self.flashing = False
            self.update()

    def handleFlashing(self, dt):
        if self.flashing:
            self.flashingTimer += dt
            if self.flashingTimer >= 2: self.flashingTimer = 0

            x = self.flashingTimer
            if x > 1: x = (2-x)
            t = (0.75 + 0.75*x)
            r = self.backgroundColor[0] * t
            g = self.backgroundColor[1] * t
            b = self.backgroundColor[2] * t
            ac.setBackgroundColor(self.pythonUIID, r, g, b)
            ac.setBackgroundOpacity(self.pythonUIID, self.backgroundOpacity)

    def handle(self, *args):
        if self.parent.isRunning and not self.parent.isLocked() and self.bUse:
            if self.CallbackFunc:
                self.CallbackFunc(self)
                super().handle(*args)

    def updatePosition(self):
        ac.setSize(self.pythonUIID, self.width * self.parent.scale, self.height * self.parent.scale)
        super().updatePosition()

    def update(self):
        super().update()
        ac.setBackgroundColor(self.pythonUIID, self.backgroundColor[0], self.backgroundColor[1], self.backgroundColor[2])
        ac.setBackgroundOpacity(self.pythonUIID, self.backgroundOpacity)

    def press(self):
        self.handle()

    def setBundledData(self, dataList):
        self.bundledData = []
        for tmp in dataList:
            self.bundledData.append(tmp)

    def getBundledData(self):
        return self.bundledData






class UI_TexButton(UI_Button):
    
    def __init__(self, parent, ui_id, section, **args):
        super().__init__(parent, ui_id, section, **args)

        self.OffTex = 0
        if 'OffTex' in args: self.OffTex = ac.newTexture(args['OffTex'])

        self.OnTex = 0
        if 'OnTex'  in args: self.OnTex  = ac.newTexture(args['OnTex'])

    def render(self):
        if self.OffTex>0 and self.visible and not self.hidden:
            ac.glColor4f(1,1,1,1)
            ac.glQuadTextured(
            self.x * self.parent.scale,
            self.y * self.parent.scale,
            self.width * self.parent.scale,
            self.height * self.parent.scale,
            self.OffTex)





class UI_TextField(UI_InteractionElement):
    def __init__(self, parent, ui_id, section, key, **args):
        super().__init__(parent, 7, ui_id, section, key, **args)

        self.pythonUIID = ac.addTextInput(self.window, "TEXT_INPUT")

        self.backgroundColor = [0,0,0]
        if 'BackgroundColor' in args: self.backgroundColor = args['BackgroundColor']

        self.backgroundOpacity = 0.25
        if 'BackgroundOpacity' in args: self.backgroundOpacity = args['BackgroundOpacity']

        if not self.useCustomDim:
            self.margin = [self.parent.width*0.02, 0, 0, 0]
            self.width = parent.width*0.96
            self.height = self.fontSize * 1.5

        self.stdHandleFunc = functools.partial(self.handle)
        ac.addOnValidateListener(self.pythonUIID, self.stdHandleFunc)

    def updatePosition(self):
        ac.setSize(self.pythonUIID, self.width * self.parent.scale, self.height * self.parent.scale)
        super().updatePosition()

    def update(self):
        super().update()
        ac.setBackgroundColor(self.pythonUIID, self.backgroundColor[0], self.backgroundColor[1], self.backgroundColor[2])
        ac.setBackgroundOpacity(self.pythonUIID, self.backgroundOpacity)

    def handle(self, *args):
        if self.parent.isRunning and not self.parent.isLocked() and self.bUse:
            if self.CallbackFunc:
                self.CallbackFunc(self)

    def setFocus(self, b):
        ac.setFocus(self.pythonUIID, b)

    def getValue(self):
        return ac.getText(self.pythonUIID).lstrip()

    def setValue(self, text):
        ac.setText(self.pythonUIID, text)









class UI_Page:

    def __init__(self, parent, number, **args):
        self.parent = parent
        self.window = parent.window
        self.number = number
        self.insert_position = 65
        self.linespace = 10

        self.section = ''
        if 'Section' in args: self.section = args['Section']
        

    def fits(self, h):
        if self.insert_position+h < self.parent.height:
            return True  
        else:
            return False 

    def expand(self, h):
        self.insert_position += (h + self.linespace)

    def addSpacer(self, **args):
        if 'Space' in args: self.insert_position += args['Space']
        else: self.insert_position += self.linespace

    def close(self):
        return True

    def __del__(self):
        self.close()







class UI:

    def __init__(self, app, **args):
        self.app = app
        self.appSettings = 0
        self.pages = []
        self.currentPage = 1
        self.useMultiplePages = True
        self.usePageButtons = True
        self.elements = []
        self.window = 0
        self.width = 0
        self.height = 0
        self.scale = 1.0
        self.pageLabel = 0
        self.pageSpinner = 0
        self.pageSpinnerFunc = 0
        self.closeButton = 0
        self.closeButtonFunc = 0

        self.appSizeUpButton = 0
        self.appSizeUpButtonFunc = 0
        self.appSizeDownButton = 0
        self.appSizeDownButtonFunc = 0

        self.dirty = False

        self.spinnersCallback = 0
        self.checkboxesCallback = 0
        self.buttonsCallback = 0
        self.textfieldsCallback = 0
        self.pagesCallback = 0
        self.dirtyCallback = 0
        self.lockingCallback = 0
        self.visibleCallback = 0
        self.scaleCallback = 0

        self.openFunc = functools.partial(self.openApp)
        self.closeFunc = functools.partial(self.closeApp)

        self.isRunning = False
        self.locked = False
        self.visible = -1

        if 'useMultiplePages' in args:
            self.useMultiplePages = args['useMultiplePages'] 

        if 'usePageButtons' in args:
            self.usePageButtons = args['usePageButtons']


    def setSpinnersCallback(self, func):
        self.spinnersCallback = func
        for v in self.elements:
            if v.type == 2: #spinners
                v.CallbackFunc = self.spinnersCallback

    def setCheckboxesCallback(self, func):
        self.checkboxesCallback = func
        for v in self.elements:
            if v.type == 3: #checkboxes
                v.CallbackFunc = self.checkboxesCallback

    def setButtonsCallback(self, func):
        self.buttonsCallback = func
        for v in self.elements:
            if v.type == 5 or v.type == 6: #buttons
                v.CallbackFunc = self.buttonsCallback

    def setTextFieldsCallback(self, func):
        self.textfieldsCallback = func
        for v in self.elements:
            if v.type == 7: #textFields
                v.CallbackFunc = self.textfieldsCallback

    def setPagesCallback(self, func):
        self.pagesCallback = func

    def setDirtyCallback(self, func):
        self.dirtyCallback = func

    def setLockingCallback(self, func):
        self.lockingCallback = func

    def setVisibleCallback(self, func):
        self.visibleCallback = func

    def setScaleCallback(self, func):
        self.scaleCallback = func

    def refresh(self):
        for v in self.elements:
            if v.pageless or (v.page and v.page.number == self.currentPage):
                v.setVisible(1)
            else:
                v.setVisible(0)

    def handlePageSpinner(self, *args):
        self.currentPage = ac.getValue(self.pageSpinner)
        if self.isRunning: self.refresh()
        if self.pagesCallback:
            self.pagesCallback(self.pages[int(self.currentPage-1)])

    def handleClose(self, *args):
        ac.setVisible(self.window, False)
        # self.setVisible(False)

    def handleAppSizeUp(self, *args):
        self.setScale(self.scale + 0.05)
            
    def handleAppSizeDown(self, *args):
        self.setScale(self.scale - 0.05)

    def getDirty(self):
        return self.dirty

    def setDirty(self, flag):
        if flag != self.dirty:
            self.dirty = flag
            if self.dirty:
                ac.setFontSize(self.closeButton, 36 * self.scale)
                ac.setText(self.closeButton, "*")
            else:
                ac.setFontSize(self.closeButton, 27 * self.scale)
                ac.setText(self.closeButton, "X")
            if self.dirtyCallback: self.dirtyCallback(self.dirty)

    def setScale(self, v):
        if v >= 0.50 and v <= 2:
            self.scale = v
            self.updateWindow()
            if self.scaleCallback:
                self.scaleCallback(self)
    
    def getScale(self):
        return self.scale

    def updateWindow(self):
        ac.setSize(self.window, self.width*self.scale, self.height*self.scale)

        if self.useMultiplePages:
            ac.setSize(self.pageSpinner,(self.width*0.25) * self.scale, 20 * self.scale)
            ac.setFontSize(self.pageSpinner, 14 * self.scale)
            ac.setPosition(self.pageSpinner, (self.width*0.375) * self.scale, 30 * self.scale)
            ac.setFontSize(self.pageLabel, 14 * self.scale)
            ac.setPosition(self.pageLabel, (self.width*0.25) * self.scale, 30 * self.scale)

        if self.usePageButtons:
            ac.setPosition(self.closeButton, 5 * self.scale, 5 * self.scale)
            ac.setSize(self.closeButton, 40 * self.scale, 40 * self.scale)
            if self.dirty:
                ac.setFontSize(self.closeButton, 36 * self.scale)
            else:
                ac.setFontSize(self.closeButton, 27 * self.scale)
            
            ac.setPosition(self.appSizeUpButton, 50 * self.scale, 5 * self.scale)
            ac.setSize(self.appSizeUpButton, 19 * self.scale, 19 * self.scale)
            ac.setFontSize(self.appSizeUpButton, 12 * self.scale)

            ac.setPosition(self.appSizeDownButton, 50 * self.scale, 26 * self.scale)
            ac.setSize(self.appSizeDownButton, 19 * self.scale, 19 * self.scale)
            ac.setFontSize(self.appSizeDownButton, 12 * self.scale)

        for v in self.elements:
            v.updatePosition()

    def setWindow(self, window, width, height, parser):
        self.window = window
        self.width = width
        self.height = height
        
        self.appSettings = parser
        self.scale = 1.0
        if self.appSettings:
            if self.appSettings.has_section("GENERAL"):
                if self.appSettings.has_option("GENERAL", "appsize"):
                    self.scale = self.appSettings.getfloat("GENERAL","appsize")
            else:
                self.appSettings.add_section("GENERAL")

        if self.useMultiplePages:
            self.pageSpinner = ac.addSpinner(self.window, '')
            ac.setRange(self.pageSpinner, 0, 0)
            ac.setStep(self.pageSpinner, 1)
            self.pageSpinnerFunc = functools.partial(self.handlePageSpinner)
            ac.addOnValueChangeListener(self.pageSpinner, self.pageSpinnerFunc)
            self.pageLabel = ac.addLabel(self.window, 'Page')

        if self.usePageButtons:
            self.closeButton = ac.addButton(self.window, "X")
            ac.setFontColor(self.closeButton, 1, 1, 1, 0.8)
            ac.setBackgroundColor(self.closeButton, 0.80, 0, 0)
            self.closeButtonFunc = functools.partial(self.handleClose)
            ac.addOnClickedListener(self.closeButton, self.closeButtonFunc)

            self.appSizeUpButton = ac.addButton(self.window, "+")
            ac.setFontColor(self.appSizeUpButton, 1, 1, 1, 0.8)
            ac.setBackgroundColor(self.appSizeUpButton, 0.5, 0.8, 0.2)
            self.appSizeUpButtonFunc = functools.partial(self.handleAppSizeUp)
            ac.addOnClickedListener(self.appSizeUpButton, self.appSizeUpButtonFunc)

            self.appSizeDownButton = ac.addButton(self.window, "-")
            ac.setFontColor(self.appSizeDownButton, 1, 1, 1, 0.8)
            ac.setBackgroundColor(self.appSizeDownButton, 0.2, 0.5, 0.8)
            self.appSizeDownButtonFunc = functools.partial(self.handleAppSizeDown)
            ac.addOnClickedListener(self.appSizeDownButton, self.appSizeDownButtonFunc)

        self.updateWindow()

        #self.checkVisible()

        ac.addOnAppDismissedListener(self.window, self.closeFunc)
        ac.addOnAppActivatedListener(self.window, self.openFunc)

    def forceVisible(self, v):
        self.visible = v
        ac.setVisible(self.window, self.visible)
        if self.visibleCallback:
            self.visibleCallback(self)

    def setVisible(self, v):
        self.visible = v
        #ac.setVisible(self.window, self.visible)
        if self.visibleCallback:
            self.visibleCallback(self)

    def isVisible(self):
        return self.visible > 0

    def checkVisible(self):
        v = ac.ext_isAppVisible(self.app)
        if self.visible != v: 
            self.setVisible(v)

    def openApp(self, *args):
        self.setVisible(True)

    def closeApp(self, *args):
        self.setVisible(False)

    def updatePageSpinner(self):
        ac.setRange(self.pageSpinner, 1, len(self.pages))
        ac.setValue(self.pageSpinner, self.currentPage)

    def newPage(self):
        self.pages.append(UI_Page(self, len(self.pages)+1))
        self.updatePageSpinner()
        return self.pages[-1]

    def setValidPage(self, element):
        page = 0
        if len(self.pages)>0:
            page = self.pages[-1]
        else:
            page = self.newPage()

        if self.useMultiplePages:
            if not page.fits(element.getHeight()):
                page = self.newPage()
        element.setPage(page)

    def addElement(self, element):
        self.elements.append(element)
        tmp = self.elements[-1]
        if not tmp.pageless and not tmp.customPage:
            self.setValidPage(tmp) 
        return tmp
        
    def newSeparator(self, **args):
        return self.addElement(UI_Separator(self, len(self.elements)+1, **args))

    def newLabel(self, **args):
        return self.addElement(UI_Label(self, len(self.elements)+1, **args))
    
    def newTitel(self, **args):
        return self.addElement(UI_Titel(self, len(self.elements)+1, **args))
        
    def newSpinner(self, section, key, title, span, bFloat, value, **args):
        return self.addElement(UI_Spinner(self, len(self.elements)+1, section, key, title, span, bFloat, value, **args))
        
    def newCheckbox(self, section, key, title, value, **args):
        return self.addElement(UI_Checkbox(self, len(self.elements)+1, section, key, title, value, **args))

    def newButton(self, section, **args):
        return self.addElement(UI_Button(self, len(self.elements)+1, section, **args))

    def newTexButton(self, section, **args):
        return self.addElement(UI_TexButton(self, len(self.elements)+1, section, **args))

    def newTextField(self, section, key, **args):
        return self.addElement(UI_TextField(self, len(self.elements)+1, section, key, **args))

    def newPictureSeq(self, tex_list, **args):
        return self.addElement(UI_Picture(self, len(self.elements)+1, tex_list, **args))


    def addSpacer(self, **args):
        self.pages[-1].addSpacer(**args)

    def getElement(self, section, key):
        t = []
        for v in self.elements:
            if v.section == section and len(v.section) == len(section) and v.key == key and len(v.key) == len(key):
                t.append(v)
        
        return t

    def getElementsOfSection(self, section):
        t = []
        for v in self.elements:
            if v.section == section and len(v.section) == len(section):
                t.append(v)
        
        return t


    def readLayout(self, file):


        try:
            with open(file, 'r', encoding='utf-8') as file:
                file_plain = file.read()
            filedata = file_plain.splitlines(True)

            for idx, line in enumerate(filedata):
                if line[0] == '-' and line[1] == '-' and line[2] == '<' and line[3] == 'b' and line[4] == 'r' and line[5] == '>':
                    self.newPage()
                elif line[0] == '-' and line[1] == '-' and line[2] == '-' and line[3] == '-':
                    self.newSeparator()
                elif line[0] == '-' and line[1] == '-' and line[2] == ' ':
                    if line[3] == '#':
                        if len(self.pages)>0:
                            self.pages[-1].addSpacer()
                    elif line[3] == '▌':
                        tmp = line.replace('-- ▌', '')
                        tmp = tmp.replace('▐', '')
                        tmp = tmp.lstrip(' ')
                        tmp = tmp.rstrip(' ')
                        self.newTitel(Text=tmp, FontSize=16)
                    else:
                        tmp = line.lstrip('-- ')
                        self.newLabel(Text=tmp, FontSize=12)
                else:
                    section = 0
                    key = 0
                    typeof = 0
                    default = 0.0
                    minof = 0.0
                    maxof = 0.0
                    saveable = True

                    # remove all spaces
                    tmp = line.replace(' ', '')
                    # split into parameter and definition
                    block = tmp.split('--')
                    if len(block)==2:
                        parameter = block[0].split('.')
                        if len(parameter)==2:
                            section = parameter[0]
                            key = parameter[1]

                        tmp = block[1].lower()
                        definition = tmp.split(',')
                        for v in definition:
                            tmp = v.split('=')
                            if len(tmp)==2:
                                if tmp[0].find('type') != -1:
                                    if tmp[1].find('float') != -1:
                                        typeof = 1
                                    elif tmp[1].find('integer') != -1:
                                        typeof = 2
                                    elif tmp[1].find('boolean') != -1:
                                        typeof = 3
                                if tmp[0].find('min') != -1:
                                    minof = float(tmp[1])
                                if tmp[0].find('max') != -1:
                                    maxof = float(tmp[1])
                                if tmp[0].find('default') != -1:
                                    if tmp[1].find('true') != -1:
                                        default = 1
                                    elif tmp[1].find('false') != -1:
                                        default = 0
                                    else: default = float(tmp[1])
                                if tmp[0].find('saveable') != -1:
                                    if tmp[1].find('true') != -1:
                                        saveable = True
                                    elif tmp[1].find('false') != -1:
                                        saveable = False

                    if section and key and typeof:
                        if typeof == 1: #float
                            self.newSpinner(section, key, key, [minof, maxof], True, default, Save=saveable)
                        elif typeof == 2: #integer
                            self.newSpinner(section, key, key, [minof, maxof], False, default, Save=saveable) 
                        elif typeof == 3: #boolean
                            self.newCheckbox(section, key, key, default, Save=saveable)


        except:
            ac.log("UI: Unexpected error:" + traceback.format_exc())






















    def render(self, dt):
        if self.visible: # and self.isRunning:
            for v in self.elements:
                if v.page and v.page.number == self.currentPage:
                    v.render()
                    if v.type == 5: #button
                        if v.flashing: v.handleFlashing(dt)
            if self.isLocked():
                ac.glColor4f(0.1,0.1,0.1,0.5)
                ac.glQuad(0, 0, self.width * self.scale, self.height * self.scale)

    def update(self):
        self.checkVisible()

    def start(self):
        for v in self.elements:
            v.updatePosition()
            v.update()
        self.isRunning = True
        self.refresh()

    def stop(self):
        self.isRunning = False
        
    def Runs(self):
        return self.isRunning

    def lock(self):
        self.locked = True
        if self.lockingCallback:
            self.lockingCallback(self)

    def unlock(self):
        self.locked = False
        if self.lockingCallback:
            self.lockingCallback(self)

    def isLocked(self):
        return self.locked


    def close(self):
        if self.appSettings:
            self.appSettings.set("GENERAL", "appsize", '%.2f'%self.scale) #
        return True

    def __del__(self):
        self.close()