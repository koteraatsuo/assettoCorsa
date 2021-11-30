##############################################################
# Kunos Simulazioni
# AC Python tutorial 03 : Add a control to the App
#
# To activate create a folder with the same name as this file
# in apps/python. Ex apps/python/tutorial01
# Then copy this file inside it and launch AC
#############################################################



import ac

# This function gets called by AC when the Plugin is initialised
# The function has to return a string with the plugin name
def acMain(ac_version):

    appWindow=ac.newApp("A Guide",1)
    ac.setSize(appWindow,320,400)

    # Now we create a simple label control, we need to pass in
    # the appWindow we want to add the label to and the label text
	# ac.addLabel(<CONTROL_IDENTIFIER>,<VALUE>)
	# <VALUE> must be a string
    label1=ac.addLabel(appWindow,"HELP :")
    label2=ac.addLabel(appWindow,"Welcome to the setup section of AC!")
    label3=ac.addLabel(appWindow,"In this page you can select the")
    label4=ac.addLabel(appWindow,"subsection using the top Tab Bar.")
    label5=ac.addLabel(appWindow,"Your setup will be helped by")
    label6=ac.addLabel(appWindow,"the Setup Apps that you can find")
    label7=ac.addLabel(appWindow,"in this page, on the right.")
    label8=ac.addLabel(appWindow,"You can navigate the available apps")
    label9=ac.addLabel(appWindow,"using the arrows or you can add")
    label10=ac.addLabel(appWindow,"a new one in the ")
    label11=ac.addLabel(appWindow,"apps\python\setup folder.")
    label12=ac.addLabel(appWindow,"If you don't find the perfect app")
    label13=ac.addLabel(appWindow,"you can create it by yourself!")
	
    # Use ac.setPosition to set the control's position in the app
	# ac.setPosition(<CONTROLO_IDENTIFIER>,<X>,<Y>)
	# <X>,<Y> must be a floating point numbers
	
    ac.setPosition(label1,10,10)
    ac.setPosition(label2,10,50)
    ac.setPosition(label3,10,70)
    ac.setPosition(label4,10,90)
    ac.setPosition(label5,10,120)
    ac.setPosition(label6,10,140)
    ac.setPosition(label7,10,160)
    ac.setPosition(label8,10,190)
    ac.setPosition(label9,10,210)
    ac.setPosition(label10,10,230)
    ac.setPosition(label11,10,250)
    ac.setPosition(label12,10,270)
    ac.setPosition(label13,10,290)
    return "AC Python Tutorial 03"
