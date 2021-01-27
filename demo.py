from numpy import sin, pi, arange
from appJar import gui 
import random
from mpl_toolkits.mplot3d import Axes3D

count = 0

def press(btn):
	global count
	count+=1
	app.setLabel("title", btn)
	app.addMessage("new"+str(count), "Count = "+str(count))
	if btn=="PRESS and check":
		app.setFg("white")

	if btn=="PRESS now what":
		app.setBg("orange")

	if count>=3:
		app.infoBox("Now message", "Counter more than 3")

	if btn=="Cancel":
		app.stop()

def getPress(btn):
	if btn=="Cancel":
		app.stop()
	else:
		usr = app.getEntry("Username:")
		pwd = app.getEntry("Password:")
		print("user:", usr, "pass:", pwd)

def updateMeter():
	global percentComplete
	app.setMeter("progress", percentComplete)

def getXY():
    x = arange(0.0, 3.0, 0.01)
    y = sin(random.randint(1,10) * pi * x)
    return x,y 

def gen(btn):
    # *getXY() will unpack the two return values
    # and pass them as separate parameters
    app.updatePlot("p1", *getXY())
    showLabels()

def showLabels():
    axes.legend(['The curve'])
    axes.set_xlabel("X Axes")
    axes.set_ylabel("Y Axes")
    app.refreshPlot("p1")

def registerValues(btn):
	if btn == "OptionBoxValue":
		print(app.getOptionBox("Options"))

	if btn == "RadioValue":
		print(app.getRadioButton("song"))

app = gui("Demo GUI", "800x900")
app.setBg("green")
app.setFg("red")
app.setFont(20)

app.addLabel("title", "Helloworld")
app.addMessage("info", "This is a demo of appjar")

app.addButton("PRESS", press)
app.addButton("PRESS and check", press)
app.addButton("PRESS now what", press)

# Input = app.entry("Input")
# Subs = app.addEntry("Subs")
# print("Input : ", Input, "subs : ", Subs)

app.addLabelEntry("Username:")
app.addLabelSecretEntry("Password:")
app.addButtons(["Submit", "Cancel"], getPress)
app.setFocus("Username:")

app.addFlashLabel("f1", "This is flashing")

# schedule function to be called regularly
# This should then be monitoring/updating a global variable: 
percentComplete = 20
app.addMeter("progress")
app.setMeterFill("progress", "blue")
app.registerEvent(updateMeter)

# app.addHorizontalSeparator(5,0,10, colour="Blue")
# app.addVerticalSeparator(5,0, colour="red")
app.addGrip(10,0)

app.addOptionBox("Options", ["- Fruits -", "Apple", "Orange",
                        "Pear", "kiwi", "- Pets -", "Dogs", "Cats",
                        "Fish", "Hamsters"])

app.addCheckBox("Apples")
app.addCheckBox("Pears")
app.addCheckBox("Oranges")
app.addCheckBox("Kiwis")

app.addRadioButton("song", "Killer Queen")
app.addRadioButton("song", "Paradise City")
app.addRadioButton("song", "Parklife")

app.addButtons(["OptionBoxValue", "RadioValue"], registerValues)

app.setCheckBox("Oranges")

canvas = app.addCanvas("c1")
canvas.create_oval(10, 10, 100, 100, fill="red", outline="blue", width=3)
canvas.create_line(0, 0, 155, 155, width=5)
#canvas.create_line(0, 255, 255, 0, dash=123)

# app.addPlotFig("p1")
# axes = app.addPlot("p1", *getXY())
# showLabels()
# app.addButton("Generate", gen)


app.go()