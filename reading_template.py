## import the necessary libraries
import serial
import matplotlib.pyplot as plt

# Define the number of lines of data to log
linestoread = 100

## Code to connect to device
connected = False
locations = ['COM8', 'COM10', 'COM11']

for device in locations:
	try:
		print("Trying...",device)
		ser = serial.Serial(device, 9600)
		break
	except:
		print("Failed to connect on",device)

while not connected:
	serin = ser.read()
	connected = True

# Define variables used later
arrs = []
curstring = ''

# This function checks if the incoming data is corrupt
def checkValid(instring):
	isValid = True

	if not linenum == 1:
		instring = instring.split('\r\n')
		instring = instring[0]
		instring = instring.split(':')
		instring = instring[0]
		instring = instring.split(';')
		if len(instring) < 7:
			isValid = False

		acc = len(instring[0].split(','))
		vel = len(instring[1].split(','))
		grav = len(instring[2].split(','))
		eul = len(instring[3].split(','))
		gyr = len(instring[4].split(','))
		mag = len(instring[5].split(','))
		quat = len(instring[6].split(','))

		if acc != 3 or vel != 3 or grav != 3 or eul != 3 or gyr != 3 or mag != 3 or quat != 4:
			isValid = False

	return isValid

# This block takes the incoming data from the arduino and saves it in the storage array
linenum = 1
while len(arrs) < linestoread:
	if ser.inWaiting():
		x = ser.read()
		x = x.decode()
		curstring = curstring + x
		# text_file.write(x)
		if x=="\n" and len(curstring) > 108 and checkValid(curstring):
			outstr = ''
			string = ''
			outstr = curstring.split("\r\n")
			outstr = outstr[0]
			outstr = outstr.replace(";",",")
			outstr = outstr.split(":")
			outstr = outstr[1] + ',' + outstr[0]
			string = str(linenum) + ',' + outstr + '\n'
			arrs.append(string)
			curstring = ''
			linenum = linenum + 1
			# print(string)
			print(len(arrs))
del arrs[0]

# Variables for formatting
title = "python counter,arduino counter,accel X,accel Y,accel Z,linacc X,linacc Y,linacc Z,grav X,grav Y,grav Z,eul X,eul Y,eul Z,gyr X,gyr Y,gyr Z,mag X,mag Y,mag Z,quat W,quat Y,quat X,quat Z\r\n"
titlearr = [title]
arrs = titlearr + arrs
titleindices = title.split(",")

# This section is for writing the data files
incfile = open('filenum.txt', "r")
prevfile = incfile.read(1)
prevfile = int(prevfile)
incfile.close()
filename = 'data{}.csv'.format(prevfile)
file = open(filename, "w", newline='\n')
file.writelines(arrs)
file.close()
nextfile = prevfile + 1
nextfile = str(nextfile)
incfile = open('filenum.txt', "w")
incfile.write(nextfile)
incfile.close()
ser.close()

# This function returns the numeric value of the requested data
def getval(instr, name):
	accval = instr.split(",")
	indx = titleindices.index(name)
	accval = accval[indx]
	accval = float(accval)
	return accval

# This section is where the processing occurs in order to graph the data
xaxisarr = []
xlinaccarr = []
ylinaccarr = []
zlinaccarr = []
xgravarr = []
ygravarr = []
zgravarr = []
xeularr = []
yeularr = []
zeularr = []
xgyrarr = []
ygyrarr = []
zgyrarr = []

for i in range(1, len(arrs)):
	xaxisarr = xaxisarr + [i];
	stuff = arrs[i]
	xlinaccarr = xlinaccarr + [getval(stuff, 'linacc X')]
	ylinaccarr = ylinaccarr + [getval(stuff, 'linacc Y')]
	zlinaccarr = zlinaccarr + [getval(stuff, 'linacc Z')]
	xgravarr = xgravarr + [getval(stuff, 'grav X')]
	ygravarr = ygravarr + [getval(stuff, 'grav Y')]
	zgravarr = zgravarr + [getval(stuff, 'grav Z')]
	xeularr = xeularr + [getval(stuff, 'eul X')]
	yeularr = yeularr + [getval(stuff, 'eul Y')]
	zeularr = zeularr + [getval(stuff, 'eul Z')]
	xgyrarr = xgyrarr + [getval(stuff, 'gyr X')]
	ygyrarr = ygyrarr + [getval(stuff, 'gyr Y')]
	zgyrarr = zgyrarr + [getval(stuff, 'gyr Z')]

# This section is where everything is graphed
plt.figure(1)
plt.subplot(221)
plt.plot(xaxisarr, xlinaccarr)
plt.ylabel('Linear Accel X')
plt.suptitle("Linear Accelerations", fontsize=16)
plt.subplot(222)
plt.plot(xaxisarr, ylinaccarr)
plt.ylabel('Linear Accel Y')
plt.subplot(223)
plt.plot(xaxisarr, zlinaccarr)
plt.ylabel('Linear Accel Z')

plt.figure(2)
plt.subplot(221)
plt.plot(xaxisarr, xgravarr)
plt.ylabel('Gravity Vector X')
plt.suptitle("Gravity", fontsize=16)
plt.subplot(222)
plt.plot(xaxisarr, ygravarr)
plt.ylabel('Gravity Vector Y')
plt.subplot(223)
plt.plot(xaxisarr, zgravarr)
plt.ylabel('Gravity Vector Z')

plt.figure(3)
plt.subplot(221)
plt.plot(xaxisarr, xeularr)
plt.ylabel('Euler Angle X')
plt.suptitle("Euler Angles", fontsize=16)
plt.subplot(222)
plt.plot(xaxisarr, yeularr)
plt.ylabel('Euler Angle Y')
plt.subplot(223)
plt.plot(xaxisarr, zeularr)
plt.ylabel('Euler Angle Z')


plt.figure(4)
plt.subplot(221)
plt.plot(xaxisarr, xgyrarr)
plt.ylabel('Angular Accel X')
plt.suptitle("Angular Acceleration", fontsize=16)
plt.subplot(222)
plt.plot(xaxisarr, ygyrarr)
plt.ylabel('Angular Accel Y')
plt.subplot(223)
plt.plot(xaxisarr, zgyrarr)
plt.ylabel('Angular Accel Z')

plt.show()