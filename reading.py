## import the necessary libraries
import serial
import matplotlib.pyplot as plt

# Define the number of lines of data to log
characterstoread = 1000

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
# arrs = []
curstring = serin.decode()

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
while len(curstring) < characterstoread:
	if ser.inWaiting():
		x = ser.read()
		x = x.decode()
		# print(x)
		curstring += x

print(curstring)
# Variables for formatting
# title = "python counter,arduino counter,accel X,accel Y,accel Z,linacc X,linacc Y,linacc Z,grav X,grav Y,grav Z,eul X,eul Y,eul Z,gyr X,gyr Y,gyr Z,mag X,mag Y,mag Z,quat W,quat Y,quat X,quat Z\r\n"
# titlearr = [title]
# arrs = titlearr + arrs
# titleindices = title.split(",")

# This section is for writing the data files
incfile = open('filenum.txt', "r")
prevfile = incfile.read(10)
prevfile = int(prevfile)
incfile.close()
filename = 'data{}.csv'.format(prevfile)
file = open(filename, "w", newline='\n')
file.write(curstring)
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