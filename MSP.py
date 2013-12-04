import serial,time
from bitstring import BitArray

ser=serial.Serial()
ser.port="/dev/ttyACM0"
ser.baudrate=115200
ser.bytesize=serial.EIGHTBITS
ser.parity=serial.PARITY_NONE
serial.stopbits=serial.STOPBITS_ONE
ser.timeout=0
ser.xonxoff=False
ser.rtscts=False
ser.dsrdtr=False
ser.writeTimeout=2

BASIC="\x24\x4d\x3c\x00" 	
MSP_IDT=BASIC+"\x64\x64" 	
MSP_STATUS=BASIC+"\x65\x65"		
MSP_RAW_IMU=BASIC+"\x66\x66"	
MSP_SERVO=BASIC+"\x67\x67"		
MSP_MOTOR=BASIC+"\x68\x68"		
MSP_RC=BASIC+"\x69\x69"			
MSP_RAW_GPS=BASIC+"\x70\x70"
MSP_COMP_GPS=BASIC+"\x71\x71"
MSP_ATTITUDE=BASIC+"\x72\x72"
MSP_ALTITUDE=BASIC+"\x73\x73"
MSP_BAT=BASIC+"\x74\x74"
MSP_RC_TUNING=BASIC+"\x75\x75"
MSP_PID=BASIC+"\x76\x76"
MSP_BOX=BASIC+"\x77\x77"
MSP_MISC=BASIC+"\x78\x78"
MSP_MOTOR_PINS=BASIC+"\x79\x79"
MSP_BOX_NAMES=BASIC+"\x80\x80"
MSP_PID_NAMES=BASIC+"\x81\x81"
MSP_WP=BASIC+"\x82\x82"
MSP_BOX_IDS=BASIC+"\x83\x83"


def connect():
	"""Call this function to connect to the MultiWii controller"""
	try:
		ser.open()
	except Exception,e:
		print("Error opening serial port: "+str(e))
		exit()	
	if ser.isOpen():
		time.sleep(2.5)
		print("Serial port is open at"+ser.portstr)		
	else:
	    print("Cannot open serial port")

def disconnect():
	"""Call this function to disconect from the MultiWii controller"""
	try:
		ser.close()
	except Exception,e:
		print("Eror closing serial port: "+str(e))
		exit()

def reqIdent():
	"""Polls the MultiWii controller and returns the identity values"""
	CURRENT=MSP_IDT
	try:
		ser.flushInput()
		ser.flushOutput()
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		version=ser.read()
		multitype=ser.read()
		mspversion=ser.read()
		capability1=ser.read()
		capability2=ser.read()
		capability3=ser.read()
		capability4=ser.read()
		checksum=ser.read()

		calcchecksum=(ord(length)^ord(code)^ord(version)^ord(multitype)^ord(mspversion)^ord(capability1)^ord(capability2)^ord(capability3)^ord(capability4))

		if (ord(checksum) == calcchecksum):

			ident={"MultiwiiVersion":ord(version),"MultiwiiType":ord(multitype),"Capability1":bin(ord(capability1)),"Capability2":bin(ord(capability2)),"Capability3":bin(ord(capability3)),"Capability4":bin(ord(capability4))}
			return ident
		else:
			print("Bad Checksum")

	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqStatus():
	"""Polls the MultiWii controller and reutrns the status values"""
	CURRENT=MSP_STATUS
	try:
		ser.flushInput()
		ser.flushOutput()
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		cycletime1=ser.read()
		cycletime2=ser.read()
		i2cerror1=ser.read()
		i2cerror2=ser.read()
		sensors1=ser.read()
		sensors2=ser.read()
		activebox1=ser.read()
		activebox2=ser.read()
		activebox3=ser.read()
		activebox4=ser.read()
		currentset=ser.read()
		checksum=ser.read()

		calcchecksum=(ord(length)^ord(code)^ord(cycletime1)^ord(cycletime2)^ord(i2cerror1)^ord(i2cerror2)^ord(sensors1)^ord(sensors2)^ord(activebox1)^ord(activebox2)^ord(activebox3)^ord(activebox4)^ord(currentset))

		if (ord(checksum) == calcchecksum):

			cycletimetemp=ord(cycletime1)
			cycletimetemp2=cycletimetemp<<8
			cycletimetemp3=ord(cycletime2)
			cycletimetemp4=cycletimetemp2|cycletimetemp3
			cycletime=int(cycletimetemp4)

			i2cerrortemp=ord(i2cerror1)
			i2cerrortemp2=i2cerrortemp<<8
			i2cerrortemp3=ord(i2cerror2)
			i2cerrortemp4=i2cerrortemp2|i2cerrortemp3
			i2cerror=int(i2cerrortemp4)

			sensorstemp=ord(sensors1)
			sensorstemp2=sensorstemp<<8 
			sensorstemp3=ord(sensors2)
			sensorstemp4=sensorstemp2|sensorstemp3
			sensors=bin(sensorstemp4)

			activeboxtemp=ord(activebox1)
			activeboxtemp2=activeboxtemp<<24
			activeboxtemp3=ord(activebox2)
			activeboxtemp4=activeboxtemp3<<16
			activeboxtemp5=ord(activebox3)
			activeboxtemp6=activeboxtemp5<<8
			activeboxtemp7=ord(activebox4)
			activeboxtemp8=activeboxtemp2|activeboxtemp4|activeboxtemp6|activeboxtemp7
			activebox=bin(activeboxtemp8)

			currentsettemp=ord(currentset)
			profilenumber=int(currentsettemp)

			status={"cycletime":cycletime,"i2cerror":i2cerror,"sensors":sensors,"activebox":activebox,"profile":profilenumber}
			return status
		else:
			print("Bad Checksum")
				
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqRawImu():
	"""Polls the MultiWii controller and returns raw IMU values"""
	CURRENT=MSP_RAW_IMU
	try:
		ser.flushInput()
		ser.flushOutput()
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		accx1=ser.read()
		accx2=ser.read()
		accy1=ser.read()
		accy2=ser.read()
		accz1=ser.read()
		accz2=ser.read()
		gyrox1=ser.read()
		gyrox2=ser.read()
		gyroy1=ser.read()
		gyroy2=ser.read()
		gyroz1=ser.read()
		gyroz2=ser.read()
		magx1=ser.read()
		magx2=ser.read()
		magy1=ser.read()
		magy2=ser.read()
		magz1=ser.read()
		magz2=ser.read()
		checksum=ser.read()
		calcchecksum=ord(length)^ord(code)^ord(accx1)^ord(accx2)^ord(accy1)^ord(accy2)^ord(accz1)^ord(accz2)^ord(gyrox1)^ord(gyrox2)^ord(gyroy1)^ord(gyroy2)^ord(gyroz1)^ord(gyroz2)^ord(magx1)^ord(magx2)^ord(magy1)^ord(magy2)^ord(magz1)^ord(magz2)

		if (ord(checksum) == calcchecksum):
		
			accxtemp = BitArray(bytes=(accx2),length=8)+BitArray(bytes=(accx1),length=8)
			accx = (accxtemp.int)

			accytemp = BitArray(bytes=(accy2),length=8)+BitArray(bytes=(accy1),length=8)
			accy = (accytemp.int)

			accztemp = BitArray(bytes=(accz2),length=8)+BitArray(bytes=(accz1),length=8)
			accz = (accztemp.int)
			
			gyroxtemp = BitArray(bytes=(gyrox2),length=8)+BitArray(bytes=(gyrox1),length=8)
			gyrox = (gyroxtemp.int)

			gyroytemp = BitArray(bytes=(gyroy2),length=8)+BitArray(bytes=(gyroy1),length=8)
			gyroy = (gyroytemp.int)

			gyroztemp = BitArray(bytes=(gyroz2),length=8)+BitArray(bytes=(gyroz1),length=8)
			gyroz = (gyroztemp.int)

			magxtemp = BitArray(bytes=(magx2),length=8)+BitArray(bytes=(magx1),length=8)
			magx = (magxtemp.int)

			magytemp = BitArray(bytes=(magy2),length=8)+BitArray(bytes=(magy1),length=8)
			magy = (magytemp.int)

			magztemp = BitArray(bytes=(magz2),length=8)+BitArray(bytes=(magz1),length=8)
			magz = (magztemp.int)

			imu={"accx":accx,"accy":accy,"accz":accz,"gyrox":gyrox,"gyroy":gyroy,"gyroz":gyroz,"magx":magx,"magy":magy,"magz":magz}
			return imu

		else:
			print("Bad Checksum")

	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqAttitude():
	"""Polls the MultiWii controller and returns the vehicle attitude"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_ATTITUDE
		ser.write(CURRENT)
		time.sleep(0.1)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		angx1=ser.read()
		angy1=ser.read()
		head1=ser.read()
		checksum=ser.read()
		calcchecksum=ord(length)^ord(code)^ord(angx1)^ord(angy1)
		if (ord(checksum) == calcchecksum):

			angxtemp=BitArray(bytes=(angx1),length=8)
			angx= (angxtemp.int)
			angytemp=BitArray(bytes=(angy1),length=8)
			angy= (angytemp.int)
			print header,
			print ord(length),
			print (code.encode("hex"))
			print ord(angx1)
			print ord(angy1)
			print ord(checksum)
			print calcchecksum

			headtemp=BitArray(bytes=(head1),length=8)
			head= (headtemp.int)
	
			attitude={"anglex":angx,"angley":angy,}
			return attitude

	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqRC():
	"""Polls the MultiWii controller and returns 8 rc input channel values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_RC
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		chan1a=ser.read()
		chan1b=ser.read()
		chan2a=ser.read()
		chan2b=ser.read()
		chan3a=ser.read()
		chan3b=ser.read()
		chan4a=ser.read()
		chan4b=ser.read()
		chan5a=ser.read()
		chan5b=ser.read()
		chan6a=ser.read()
		chan6b=ser.read()
		chan7a=ser.read()
		chan7b=ser.read()
		chan8a=ser.read()
		chan8b=ser.read()
		checksum=ser.read()

		calcchecksum=ord(length)^ord(code)^ord(chan1a)^ord(chan1b)^ord(chan2a)^ord(chan2b)^ord(chan3a)^ord(chan3b)^ord(chan4a)^ord(chan4b)^ord(chan5a)^ord(chan5b)^ord(chan6a)^ord(chan6b)^ord(chan7a)^ord(chan7b)^ord(chan8a)^ord(chan8b)

		if (ord(checksum) == calcchecksum):
			chan1temp=BitArray(bytes=(chan1b),length=8)+BitArray(bytes=(chan1a),length=8)
			chan2temp=BitArray(bytes=(chan2b),length=8)+BitArray(bytes=(chan2a),length=8)
			chan3temp=BitArray(bytes=(chan3b),length=8)+BitArray(bytes=(chan3a),length=8)
			chan4temp=BitArray(bytes=(chan4b),length=8)+BitArray(bytes=(chan4a),length=8)			
			chan5temp=BitArray(bytes=(chan5b),length=8)+BitArray(bytes=(chan5a),length=8)
			chan6temp=BitArray(bytes=(chan6b),length=8)+BitArray(bytes=(chan6a),length=8)
			chan7temp=BitArray(bytes=(chan7b),length=8)+BitArray(bytes=(chan7a),length=8)
			chan8temp=BitArray(bytes=(chan8b),length=8)+BitArray(bytes=(chan8a),length=8)

			chan1=chan1temp.int
			chan2=chan2temp.int
			chan3=chan3temp.int
			chan4=chan4temp.int
			chan5=chan5temp.int
			chan6=chan6temp.int
			chan7=chan7temp.int
			chan8=chan8temp.int

			rc={"channel1":chan1,"channel2":chan2,"channel3":chan3,"channel4":chan4,"channel5":chan5,"channel6":chan6,"channel7":chan7,"channel8":chan8}

			return rc
			
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqMotor():
	"""Polls the MultiWii controller and returns 8 motor output values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_MOTOR
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		mtr1a=ser.read()
		mtr1b=ser.read()
		mtr2a=ser.read()
		mtr2b=ser.read()
		mtr3a=ser.read()
		mtr3b=ser.read()
		mtr4a=ser.read()
		mtr4b=ser.read()
		mtr5a=ser.read()
		mtr5b=ser.read()
		mtr6a=ser.read()
		mtr6b=ser.read()
		mtr7a=ser.read()
		mtr7b=ser.read()
		mtr8a=ser.read()
		mtr8b=ser.read()
		checksum=ser.read()

		calcchecksum=ord(length)^ord(code)^ord(mtr1a)^ord(mtr1b)^ord(mtr2a)^ord(mtr2b)^ord(mtr3a)^ord(mtr3b)^ord(mtr4a)^ord(mtr4b)^ord(mtr5a)^ord(mtr5b)^ord(mtr6a)^ord(mtr6b)^ord(mtr7a)^ord(mtr7b)^ord(mtr8a)^ord(mtr8b)

		if (ord(checksum) == calcchecksum):
			mtr1temp=BitArray(bytes=(mtr1b),length=8)+BitArray(bytes=(mtr1a),length=8)
			mtr2temp=BitArray(bytes=(mtr2b),length=8)+BitArray(bytes=(mtr2a),length=8)
			mtr3temp=BitArray(bytes=(mtr3b),length=8)+BitArray(bytes=(mtr3a),length=8)
			mtr4temp=BitArray(bytes=(mtr4b),length=8)+BitArray(bytes=(mtr4a),length=8)			
			mtr5temp=BitArray(bytes=(mtr5b),length=8)+BitArray(bytes=(mtr5a),length=8)
			mtr6temp=BitArray(bytes=(mtr6b),length=8)+BitArray(bytes=(mtr6a),length=8)
			mtr7temp=BitArray(bytes=(mtr7b),length=8)+BitArray(bytes=(mtr7a),length=8)
			mtr8temp=BitArray(bytes=(mtr8b),length=8)+BitArray(bytes=(mtr8a),length=8)

			mtr1=mtr1temp.int
			mtr2=mtr2temp.int
			mtr3=mtr3temp.int
			mtr4=mtr4temp.int
			mtr5=mtr5temp.int
			mtr6=mtr6temp.int
			mtr7=mtr7temp.int
			mtr8=mtr8temp.int

			motor={"motor1":mtr1,"motor2":mtr2,"motor3":mtr3,"motor4":mtr4,"motor5":mtr5,"motor6":mtr6,"motor7":mtr7,"motor8":mtr8}

			return motor
			
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqServo():
	"""Polls the MultiWii controller and returns 8 servo output values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_SERVO
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		srv1a=ser.read()
		srv1b=ser.read()
		srv2a=ser.read()
		srv2b=ser.read()
		srv3a=ser.read()
		srv3b=ser.read()
		srv4a=ser.read()
		srv4b=ser.read()
		srv5a=ser.read()
		srv5b=ser.read()
		srv6a=ser.read()
		srv6b=ser.read()
		srv7a=ser.read()
		srv7b=ser.read()
		srv8a=ser.read()
		srv8b=ser.read()
		checksum=ser.read()

		calcchecksum=ord(length)^ord(code)^ord(srv1a)^ord(srv1b)^ord(srv2a)^ord(srv2b)^ord(srv3a)^ord(srv3b)^ord(srv4a)^ord(srv4b)^ord(srv5a)^ord(srv5b)^ord(srv6a)^ord(srv6b)^ord(srv7a)^ord(srv7b)^ord(srv8a)^ord(srv8b)

		if (ord(checksum) == calcchecksum):
			srv1temp=BitArray(bytes=(srv1b),length=8)+BitArray(bytes=(srv1a),length=8)
			srv2temp=BitArray(bytes=(srv2b),length=8)+BitArray(bytes=(srv2a),length=8)
			srv3temp=BitArray(bytes=(srv3b),length=8)+BitArray(bytes=(srv3a),length=8)
			srv4temp=BitArray(bytes=(srv4b),length=8)+BitArray(bytes=(srv4a),length=8)			
			srv5temp=BitArray(bytes=(srv5b),length=8)+BitArray(bytes=(srv5a),length=8)
			srv6temp=BitArray(bytes=(srv6b),length=8)+BitArray(bytes=(srv6a),length=8)
			srv7temp=BitArray(bytes=(srv7b),length=8)+BitArray(bytes=(srv7a),length=8)
			srv8temp=BitArray(bytes=(srv8b),length=8)+BitArray(bytes=(srv8a),length=8)

			srv1=srv1temp.int
			srv2=srv2temp.int
			srv3=srv3temp.int
			srv4=srv4temp.int
			srv5=srv5temp.int
			srv6=srv6temp.int
			srv7=srv7temp.int
			srv8=srv8temp.int

			servo={"servo1":srv1,"servo2":srv2,"servo3":srv3,"servo4":srv4,"servo5":srv5,"servo6":srv6,"servo7":srv7,"servo8":srv8}

			return servo
			
	except Exception,e1:
		print("Error communicating..."+str(e1))




def reqAltitude():
	"""Just a test function to verify connection and communication"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_ALTITUDE
		ser.write(CURRENT)
		print("Writing to "+ser.portstr+" "+CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		alt1=ser.read()
		alt2=ser.read()
		alt3=ser.read()
		alt4=ser.read()
		alt5=ser.read()
		alt6=ser.read()
		alt7=ser.read()
		alt8=ser.read()
		checksum=ser.read()
		calcchecksum=ord(length)^ord(code)^ord(alt1)^ord(alt2)^ord(alt3)^ord(alt4)^ord(alt5)^ord(alt6)^ord(alt7)^ord(alt8)
		
		if ord(checksum)==calcchecksum:

			alttemp=BitArray(bytes=(alt4),length=8)+BitArray(bytes=(alt3),length=8)+BitArray(bytes=(alt2),length=8)+BitArray(bytes=(alt1),length=8)
			vartemp=BitArray(bytes=(alt8),length=8)+BitArray(bytes=(alt7),length=8)+BitArray(bytes=(alt6),length=8)+BitArray(bytes=(alt5),length=8)

			alt=(alttemp.float)
			var=(vartemp.float)
			
			altitude={"altitude":alt,"variometer":var}
			return altitude

	except Exception,e1:
		print("Error communicating..."+str(e1))

def req():
	"""Just a test function to verify connection and communication"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_
		ser.write(CURRENT)
		print("Writing to "+ser.portstr+" "+CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		response=ser.readline()

		print header,
		print (length.encode("hex")),
		print (code.encode("hex"))
		print(response.encode("hex"))	
		
	except Exception,e1:
		print("Error communicating..."+str(e1))

def talk():
	"""Just a test function to verify connection and communication"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_ALTITUDE
		ser.write(CURRENT)
		print("Writing to "+ser.portstr+" "),
		print (CURRENT.encode("hex"))
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		response=ser.readline()

		print header,
		print (length.encode("hex")),
		print (code.encode("hex"))
		print(response.encode("hex"))	
		
	except Exception,e1:
		print("Error communicating..."+str(e1))

connect()






