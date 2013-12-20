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
MSP_RAW_GPS=BASIC+"\x6A\x6A"
MSP_COMP_GPS=BASIC+"\x6B\x6B"
MSP_ATTITUDE=BASIC+"\x6C\x6C"
MSP_ALTITUDE=BASIC+"\x6D\x6D"
MSP_BAT=BASIC+"\x6E\x6E"
MSP_RC_TUNING=BASIC+"\x6F\x6F"
MSP_PID=BASIC+"\x70\x70"
MSP_BOX=BASIC+"\x71\x71"
MSP_MISC=BASIC+"\x72\x72"
MSP_MOTOR_PINS=BASIC+"\x73\x73"
MSP_BOX_NAMES=BASIC+"\x74\x74"
MSP_PID_NAMES=BASIC+"\x75\x75"
MSP_WP=BASIC+"\x76\x76"
MSP_BOX_IDS=BASIC+"\x77\x77"
MSP_SERVO_CONF=BASIC+"\x78\x78"

def connect():
	"""Call this function to connect to the MultiWii controller"""
	try:
		ser.open()
	except Exception,e:
		print("Error opening serial port: "+str(e))
		exit()	
	if ser.isOpen():
		time.sleep(3.5)
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

def reqident():
	"""Polls the MultiWii controller and returns the identity values as a dictionary"""
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
			capabilitytemp=BitArray(bytes=(capability4),length=8)+BitArray(bytes=(capability3),length=8)+BitArray(bytes=(capability2),length=8)+BitArray(bytes=(capability1),length=8)		
			capability=capabilitytemp.bin
			ident={"MultiwiiVersion":ord(version),"MultiwiiType":ord(multitype),"MSP Version":ord(mspversion),"Capability":capability}
			return ident
		else:
			print("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqstatus():
	"""Polls the MultiWii controller and reutrns the status values as a dictionary"""
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
			cycletimetemp=ord(cycletime2)
			cycletimetemp2=cycletimetemp<<8
			cycletimetemp3=ord(cycletime1)
			cycletimetemp4=cycletimetemp2|cycletimetemp3
			cycletime=int(cycletimetemp4)
			i2cerrortemp=ord(i2cerror2)
			i2cerrortemp2=i2cerrortemp<<8
			i2cerrortemp3=ord(i2cerror1)
			i2cerrortemp4=i2cerrortemp2|i2cerrortemp3
			i2cerror=int(i2cerrortemp4)
			sensorstemp=BitArray(bytes=(sensors2),length=8)+BitArray(bytes=(sensors1),length=8)
			sensors=sensorstemp.bin
			activeboxtemp=BitArray(bytes=(activebox4),length=8)+BitArray(bytes=(activebox3),length=8)+BitArray(bytes=(activebox2),length=8)+BitArray(bytes=(activebox1),length=8)
			activebox=activeboxtemp.bin
			currentsettemp=ord(currentset)
			profilenumber=int(currentsettemp)
			status={"cycletime":cycletime,"i2cerror":i2cerror,"sensors":sensors,"activebox":activebox,"profile":profilenumber}
			return status
		else:
			print("Bad Checksum")				
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqrawimu():
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

def reqattitude():
	"""Polls the MultiWii controller and returns the vehicle attitude"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_ATTITUDE
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		angx1=ser.read()
		angx2=ser.read()
		angy1=ser.read()
		angy2=ser.read()
		head1=ser.read()
		head2=ser.read()
		checksum=ser.read()
		calcchecksum=ord(length)^ord(code)^ord(angx1)^ord(angx2)^ord(angy1)^ord(angy2)^ord(head1)^ord(head2)	
		if (ord(checksum) == calcchecksum):
			angxtemp=BitArray(bytes=(angx2),length=8)+BitArray(bytes=(angx1),length=8)
			angx= (angxtemp.int)
			angytemp=BitArray(bytes=(angy2),length=8)+BitArray(bytes=(angy1),length=8)
			angy= (angytemp.int)
			headtemp=BitArray(bytes=(head2),length=8)+BitArray(bytes=(head1),length=8)
			head= (headtemp.int)	
			attitude={"anglex":angx,"angley":angy,"heading":head}
			return attitude
		else:
			print("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqrc():
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
		else:
			print ("Bad checksum")			
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqmotor():
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
		else:
			print ("Bad Checksum")			
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqservo():
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
		else:
			print ("Bad Checksum")			
	except Exception,e1:
		print("Error communicating..."+str(e1))
		
def reqaltitude():
	"""Polls the MultiWii controller and returns the barometer altitude and variometer reading"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_ALTITUDE
		ser.write(CURRENT)
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
		checksum=ser.read()
		calcchecksum=ord(length)^ord(code)^ord(alt1)^ord(alt2)^ord(alt3)^ord(alt4)^ord(alt5)^ord(alt6)		
		if ord(checksum)==calcchecksum:
			alttemp=BitArray(bytes=(alt4),length=8)+BitArray(bytes=(alt3),length=8)+BitArray(bytes=(alt2),length=8)+BitArray(bytes=(alt1),length=8)
			vartemp=BitArray(bytes=(alt6),length=8)+BitArray(bytes=(alt5),length=8)
			alt=(alttemp.int)
			var=(vartemp.int)			
			altitude={"altitude":alt,"variometer":var}
			return altitude
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqboxids():
	"""This function polls the MultiWii controll for the box ids and returns a list of the values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_BOX_IDS
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		calcchecksum=ord(getlength)^ord(code)
		response=[]
		for x in range(length):
			inbyte=ser.read()
			boxidtemp=BitArray(bytes=(inbyte),length=8)
			response.append(boxidtemp.int)
			calcchecksum=calcchecksum^ord(inbyte)
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			return response
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqboxnames():
	"""This function polls the multiwii controller and returns a list of the activation box names"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_BOX_NAMES
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		namestring=""
		calcchecksum=ord(getlength)^ord(code)
		for x in range((length)-1):
			inbyte=ser.read()
			calcchecksum=calcchecksum^ord(inbyte)
			namestring=namestring+inbyte
		checksummakeup=ser.read()
		calcchecksum=calcchecksum^ord(checksummakeup)
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			names=namestring.split(";")
			return names
		else:
			print ("Bad checksum")		
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqactivate():
	"""This function polls the MultiWii controller for the current box activations and return a list or those activations in the same order as the BoxNames function call"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_BOX
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		responsetemp=0
		response=[]
		calcchecksum=ord(getlength)^ord(code)
		for x in range((length)/2):
			inbyte1=ser.read()
			inbyte2=ser.read()
			calcchecksum=calcchecksum^ord(inbyte1)^ord(inbyte2)
			responsetemp=(BitArray(bytes=(inbyte1),length=8)+BitArray(bytes=(inbyte2),length=8))
			response.append(responsetemp.bin)		
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			return response
		else:
			print("Bad Checksum")
		
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqrawgps():
	"""This function polls the MultiWii controller for its GPS information and returns those values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_RAW_GPS
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		fix=ser.read()
		nmst=ser.read()
		lat1=ser.read()
		lat2=ser.read()
		lat3=ser.read()
		lat4=ser.read()
		lon1=ser.read()
		lon2=ser.read()
		lon3=ser.read()
		lon4=ser.read()
		alt1=ser.read()
		alt2=ser.read()
		spd1=ser.read()
		spd2=ser.read()
		crs1=ser.read()
		crs2=ser.read()
		checksum=ser.read()
		fixtemp=BitArray(bytes=(fix),length=8)
		numsattemp=BitArray(bytes=(nmst),length=8)
		lattitudetemp=BitArray(bytes=(lat4),length=8)+BitArray(bytes=(lat3),length=8)+BitArray(bytes=(lat2),length=8)+BitArray(bytes=(lat1),length=8)
		longitudetemp=BitArray(bytes=(lon4),length=8)+BitArray(bytes=(lon3),length=8)+BitArray(bytes=(lon2),length=8)+BitArray(bytes=(lon1),length=8)
		altitudetemp=BitArray(bytes=(alt2),length=8)+BitArray(bytes=(alt1),length=8)
		speedtemp=BitArray(bytes=(spd2),length=8)+BitArray(bytes=(spd1),length=8)
		coursetemp=BitArray(bytes=(crs2),length=8)+BitArray(bytes=(crs1),length=8)
		calcchecksum=ord(length)^ord(code)^ord(fix)^ord(nmst)^ord(lat1)^ord(lat2)^ord(lat3)^ord(lat4)^ord(lon1)^ord(lon2)^ord(lon3)^ord(lon4)^ord(alt1)^ord(alt2)^ord(spd1)^ord(spd2)^ord(crs1)^ord(crs2)
		fix=fixtemp.uint
		numsat=numsattemp.uint
		lattitude=lattitudetemp.uint
		longitude=longitudetemp.uint
		altitude=altitudetemp.uint
		speed=speedtemp.uint
		course=coursetemp.uint
		if ord(checksum)==calcchecksum:
			response={"fix":fix,"Numsat":numsat,"lattitude":lattitude,"longitude":longitude,"altitude":altitude,"speed":speed,"course":course}
			return response
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqcompgps():
	"""This function polls the MultiWii controller for the computed GPS and returns the values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_COMP_GPS
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		length=ser.read()
		code=ser.read()
		dth1=ser.read()
		dth2=ser.read()
		hth1=ser.read()
		hth2=ser.read()
		gps=ser.read()
		checksum=ser.read()
		distancetohometemp=BitArray(bytes=(dth2),length=8)+BitArray(bytes=(dth1),length=8)
		directiontohometemp=BitArray(bytes=(hth2),length=8)+BitArray(bytes=(hth1),length=8)
		gpsupdatetemp=BitArray(bytes=(gps),length=8)
		distancetohome=distancetohometemp.int
		directiontohome=directiontohometemp.int
		gpsupdate=gpsupdatetemp.int
		calcchecksum=ord(length)^ord(code)^ord(dth1)^ord(dth2)^ord(hth1)^ord(hth2)^ord(gps)	
		if ord(checksum)==calcchecksum:
			response={"Distance to home":distancetohome,"Direction to home":directiontohome,"GPS Update":gpsupdate}
			return response
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqanalog():
	"""This function polls the MultiWii controller and returns the analog reading values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_BAT
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		vbat1=ser.read()
		vbattemp=BitArray(bytes=(vbat1),length=8)
		vbat=((vbattemp.int)/10)
		powersum1=ser.read()
		powersum2=ser.read()
		powersumtemp=BitArray(bytes=(powersum2),length=8)+BitArray(bytes=(powersum1),length=8)
		powermetersum=powersumtemp.int
		rssi1=ser.read()
		rssi2=ser.read()
		rssitemp=BitArray(bytes=(rssi2),length=8)+BitArray(bytes=(rssi1),length=8)
		rssi=rssitemp.int
		amps1=ser.read()
		amps2=ser.read()
		ampstemp=BitArray(bytes=(amps2),length=8)+BitArray(bytes=(amps1),length=8)
		amps=ampstemp.int
		checksum=ser.read()
		calcchecksum=ord(getlength)^ord(code)^ord(vbat1)^ord(powersum1)^ord(powersum2)^ord(rssi1)^ord(rssi2)^ord(amps1)^ord(amps2)
		if ord(checksum)==calcchecksum:
			response={"power meter sum":powermetersum,"rssi":rssi,"amps":amps,"battery voltage":vbat}
			return response
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqrctuning():
	"""This function polls the MultiWii controller and returns the RC tuning parameters"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_RC_TUNING
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		getrcrate=ser.read()
		rcratetemp=BitArray(bytes=(getrcrate),length=8)
		rcrate=rcratetemp.int		
		getrcexpo=ser.read()
		rcexpotemp=BitArray(bytes=(getrcexpo),length=8)
		rcexpo=rcexpotemp.int
		getrprate=ser.read()
		rpratetemp=BitArray(bytes=(getrprate),length=8)
		rprate=rpratetemp.int
		getyrate=ser.read()
		yratetemp=BitArray(bytes=(getyrate),length=8)
		yrate=yratetemp.int
		getdtpid=ser.read()
		dtpidtemp=BitArray(bytes=(getdtpid),length=8)
		dtpid=dtpidtemp.int
		gettmid=ser.read()
		tmidtemp=BitArray(bytes=(gettmid),length=8)
		tmid=tmidtemp.int
		gettexpo=ser.read()
		texpotemp=BitArray(bytes=(gettexpo),length=8)
		texpo=texpotemp.int
		checksum=ser.read()
		calcchecksum=ord(getlength)^ord(code)^ord(getrcrate)^ord(getrcexpo)^ord(getrprate)^ord(getyrate)^ord(getdtpid)^ord(gettmid)^ord(gettexpo)
		if ord(checksum)==calcchecksum:
			response={"RC Rate":rcrate,"RC Expo":rcexpo,"Roll Pitch Rate":rprate,"Yaw Rate":yrate,"Dynamic Throttle PID":dtpid,"Throttle Mid":tmid,"Throttle EXPO":texpo}
			return response
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqpidnames():
	"""This function polls the multiwii controller and returns a list of the PID names"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_PID_NAMES
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		namestring=""
		calcchecksum=ord(getlength)^ord(code)
		for x in range((length)-1):
			inbyte=ser.read()
			calcchecksum=calcchecksum^ord(inbyte)
			namestring=namestring+inbyte
		checksummakeup=ser.read()
		calcchecksum=calcchecksum^ord(checksummakeup)
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			names=namestring.split(";")
			return names
		else:
			print ("Bad checksum")		
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqpid():
	"""This function polls the Muliwii controller and return the PID values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_PID
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		piditem={}
		pidlist=[]
		calcchecksum=ord(getlength)^ord(code)
		for x in range((length)/3):
			getp=ser.read()
			ptemp=BitArray(bytes=(getp),length=8)
			p=ptemp.int
			geti=ser.read()
			itemp=BitArray(bytes=(geti),length=8)
			i=itemp.int
			getd=ser.read()
			dtemp=BitArray(bytes=(getd),length=8)
			d=dtemp.int
			piditem[x]={"P":p,"I":i,"D":d}
			calcchecksum=calcchecksum^ord(getp)^ord(geti)^ord(getd)
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			for b in range((length)/3):
				pidlist.append(piditem[b])
			return pidlist
		else:
			print("Bad Checksum")		
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqmisc():
	"""This function polls the MultiWii controller and returns the misc values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_MISC
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		ptr1=ser.read()
		ptr2=ser.read()
		powertrigtemp=BitArray(bytes=(ptr2),length=8)+BitArray(bytes=(ptr1),length=8)
		powertrig=powertrigtemp.int
		mth1=ser.read()
		mth2=ser.read()
		minthrottletemp=BitArray(bytes=(mth2),length=8)+BitArray(bytes=(mth1),length=8)
		minthrottle=minthrottletemp.int
		mxth1=ser.read()
		mxth2=ser.read()
		maxthrottletemp=BitArray(bytes=(mxth2),length=8)+BitArray(bytes=(mth1),length=8)
		maxthrottle=maxthrottletemp.int
		mncm1=ser.read()
		mncm2=ser.read()
		mincommandtemp=BitArray(bytes=(mncm2),length=8)+BitArray(bytes=(mncm1),length=8)
		mincommand=mincommandtemp.int
		fsth1=ser.read()
		fsth2=ser.read()
		failsafethrottletemp=BitArray(bytes=(fsth2),length=8)+BitArray(bytes=(fsth1),length=8)
		failsafethrottle=failsafethrottletemp.int
		plgal1=ser.read()
		plgal2=ser.read()
		plogalarmtemp=BitArray(bytes=(plgal2),length=8)+BitArray(bytes=(plgal1),length=8)
		plogalarm=plogalarmtemp.int
		plglt1=ser.read()
		plglt2=ser.read()
		plglt3=ser.read()
		plglt4=ser.read()
		ploglifetimetemp=BitArray(bytes=(plglt4),length=8)+BitArray(bytes=(plglt3),length=8)+BitArray(bytes=(plglt2),length=8)+BitArray(bytes=(plglt1),length=8)
		ploglifetime=ploglifetimetemp.int
		mdec1=ser.read()
		mdec2=ser.read()
		magdectemp=BitArray(bytes=(mdec2),length=8)+BitArray(bytes=(mdec1),length=8)
		magdec=magdectemp.int
		vbsc=ser.read()
		vbatscaletemp=BitArray(bytes=(vbsc),length=8)
		vbatscale=vbatscaletemp.int
		vbw1=ser.read()
		vbatwarningtemp1=BitArray(bytes=(vbw1),length=8)
		vbatwarning1=vbatwarningtemp1.int
		vbw2=ser.read()
		vbatwarningtemp2=BitArray(bytes=(vbw2),length=8)
		vbatwarning2=vbatwarningtemp2.int
		vbcr=ser.read()
		vbatcriticaltemp=BitArray(bytes=(vbcr),length=8)
		vbatcritical=vbatcriticaltemp.int
		checksum=ser.read()
		calcchecksum=ord(getlength)^ord(code)^ord(ptr1)^ord(ptr2)^ord(mth1)^ord(mth2)^ord(mxth1)^ord(mxth2)^ord(mncm1)^ord(mncm2)^ord(fsth1)^ord(fsth2)^ord(plgal1)^ord(plgal2)^ord(plglt1)^ord(plglt2)^ord(plglt3)^ord(plglt4)^ord(mdec1)^ord(mdec2)^ord(vbsc)^ord(vbw1)^ord(vbw2)^ord(vbcr)	
		if ord(checksum)==calcchecksum:
			response={"Power Trigger":powertrig,"Min Throttle":minthrottle,"Max Throttle":maxthrottle,"Min Command":mincommand,"FailSafe Throttle":failsafethrottle,"PLog Alarm":plogalarm,"PLog Lifetime":ploglifetime,"Magnetic Declination":magdec,"VBat Scale":vbatscale,"VBat Warning 1":vbatwarning1,"VBat Warning 2":vbatwarning2,"VBat Critical":vbatcritical}
			return response
		else:
			print ("Bad Checksum")				
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqmotorpins():
	"""Polls the MultiWii controller and returns the motor pin numbers"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_MOTOR_PINS
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		motorpinslist=[]
		calcchecksum=ord(getlength)^ord(code)
		for i in range(length):
			getpin=ser.read()
			pintemp=BitArray(bytes=(getpin),length=8)
			motorpin=pintemp.int
			motorpinslist.append(motorpin)
			calcchecksum=calcchecksum^ord(getpin)
		checksum=ser.read()
		pinnames={"Motor 1","Motor 2","Motor 3","Motor 4","Motor 5","Motor 6","Motor 7","Motor 8"}
		if ord(checksum)==calcchecksum:
			motorpins=dict(zip(pinnames,motorpinslist))
			return motorpins
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))

def reqservoconf():
	"""Polls the MultiWii controller and returns the servo cofiguration values"""
	try:
		ser.flushInput()
		ser.flushOutput()
		CURRENT=MSP_SERVO_CONF
		ser.write(CURRENT)
		time.sleep(0.01)
		header=ser.read(3)
		getlength=ser.read()
		lengthtemp=BitArray(bytes=(getlength),length=8)
		length=lengthtemp.int
		code=ser.read()
		calcchecksum=ord(getlength)^ord(code)
		servo={}
		for i in range((length)/7):
			min1=ser.read()
			min2=ser.read()
			max1=ser.read()
			max2=ser.read()
			mid1=ser.read()
			mid2=ser.read()
			rate=ser.read()
			mintemp=BitArray(bytes=(min2),length=8)+BitArray(bytes=(min1),length=8)
			maxtemp=BitArray(bytes=(max2),length=8)+BitArray(bytes=(max1),length=8)
			midtemp=BitArray(bytes=(mid2),length=8)+BitArray(bytes=(mid1),length=8)
			ratetemp=BitArray(bytes=(rate),length=8)
			servomin=mintemp.int
			servomax=maxtemp.int
			servomid=midtemp.int
			servorate=ratetemp.int
			servo[i]={"Servo Min":servomin,"Servo Max":servomax,"Servo Mid":servomid,"Servo Rate":servorate}
			calcchecksum=calcchecksum^ord(min1)^ord(min2)^ord(max1)^ord(max2)^ord(mid1)^ord(mid2)^ord(rate)
		checksum=ser.read()
		if ord(checksum)==calcchecksum:
			return servo
		else:
			print ("Bad Checksum")
	except Exception,e1:
		print("Error communicating..."+str(e1))


connect()






