import serial,time
ser=serial.Serial()
ser.port="/dev/ttyACM0"

msp ident 100
msp status 101
msp raw imu 102
msp servo 103
msp motor 104
msp rc 105
msp raw gps 106
msp comp gps 107
msp attitude 108
msp altitude 109
msp bat 110
msp rc tuning 111
msp pid 112
msp box 113
msp misc 114
msp motor pins 115
msp box names 116
msp pid names 117
msp wp 118
msp box ids 119

mspset raw rc 200
mspset raw gps 201
mspset pid 202
mspset box 203
mspset rc tuning 204
mspset acc calibration 205
mspset mag calibration 206
mspset set misc 207
mspset reset conf 208
mspset wp set 209
mspset select setting 210
mspset spek bund 240
mspset eeprom write 250
mspset debug msg 253
mspset debug 254

checksum=(reduce(lambda x,y:x^y,data_raw+[cmd]+[data_size]) & 0xff)


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

CURRENT=MSP_STATUS
ser.baudrate=115200
ser.bytesize=serial.EIGHTBITS
ser.parity=serial.PARITY_NONE
serial.stopbits=serial.STOPBITS_ONE
ser.timeout=0
ser.xonxoff=False
ser.rtscts=False
ser.dsrdtr=False
ser.writeTimeout=2
def connect():
	try:
		ser.open()
	except Exception,e:
		print("Error opening serial port: "+str(e))
		exit()	
	if ser.isOpen():
		print("Serial port is open at"+ser.portstr)
	else:
	    	print("Cannot open serial port")
def disconnect():
	try:
		ser.close()
	except Exception,e:
		print("Eror closing serial port: "+str(e))
		exit()
def talk():
	try:
	        ser.flushInput()
	        ser.flushOutput()
	        ser.write(CURRENT)
	        print("Writing to "+ser.portstr+" "+CURRENT)
	        time.sleep(0.5)
	        numOfLines=0
	        while True:
	            response=ser.readline()
	            #print(type(response))
	            
	            print(response.encode("hex"))
	            numOfLines=numOfLines+1
	    
	            if(numOfLines>=1):
	                break
	except Exception,e1:
	        print("Error communicating..."+str(e1))
	
    

