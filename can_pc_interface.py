import serial

#Configure serial port
#115200 baud, 8 bits, 1 stop, no parity
ser = serial.Serial('/dev/ttyUSB0',baudrate=115200,timeout=0.1)
ser.close() #Close the port if already open
ser.open()
i = 0
j = 0
while 1:
    #Check if there is data to tx
    if(i == 10):
        i = 0
	ser.write(chr(13))#CR
	ser.write('can tx 0x0')
	ser.write(chr(ord('0')+j))
	ser.write(chr(13))#CR
	j = j + 1
	if(j == 10):
            j = 0
	#ser.read(16)
    #Check if there is data to rx
    #print ser.read(100)
    ser.write(chr(13))#CR
    ser.write('can rx')
    ser.write(chr(13))#CR
    buf = ser.read(100)
    idx = buf.find('can rx')
    if(idx >= 0):
        try:
	    rx_byte = buf[idx+8:idx+10]
	    print rx_byte
	    rx_num = int(rx_byte,16)
	    print "received"+str(rx_num)
        except:
	    if(rx_byte == 'na'):
	        print "No Data"
    i = i + 1
    