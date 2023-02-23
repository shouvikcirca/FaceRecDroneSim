import serial
ser = serial.Serial('/dev/tty0')
ser.timeout = 0.5
for baudrate in ser.BAUDRATES:
    if 9600 <= baudrate <= 115200:
        ser.baudrate = baudrate
        ser.write(packet)
        resp = ser.read()
        if resp != '':
            break
if ser.baudrate > 115200:
    raise RuntimeError("Couldn't find appropriate baud rate!")
