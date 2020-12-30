import serial
import pynmea2

"""
gpsTest.py
just print lat, lon gpsData 
@authors: Chanyang Yim
"""


while True:
    with serial.Serial('/dev/ttyACM0',baudrate=9600,timeout=1) as ser:
        msg = ser.readline().decode()
        #print(msg)
        if "GNRMC" not in msg:
            continue
        gpsCoord = pynmea2.parse(msg)
        #print(gpsCoord.longitude)
        print("[",gpsCoord.latitude,",",gpsCoord.longitude,"]")
        #print(ser.readline())
