import serial
import micropyGPS
import threading
import time

gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSIuWFNg𐶐¬·éB
                                     # ø̓^C][̎·Əo̓tH[}bg

def rungps(): # GPSW[ðǂ݁AGPSIuWFNgðXV·é
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # ŏÌ1s͒r¼[ȃf[^[ªǂ߂邱Ƃª é̂ŁA̂Ăé    print(s)
    while True:
        sentence = s.readline().decode('utf-8') # GPSf[^[ðǂ݁A¶ñɕϊ··é        print(sentence)
        if sentence == "":
            continue
        if sentence[0] != '$': # 擪ª'$'łȂ¯êΎ̂Ăé
            continue
        for x in sentence: # ǂ񂾕¶ñðð͂µÄGPSIuWFNgɃf[^[ðǉÁAXV·é
            gps.update(x)

gpsthread = threading.Thread(target=rungps, args=()) # ã̊֐ðÀs·éXbh𐶐¬
gpsthread.daemon = True
gpsthread.start() # XbhðN®

while True:
    if gps.clean_sentences > 20: # ¿áñƂµ½f[^[ª éöx½܂Á½ço͂·é
        h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
        print(f"lat:{gps.latitude[0]}, lon:{gps.longitude[0]}")
    #time.sleep(3.0)
