#!/usr/bin/env python3
# pip3 install pyserial

import serial
import select
import time



def main():
    timeout = 10
    conn = serial.Serial('/dev/ttyACM0', 57600, timeout=0) # USB Serial setup
    read,_,_ = select.select([conn], [], [], timeout)

    try:
        while True:
            read_data = str(conn.readline(), encoding='utf-8', errors='ignore')
            if len(read_data)==0:
                time.sleep(0.016) # 60Hz
                continue
            print("{}".format(read_data))
    finally:
        conn.close()

if __name__ == '__main__':
    main()
