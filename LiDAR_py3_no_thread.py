#!/usr/bin/env python3

import subprocess
import shlex
import threading
import time
import os
import signal

buffer = []
continue_loop = True

class Cloud:
    def __init__(self):
        self.Data = [None] * 35999

    def AddData (self, point):
        self.Data[int(point[0]*100)%35999] = point

    def GetData (self):
        return self.Data

PointCloud = Cloud()

def process_data(buffer, pc):

    for point in buffer:
        if(point[0]==116):
            pnt = point.decode("utf-8")
            if (float(pnt.split()[3]) == 0.0):
                 continue
            p = [float(pnt.split()[1]), float(pnt.split()[3]), time.time()]
            pc.AddData(p)

def run_command(command, buffer, pc):
    count = 0
    try:
        process = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE, shell=True)

        while True:
            output = process.stdout.readline()

            if output == '' and process.poll() is not None:
                break
            if output:
                buffer.insert(0, output.strip())
                process_data(buffer, pc)
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)

run_command('sudo ./ultra_simple /dev/ttyUSB0',buffer,PointCloud)
