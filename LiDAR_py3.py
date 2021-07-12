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
    #print(pc.GetData())
    #print("Done with process_data")

def run_command(command, buffer, pc,timeout):
    count = 0
    process = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE)
    LiDARThread = threading.currentThread()
    while getattr(LiDARThread, "do_run", True):
        output = process.stdout.readline()
        if(timeout):
            count += 1
            if(count > 1):
                break
        if output == '' and process.poll() is not None:
            break
        if output:
            buffer.insert(0, output.strip())
            process_data(buffer, pc)
    print("Out of run_command loop")
    print("Process PID %s Type: %s" % (process.pid, type(process.pid)))
    print("OS.getpgid(process.pid) %s" % os.getpgid(process.pid))
    print("os.getppid() %s" % os.getppid())
    print("process.pid + 1 %s Type: %s" % ((process.pid + 1), type(process.pid + 1)))
    #process.send_signal(signal.SIGINT)
    #os.killpg(os.getpgid(process.pid), signal.CTRL_C_EVENT)
    stop_LiDAR(process)
    #process.terminate() #Not permitted. Use 'sudo' when running Program to allow
    #os.killpg(os.getpgid(process.pid + 1), signal.SIGINT) #kills program

def stop_LiDAR(self):
        print("Self PID: %s" % self.pid)
        command = "sudo kill -2 %s" % str(self.pid + 1)
        subprocess.run(["ps", "all"])
        subprocess.run(shlex.split(command))
        subprocess.run(["ps", "all"])
        print("Killed")

LiDARThread = threading.Thread(target=run_command, args=('sudo ./ultra_simple /dev/ttyUSB0',buffer,PointCloud,False))
LiDARThread.start()
Start = time.time()
LiDARThread.join(60)
if LiDARThread.isAlive():
    print("I AM STILL ALIVE!!")
    LiDARThread.do_run = False
    #LiDARThread.join()
    if LiDARThread.isAlive():
        print("NOT DEAD YET")
print(time.time() - Start)
# print(PointCloud.GetData())



import numpy as np
import matplotlib.pyplot as plot

plot.axes(projection='polar')

# Set the title of the polar plot
plot.title('Circle in polar format:r=R')

dat = PointCloud.GetData()


for point in dat:
    if(point != None):
        plot.polar(np.deg2rad(point[0]), point[1],'o')
# Display the Polar plot


plot.show()

