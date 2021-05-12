import subprocess
import shlex
import threading
import time
import os
import signal

buffer = []

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
            p = [float(pnt.split()[1]), float(pnt.split()[3]), time.time()]
            pc.AddData(p)
    #print(pc.GetData())
            

def run_command(command, buffer, pc,timeout):
    count = 0
    process = subprocess.Popen(shlex.split(command), stdout=subprocess.PIPE)
    while True:
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
    #os.killpg(os.getpgid(process.pid), signal.SIGTERM)

LiDARThread = threading.Thread(target=run_command, args=('sudo ./ultra_simple /dev/ttyUSB0',buffer,PointCloud,False))
LiDARThread.start()
Start = time.time()
LiDARThread.join(30)
print(time.time() - Start)
#print(PointCloud.GetData())



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

