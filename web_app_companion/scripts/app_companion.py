#!/usr/bin/env python3
from typing import List
import requests as req
import time
import rospy
from web_app_companion.msg import String
from usb_cam import Sensor as msgSensor

class Sensor:
    def __init__(self, name: str) -> None:
        self.name = name
        self.state = False
        self.timeTurnedOn = 0
        self.timeTurnedOff = time.time()

def initJsonData(sensors: list) -> dict:
    '''
    Returns an array of dictionaries under key "sensor"
    Each dictionary is set up with a sensor's name, status, turnedOnAt, and turnedOffAt values
    '''
    retDict = {
        "sensor": []
    }

    if len(sensors) == 0:
        retDict['sensor'].append({
            "name": '',
            "status": False,
            "turnedOnAt": None,
            "turnedOffAt": None
        })
        return retDict

    for sensor in sensors:
        retDict['sensor'].append({
            "name": sensor.name,
            "status": sensor.status,
            "turnedOnAt": sensor.timeTurnedOn,
            "turnedOffAt": sensor.timeTurnedOff
        })

    
    return retDict

def postSensorData(URL: str, sensors: list) -> bool:
    '''
    Send http header data of a sensor to app with url=URL and endpoint being /micro
    '''
    endpoint = "micro"
    URL.join(endpoint)
    data = initJsonData(sensors=sensors)

    ret_val = req.post(url=URL, json=data).txt
    if ret_val == 200:
        print(f"Post request SUCCESS")
        return True
    elif ret_val == 500:
        print(f"Post request FAILURE")
        return False


def getState(URL: str, sensors: list) -> bool:
    '''
    Get current state at endpoint /getState
    '''
    endpoint = "getState"
    URL.join(endpoint)
    data = initJsonData(sensors=sensors)

    sensorStates = req.get(url=URL, json=data).json()
    
    print(sensorStates)
    
    return sensorStates

def pubCurrState(sensor: Sensor, pub) -> None:
    '''
    Publish the current state of a sensor.
    '''
    msg = msgSensor()
    msg.name = sensor.name
    msg.state = sensor.state
    pub.publish(msg)
    # for sensor in sensors:
    #     pub.Publish()

def runApp():
    '''
    Connects to the web app and publishes which sensor state changed.
    All sensors begin in an off state.
    '''
    
    front_usb = Sensor('front_usb')
    rear_usb = Sensor('rear_usb')
    rp_lidar = Sensor('rp_lidar')
    veloview_lidar = Sensor('veloview_lidar')
    URL = 'http://localhost:9000/'

    print('Setting up app companion publisher on topic /sensor_status')
    pub = rospy.Publisher('sensor_status', Sensor, queue_size=4)
    rospy.init_node('app_companion_pub', anonymous=True)
    # r = rospy.Rate(1) # 1hz
    # r.sleep()
    print('Initialized app companion publisher on topic /sensor_status')
    time.sleep(2)
    
    while True:
        if rospy.is_shutdown():
            print("Terminating Web App Companion")
            break
        pubCurrState(pub)
        time.sleep(2)

        

if __name__ == '__main__':
    runApp()
