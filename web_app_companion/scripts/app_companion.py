#!/usr/bin/env python3
import requests as req
import time
import rospy
from std_msgs.msg import String

class Sensor:
    def __init__(self, name: str) -> None:
        self.name = name
        self.statusOn = False


def postSensorData(URL: str, sensor: Sensor) -> bool:
    '''
    Send http header data of a sensor to app with url=URL and endpoint being /micro
    '''
    endpoint = "micro"
    URL.join(endpoint)
    data = {
        "sensor": sensor.name,
        "timestamp": sensor.timestamp
    }

    ret_val = req.post(url=URL, headers=data).text
    if ret_val == 200:
        print(f"Post request SUCCESS for {sensor.name}")
        return True
    elif ret_val == 500:
        print(f"Post request FAILURE for {sensor.name}")
        return False


def getState(URL: str) -> bool:
    '''
    Get current state at endpoint /getState
    '''
    endpoint = "getState"
    URL.join(endpoint)
    state = req.get(url=URL)
    print(f'My current state is {state.text}')
    return state.text

def pubInitState(sensors: list, pub) -> None:
    '''
    Publish to all sensors their initial state.
    '''
    pub.publish("0")
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
    pub = rospy.Publisher('sensor_status', String, queue_size=4)
    rospy.init_node('app_companion_pub', anonymous=True)
    print('Initialized app companion publisher on topic /sensor_status')
    time.sleep(2)
    
    while True:
        if rospy.is_shutdown():
            print("Terminating Web App Companion")
            break
        pubInitState(front_usb, pub=pub)
        time.sleep(2)

        

if __name__ == '__main__':
    runApp()
