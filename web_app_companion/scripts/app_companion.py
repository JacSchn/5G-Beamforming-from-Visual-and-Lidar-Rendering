#!/usr/bin/env python3

import argparse
import requests as req
from datetime import datetime
import time
import rospy
from web_app_companion.msg import Sensor as msgSensor

def parse_args():
    # Parse input arguments
    desc = 'Subscriber for the Sony IMX322 USB camera publisher.\nSaves image arrays to a specified folder as a flattened array.'
    parser = argparse.ArgumentParser(description=desc)
    parser.add_argument('--ip', dest='web_app_ip',
                        help='IP address of the device hosting the web application',
                        default=None, type=str)

    args = parser.parse_args()

    if args.web_app_ip == None:
        parser.error('--ip Must be set!')

    return args


class Sensor:
    def __init__(self, name: str) -> None:
        self.name = name
        self.state = False
        self.timestamp = datetime.now().strftime('%I:%M:%S%p')
        self.newState = False

def initJsonData(sensors: list) -> dict:
    '''
    Returns an array of dictionaries under key "sensor"
    Each dictionary is set up with a sensor's name, state, turnedOnAt, and turnedOffAt values
    '''
    if len(sensors) == 0:
        raise ValueError("Length of sensors must be greater than 0")

    retDict = {
        "sensor": [],
    }

    for sensor in sensors:
        retDict['sensor'].append({
            "name": sensor.name,
            "state": sensor.state,
            "timestamp": sensor.timestamp,
        })

    return retDict

def postSensorData(URL: str, sensors: list) -> bool:
    '''
    Send http header data of a sensor to app with url=URL and endpoint being /micro
    '''
    endpoint = "micro"
    URL = URL + endpoint
    data = initJsonData(sensors=sensors)

    ret_val = req.post(url=URL, json=data).text
    print(f'Return from post is {ret_val}')

    if ret_val == 'OK':
        print(f"Post request SUCCESS")
        return True
    elif ret_val == 500:
        print(f"Post request FAILURE")
        return False


def updateCurrentState(sensorStates: dict, sensors: list) -> list:
    '''
    Update the current state of a sensor if their state has been updated.
    '''
    updatedSensors = []
    for s in sensors:
        if s.name in sensorStates:
            s.state = sensorStates[s.name]
            s.timestamp = datetime.now().strftime('%I:%M:%S%p')
            s.newState = True
            updatedSensors.append(s)
    
    return updatedSensors

def getState(URL: str) -> dict:
    '''
    Get current state at endpoint /getState
    '''
    endpoint = "getState"
    URL += endpoint

    newStates = req.get(url=URL).json()
    print(newStates)
    for s in newStates:
        print(f'{s} current state is {newStates[f"{s}"]}')
    
    return newStates

def pubCurrState(sensors: list, pub) -> None:
    '''
    Publish the current state of a sensor if it has been updated
    '''
    msg = msgSensor()
    for s in sensors:
        msg.name = s.name
        msg.state = bool(s.state)
        pub.publish(msg)

def runApp():
    '''
    Connects to the web app and publishes which sensor state changed.
    All sensors begin in an off state.
    '''
    args = parse_args()
    sensors = [
        Sensor('front_usb'),
        Sensor('rear_usb'),
        Sensor('rp_lidar'),
        Sensor('veloview_lidar')
    ]

    URL = f'http://{args.web_app_ip}:9000/'

    print('Setting up app companion publisher on topic /sensor_status')
    pub = rospy.Publisher('sensor_status', msgSensor, queue_size=5)
    rospy.init_node('app_companion_pub', anonymous=True)
    # r = rospy.Rate(1) # 1hz
    # r.sleep()
    print('Initialized app companion publisher on topic /sensor_status')
    time.sleep(2)
    postSensorData(URL, sensors)

    while True:
        if rospy.is_shutdown():
            print("Terminating Web App Companion")
            break

        updatedSensors = updateCurrentState(getState(URL), sensors)
        if len(updatedSensors) > 0:
            success = postSensorData(URL, updatedSensors)
            if not success:
                # Try to post sensor info one more time
                time.sleep(1)
                postSensorData(URL, updatedSensors)

            pubCurrState(updatedSensors, pub)

        time.sleep(1)

if __name__ == '__main__':
    runApp()