#coding:utf-8
'''
树莓派WiFi无线视频小车机器人驱动源码
作者：liuviking
版权所有：小R科技（深圳市小二极客科技有限公司www.xiao-r.com）；WIFI机器人网论坛 www.wifi-robots.com
本代码可以自由修改，但禁止用作商业盈利目的！
本代码已申请软件著作权保护，如有侵权一经发现立即起诉！
'''


import RPi.GPIO as GPIO
import time
#######################################
#############信号引脚定义##############
#######################################
########LED口定义#################
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
LED0 = 10
LED1 = 9
LED2 = 25



#########led初始化为000##########
GPIO.setup(LED0,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(LED1,GPIO.OUT,initial=GPIO.HIGH)
GPIO.setup(LED2,GPIO.OUT,initial=GPIO.HIGH)


def	INIT_LED():#流水灯
	for i in range(0, 25):
		GPIO.output(LED0,False)
		GPIO.output(LED1,True)
		GPIO.output(LED2,True)
		time.sleep(0.3)
		GPIO.output(LED0,True)
		GPIO.output(LED1,False)
		GPIO.output(LED2,True)
		time.sleep(0.3)
		GPIO.output(LED0,True)
		GPIO.output(LED1,True)
		GPIO.output(LED2,False)
		time.sleep(0.3)
	for i in range(0, 10):
		GPIO.output(LED0,True)
		GPIO.output(LED1,True)
		GPIO.output(LED2,True)
		time.sleep(0.1)
		GPIO.output(LED0,False)
		GPIO.output(LED1,False)
		GPIO.output(LED2,False)
		time.sleep(0.1)

INIT_LED()


