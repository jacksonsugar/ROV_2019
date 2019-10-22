#!/usr/bin/env python
# Gstreamer camera switcher

import os
import time

camera = 0

def killstream():
	os.system("sudo pkill gst-launch-1.0")
	os.system("sudo sshpass -p 'pi' ssh pi@192.168.1.200 'pkill gst-launch-1.0'")


def camera1():
	os.system("sudo sshpass -p 'pi' ssh pi@192.168.1.200 'gst-launch-1.0 rkisp device=/dev/video0 sensor-id=1 io-mode=4 path-iqf=/etc/cam_iq/ov13850.xml ! video/x-raw,format=NV12,width=800,height=600,framerate=15/1 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.150 port=5000' &")
	os.system("gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=10 ! rtpjpegdepay ! jpegdec ! autovideosink &")


def camera2():
	os.system("sudo sshpass -p 'pi' ssh pi@192.168.1.200 'gst-launch-1.0 rkisp device=/dev/video6 sensor-id=5 io-mode=4 path-iqf=/etc/cam_iq/ov13850.xml ! video/x-raw,format=NV12,width=800,height=600,framerate=15/1 ! jpegenc ! rtpjpegpay ! udpsink host=192.168.1.150 port=5000' &")
	os.system("gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink &")


camera = input("Choose camera (1 or 2):  ")

if camera == 1:
	killstream()
	camera1()
	time.sleep(1/2)

elif camera == 2:
	killstream()
	camera2()
	time.sleep(1/2)

else:
	camera = input("Choose camera (1 or 2):  ")
