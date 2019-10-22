#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages for all 3 onboard ADCs
## to the 'Multi-ADC' topic

import rospy
import time
import Adafruit_ADS1x15
from std_msgs.msg import String

adc1 = Adafruit_ADS1x15.ADS1115(address=0x48, busnum=2)
adc2 = Adafruit_ADS1x15.ADS1115(address=0x49, busnum=2)
adc3 = Adafruit_ADS1x15.ADS1115(address=0x4b, busnum=2)

GAIN = 1

print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)


def talker():
    pub = rospy.Publisher('Multi-ADC', String, queue_size=10)
    rospy.init_node('ADC_Pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    values1 = [0]*4
    values2 = [0]*4
    values3 = [0]*4
    for i in range(4):
    	values1[i] = adc1.read_adc(i, gain=GAIN)
	values2[i] = adc2.read_adc(i, gain=GAIN)
	values3[i] = adc3.read_adc(i, gain=GAIN)
    reading = ('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values1) + '| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values2) + '| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values3))

    while not rospy.is_shutdown():
        hello_str = "%s" % reading
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
