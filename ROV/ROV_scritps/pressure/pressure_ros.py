#!/usr/bin/env python
import ms5837
import time
import rospy
from std_msgs.msg import String

sensor = ms5837.MS5837_30BA(2) # Specify I2C bus

# We must initialize the sensor before reading it
if not sensor.init():
        print "Sensor could not be initialized"
        exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print "Sensor read failed!"
    exit(1)

def pressure():
    pub = rospy.Publisher('pressure', String, queue_size=10)
    rospy.init_node('press-sensor', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        if sensor.read():
            print("P: %0.1f mbar  %0.3f psi\tT: %0.2f C  %0.2f F") % (
            sensor.pressure(), # Default is mbar (no arguments)
            sensor.pressure(ms5837.UNITS_psi), # Request psi
            sensor.temperature(), # Default is degrees C (no arguments)
            sensor.temperature(ms5837.UNITS_Farenheit)) # Request Farenheit
        else:
            print "Sensor read failed!"


    	freshwaterDepth = sensor.depth() # default is freshwater
	sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
	saltwaterDepth = sensor.depth() # No nead to read() again
	sensor.setFluidDensity(1000) # kg/m^3
	#print("Depth: %.3f m (freshwater)  %.3f m (saltwater)") % (freshwaterDepth, saltwaterDepth)

        pressure_str = "%.3f" % (freshwaterDepth)
        rospy.loginfo(pressure_str)
        pub.publish(pressure_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        pressure()
    except rospy.ROSInterruptException:
        pass

