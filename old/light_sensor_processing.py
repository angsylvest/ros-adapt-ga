#! /usr/bin/python3
import board
import adafruit_tsl2591
import adafruit_tca9548a
import time

import rospy 
from std_msgs.msg import Float32 

i2c = board.I2C()
tca = adafruit_tca9548a.TCA9548A(i2c)
l1 = adafruit_tsl2591.TSL2591(tca[0])
l2 = adafruit_tsl2591.TSL2591(tca[1])
l3 = adafruit_tsl2591.TSL2591(tca[2])

def light_sensor_publisher():
    rospy.init_node("light_sensor_pub", anonymous=True)
    pub = rospy.publisher("light_pub_one", anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pub.publish(l1.lux)
        print("light detected from sensor 1:{0}lux", format(l1.lux))
        rate.sleep()

# while True:
    # print("light detected from sensor 1:{0}lux", format(l1.lux))
    # print("light detected from sensor 2:{0}lux", format(l2.lux))
    # print("light detected from sensor 3:{0}lux", format(l3.lux))
    # print("--------------------------------")
    # time.sleep(0.5)

if __name__ == '__main__':
    try: 
        light_sensor_publisher()
    except rospy.ROSInterruptException:
        pass
