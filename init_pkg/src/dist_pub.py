#! /usr/bin/python3
import serial
import rospy
from std_msgs.msg import Float32, Bool

class DistanceParser:
    def __init__(self, port):
        self.ser = serial.Serial(port, 9600)
        self.contact = False
        self.last_contact = None

    def parse(self):
        data = self.ser.readline().decode('utf-8').strip()
        contact = data
        # print(contact)
        if contact == '1':
            self.contact = False
            # print('No contact')
        else:
            self.contact = True
           
        contact = False
        if self.contact:
            if self.last_contact is None:
                self.last_contact = rospy.Time.now()
                contact = True
            else:
                if rospy.Time.now() - self.last_contact > rospy.Duration(30):
                    contact = True
                    self.last_contact = rospy.Time.now()
        return contact
   

if __name__ == '__main__':
    rospy.init_node('parser')
    parser = DistanceParser('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0')
    publisher_cont = rospy.Publisher('/contact', Bool, queue_size=10)
    while not rospy.is_shutdown():
        try:
            contact = parser.parse()
            if contact:
                publisher_cont.publish(True)
                print('Contact')
            rospy.sleep(0.005)
        except Exception as e:
            print(e)
