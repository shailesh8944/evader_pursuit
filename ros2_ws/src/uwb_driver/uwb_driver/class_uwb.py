import rclpy
from rclpy.node import Node
import numpy as np
import cmath
from serial import Serial
import json
from geometry_msgs.msg import PointStamped

class UWB():

    node = None

    portName = None

    L = None
    B = None
    c1 = None

    def __init__(self, portName='/dev/ttyUSB1', topic='/makara_00/uwb_01', rate=5):
        
        self.topic = topic
        self.rate = rate

        self.L = 31.6
        self.B = 20.6 - 6.5   # Changed for half wave basin test
        self.c1 = 2.5

        # USB PORT
        self.portName = portName
        
        # Create Serial object:
        self.ser = Serial(self.portName) # Also opens the port during object creation
        self.ser.close() # To set parameters

        # Change settings for Arduino default Serial.begin:
        self.ser.baudrate=115200
        self.ser.port=portName
        self.ser.bytesize=8
        self.ser.parity='N'
        self.ser.stopbits=1
        self.ser.timeout=0.1

        # Open port
        self.ser.open()
        self.ser.reset_input_buffer()

        # Start ROS2 Node
        rclpy.init()
        self.node = Node('UWB')

        # Initialize UWB object
        self.uwb={}
        
        # Initialize the position of UWB Rover
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.register_uwb()


    def P123(self,r1,r2,r3):
        f = cmath.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r1**2 + 2*self.B**2*self.L**2*r3**2 - self.B**2*r2**4 + 2*self.B**2*r2**2*r3**2 - self.B**2*r3**4 - self.L**2*r1**4 + 2*self.L**2*r1**2*r2**2 - self.L**2*r2**4)
        if f.real == 0:
            return None
        if f.real != 0: 
            s = ((self.L**2 + r2**2 - r3**2)/(2*self.L), (self.B**2 + r1**2 - r2**2)/(2*self.B), self.c1 - np.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r1**2 + 2*self.B**2*self.L**2*r3**2 - self.B**2*r2**4 + 2*self.B**2*r2**2*r3**2 - self.B**2*r3**4 - self.L**2*r1**4 + 2*self.L**2*r1**2*r2**2 - self.L**2*r2**4)/(2*self.B*self.L))
            return [s[0],s[1]]
        
    def P124(self,r1,r2,r4):
        f = cmath.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r2**2 + 2*self.B**2*self.L**2*r4**2 - self.B**2*r1**4 + 2*self.B**2*r1**2*r4**2 - self.B**2*r4**4 - self.L**2*r1**4 + 2*self.L**2*r1**2*r2**2 - self.L**2*r2**4)
        if f.real == 0:
            return None
        if f.real != 0: 
            s = ((self.L**2 + r1**2 - r4**2)/(2*self.L), (self.B**2 + r1**2 - r2**2)/(2*self.B), self.c1 - np.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r2**2 + 2*self.B**2*self.L**2*r4**2 - self.B**2*r1**4 + 2*self.B**2*r1**2*r4**2 - self.B**2*r4**4 - self.L**2*r1**4 + 2*self.L**2*r1**2*r2**2 - self.L**2*r2**4)/(2*self.B*self.L))
            return [s[0],s[1]]
        
    def P143(self,r1,r4,r3):
        f = cmath.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r1**2 + 2*self.B**2*self.L**2*r3**2 - self.B**2*r1**4 + 2*self.B**2*r1**2*r4**2 - self.B**2*r4**4 - self.L**2*r3**4 + 2*self.L**2*r3**2*r4**2 - self.L**2*r4**4)
        if f.real == 0:
            return None
        if f.real != 0:
            s = ((self.L**2 + r1**2 - r4**2)/(2*self.L), -(-self.B**2 + r3**2 - r4**2)/(2*self.B), self.c1 - np.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r1**2 + 2*self.B**2*self.L**2*r3**2 - self.B**2*r1**4 + 2*self.B**2*r1**2*r4**2 - self.B**2*r4**4 - self.L**2*r3**4 + 2*self.L**2*r3**2*r4**2 - self.L**2*r4**4)/(2*self.B*self.L))
            return [s[0],s[1]]
        
    def P243(self,r2,r4,r3):
        f = cmath.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r2**2 + 2*self.B**2*self.L**2*r4**2 - self.B**2*r2**4 + 2*self.B**2*r2**2*r3**2 - self.B**2*r3**4 - self.L**2*r3**4 + 2*self.L**2*r3**2*r4**2 - self.L**2*r4**4)
        if f.real == 0:
            return None
        if f.real != 0: 
            s = ((self.L**2 + r2**2 - r3**2)/(2*self.L), -(-self.B**2 + r3**2 - r4**2)/(2*self.B), self.c1 - np.sqrt(-self.B**4*self.L**2 - self.B**2*self.L**4 + 2*self.B**2*self.L**2*r2**2 + 2*self.B**2*self.L**2*r4**2 - self.B**2*r2**4 + 2*self.B**2*r2**2*r3**2 - self.B**2*r3**4 - self.L**2*r3**4 + 2*self.L**2*r3**2*r4**2 - self.L**2*r4**4)/(2*self.B*self.L))
            return [s[0],s[1]]
    
    def register_uwb(self):
        self.uwb['pub'] = self.node.create_publisher(PointStamped, self.topic, self.rate)
        self.uwb['timer'] = self.node.create_timer(1.0/self.rate, callback=self.publish_uwb)
    
    def publish_uwb(self):
        current_time = self.node.get_clock().now()

        # Create PointStamped message
        pt = PointStamped()
        pt.header.stamp = current_time.to_msg()
        pt.x = self.x
        pt.y = self.y
        pt.z = self.z

        self.uwb['pub'].publish(pt)

    def start_uwb(self):
        while True:            
            try:                
                N = self.ser.in_waiting        
                data = self.ser.readline() 
                data = data.decode('utf-8')
                f = json.loads(data)
                dis = {}
                for i in f['links']:
                    if int(i["A"])==1786:
                        dis["r1"]=float(i["R"])
                    if int(i["A"])==1787:
                        dis["r2"]=float(i["R"])
                    if int(i["A"])==1788:
                        dis["r3"]=float(i["R"])
                    if int(i["A"])==1789:
                        dis["r4"]=float(i["R"])
                
                if len(dis)<3:
                    self.node.get_logger().info(f"No Fix.  {len(dis)} anchors detected")
                
                elif len(dis)==3:
                    self.node.get_logger().info("3d fix")
                    try:
                        if set(dis.keys()) == {'r1','r2','r3'}:
                            fs = self.P123(dis['r1'],dis['r2'],dis['r3'])
                        if set(dis.keys()) == {'r1','r2','r4'}:
                            fs = self.P124(dis['r1'],dis['r2'],dis['r4'])
                        if set(dis.keys()) == {'r1','r4','r3'}:
                            fs = self.P143(dis['r1'],dis['r4'],dis['r3'])
                        if set(dis.keys()) == {'r2','r4','r3'}:
                            fs = self.P243(dis['r2'],dis['r4'],dis['r3'])
                        
                        self.x = fs[0]
                        self.y = fs[1]                        
                        
                    except RuntimeError:
                        self.node.get_logger().warning("Value error")
                
                elif len(dis)==4:
                    try:
                        self.node.get_logger().info("4d Fix")
                        
                        fs1 = self.P123(dis['r1'],dis['r2'],dis['r3'])
                        fs2 = self.P124(dis['r1'],dis['r2'],dis['r4'])
                        fs3 = self.P143(dis['r1'],dis['r4'],dis['r3'])
                        fs4 = self.P243(dis['r2'],dis['r4'],dis['r3'])
                        fsx = np.mean([fs1[0],fs2[0],fs3[0],fs4[0]])
                        fsy = np.mean([fs1[1],fs2[1],fs3[1],fs4[1]])
                        fs = [fsx,fsy]
                        
                        self.x = fs[0]
                        self.y = fs[1]                       
                    
                    except RuntimeError:
                        self.node.get_logger().warning("value error")
                
            except KeyboardInterrupt:
                break

            except Exception as e:
                pass
        self.ser.close()