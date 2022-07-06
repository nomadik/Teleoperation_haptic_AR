from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Float64, Float32
import rospy
import numpy as np
from franka_msgs.msg import FrankaState
from my_message.msg import GraphData, linefollowData

class Nodo():

    def __init__(self):
        sub1_2_3 = rospy.Subscriber('/linefollow_data', linefollowData, self.error_callback)
        sub4 = rospy.Subscriber('/wrench', WrenchStamped, self.weiss_callback)
        sub5 = rospy.Subscriber('/franka_state_controller/F_ext',WrenchStamped, self.robotf_callback)
        sub6 = rospy.Subscriber('/wrench2',WrenchStamped,self.weiss2_callback)
        sub7 = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.deltax_callback)
        sub8_9 = rospy.Subscriber('/tobeupdated', update, self.AR_callback) # for desired and actual AR force
        rospy.spin()

    #1_2_3
    def linefollow_callback(self,msg):
        self.mse=msg.mean_squared_error
        self.desForce=msg.y_desired
        self.event_force = msg.sensed

    #4  
    def weiss_callback(self, msg):
        self.weiss_force = -msg.wrench.force.z
    #5
    def robotf_callback(self, msg):
        self.sensed_robot_force = msg.wrench.force.z
    #6
    def weiss2_callback(self,msg):
        self.weiss2_force = msg.update
    
    #7
    def deltax_callback(self, msg):
        self.delta_x = msg.O_T_EE_d[14]
    

    #8_9
    def AR_callback(self, msg):
        self.actualAR = msg.tobeupdated
        self.desiredAR= msg.tobeupdated
        pub = rospy.Publisher('force_data', GraphData, queue_size=1)
        message = GraphData()
        message.ms_error=self.mse
        message.force_desired=self.desForce
        message.force_event= self.event_force
        message.weiss_z_force = self.weiss_force
        message.force_robot = self.sensed_robot_force
        message.delta_z = self.delta_x
        message.force_weiss2 = self.weiss2_force
        
        pub.publish(message)


if __name__ == '__main__':
    rospy.init_node('force_visual_node')
    #print("mode 0")
    s = Nodo(1000.0)	