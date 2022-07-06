import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Float64, Float32
from my_message.msg import GraphData, linefollowData
from datetime import datetime

f = 1000
fs = 2000
Ts = 1/fs


class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.sum = 0.0
        self.ln2, = plt.plot([],[], 'g')
        self.ln, = plt.plot([], [], 'r')
        self.x_data, self.y_data = [0] , [0]
        self.y_desired_data, self.x_desired_data = [0] , [0] 
        self.count=0
        self.sinwave=[0]
        self.trial=0
        self.start_compare=False

    def plot_init(self, dt = Ts):
        self.dt = dt
        self.maxt = 320
        self.ax.set_xlim(0, self.maxt)
        self.ax.set_ylim(-10, 2)
        return self.ln, self.ln2

    def odom_callback(self, msg):
        self.force_data = msg.data


    def update_plot(self,frame):
        #self.force_data = 0#msg.data
        x_index = len(self.x_data)
        x_index2 = self.maxt/2+x_index
        self.y_data.append(-self.force_data)
        self.x_data.append(x_index+1)

        print(x_index)
        if x_index>=int(self.maxt/2):
            self.start_compare=True

        lastt=self.x_data[-1]
        
        if lastt >self.maxt/2:
            self.ax.set_xlim(lastt - self.maxt/2, lastt + self.maxt/2)
        
        t = self.x_data[-1] +self.dt

        if x_index <self.maxt/2+100 or np.sin((x_index2/fs)*2*np.pi*f/100)*7>0:
            self.y_desired_data.append(0)
        else:
            self.y_desired_data.append(np.sin((x_index2/fs)*2*np.pi*f/100)*7)


        #sinwave to send to the AR
        self.sinwave.append(np.sin((x_index2/fs)*2*np.pi*f/100)*7)

        self.x_desired_data.append(x_index2+1)
          
        self.ln2.set_data(self.x_desired_data, self.y_desired_data)

        self.ln.set_data(self.x_data, self.y_data)
        
        pub = rospy.Publisher('linefollow_data', linefollowData, queue_size=1)
        msg2 = linefollowData()
        if self.start_compare:
            if x_index>self.maxt/2:
                dif = self.y_data[x_index] - self.y_desired_data[x_index-int(self.maxt/2)] #self.y_desired_data[int(x_index-self.maxt/4)]
            else:
                dif=0
            squared_dif = dif**2

            self.sum = self.sum + squared_dif
            if x_index!=0:
                MSE = self.sum/x_index
                print(MSE)
                print("actual data:"+ str(self.y_data[x_index]))
                print("desired data:"+ str(self.y_desired_data[x_index-int(self.maxt/2)]))

            msg2.mean_squared_error = round(MSE,3)
        else:
            msg2.mean_squared_error=0

        if self.start_compare:
            if self.y_desired_data[x_index-int(self.maxt/2)-1]>=self.y_desired_data[x_index-int(self.maxt/2)]:
                msg2.y_desired = self.y_desired_data[x_index-int(self.maxt/2)]
            else:
                msg2.y_desired = abs(self.y_desired_data[x_index-int(self.maxt/2)])
        else:
            msg2.y_desired=0

        msg2.y_sensed = self.y_data[x_index]

        if self.start_compare:
            msg2.wave= self.sinwave[x_index-int(self.maxt/2)]
        else:
            msg2.wave=0
        if self.start_compare:
            if self.y_desired_data[x_index-int(self.maxt/2)-1]==0 and self.y_desired_data[x_index-int(self.maxt/2)]<0:
                self.trial+=1

        msg2.numoftrial=self.trial
        #msg2.x_value = x_index/500 #???????


        pub.publish(msg2)

        return  self.ln, self.ln2



vis = Visualiser()
rospy.init_node('force_visual_node')
sub = rospy.Subscriber('/chatter', Float32, vis.odom_callback)
ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval=1, cache_frame_data=False)

plt.show(block=True) 
