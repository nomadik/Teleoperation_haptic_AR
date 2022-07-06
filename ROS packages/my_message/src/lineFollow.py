import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String, Float64, Float32

from my_message.msg import GraphData

amp =1 

f = 1000
fs = 20000
T = 1/f
Ts = 1/fs


continous = True

#y = [amp.np.sin(2*np.pi*f*(i/fs)) for i in x]

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.iFrame = 0
        self.sum = 0.0
        self.ln2, = plt.plot([],[], 'g')

        self.ln, = plt.plot([], [], 'r')

        self.x_data, self.y_data = [0] , [0]
        self.y_desired_data, self.x_desired_data = [0] , [0] 

        self.count=0

        
    def plot_init(self, dt = Ts):
        self.dt = dt
        self.maxt = 5000
        self.ax.set_xlim(0, self.maxt)
        self.ax.set_ylim(-20, 20)
        return self.ln, self.ln2
    

    def odom_callback(self, msg):
        self.force_data = msg.data
        self.y_data.append(-self.force_data)
        #print(self.force_data)
        x_index = len(self.x_data)
      
        x_index2 = 2500+x_index
        
        if x_index <2500 or np.sin((x_index2/fs)*2*np.pi*f/100)*6<0:
            self.y_desired_data.append(0)
        else:
            self.y_desired_data.append(-np.sin((x_index2/fs)*2*np.pi*f/100)*6)

        self.x_desired_data.append(x_index2+1)
        self.x_data.append(x_index+1)
        
        if x_index>2500:
            dif = self.y_data[x_index] - self.y_desired_data[x_index-2500]
        else:
            dif=0
        squared_dif = dif**2

        self.sum = self.sum + squared_dif
        #if x_index!=0:
        #    MSE = self.sum/x_index
            #print(MSE)
            #print("actual data:"+ str(self.y_data[x_index]))
            #print("desired data:"+ str(self.y_desired_data[x_index]))
        #pub = rospy.Publisher('force_data', GraphData, queue_size=1)
        #msg2 = GraphData()
        #msg2.ms_error = round(MSE,3)
        #msg2.y_desired = self.y_desired_data[x_index]
        #msg2.y_sensed = self.y_data[x_index]
        #msg2.x_value = x_index/500
        #pub2.publish(msg)


    
    def update_plot(self, frame):
        
        self.count+=1
        lastt=self.x_data[-1]
        if continous:
            if lastt >self.x_data[0]+self.maxt/2:
                self.ax.set_xlim(lastt - self.maxt/2,lastt + self.maxt/2)
        t = self.x_data[-1] +self.dt
        self.x_data.append(t)
        self.y_data.append(-self.force_data)


        t2 = self.x_desired_data[-1]+self.dt
        if t < 2500 or np.sin((t2/fs)*2*np.pi*f/100)*6<0:
                self.y_desired_data.append(0)
        else:
            self.y_desired_data.append(-np.sin((t2/fs)*2*np.pi*f/100)*6)
        self.x_desired_data.append(t2)
          
        self.ln2.set_data(self.x_desired_data, self.y_desired_data)


        return self.ln , self.ln2


rospy.init_node('force_visual_node')
vis = Visualiser()
sub = rospy.Subscriber('/chatter', Float32, vis.odom_callback)

#ani = FuncAnimation(vis.fig, vis.update_plot, interval= 1)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init , interval= 1)
plt.show(block=True) 