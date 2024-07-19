# adapted from https://github.com/jeremy7710/Simple-PID/blob/master/Simple_PID_Python/PID_Demo.py

class PID:

    def __init__(self, Kp = 0.5, Ki = 0.5, Kd = 0.1, SetPoint = 1, dt = 0.1):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.SetPoint = SetPoint
        self.dt = dt
        
        self.clear()

    def clear(self):
        
        self.integral = 0.0
        self.pre_err = 0.0
        self.feedback_value = 0.0
        
        self.Yp = 0.0
        
    def compute(self, feedback_value = None):

        err, output, derivative = 0, 0, 0
        
        if(feedback_value is not None):
            self.feedback_value = feedback_value
            
        err = self.SetPoint - self.feedback_value
        self.integral += err * self.dt
        derivative = (err - self.pre_err) / self.dt
        output = (self.Kp * err + self.Ki * self.integral + self.Kd * derivative)


        self.pre_err = err
        self.feedback_value = output

        return (output, err, derivative, self.integral)
    
    
    # if you don't need lowpassFilter, you can delete the code below
    def lowpassFilter(self, X, beta):
    
        Y = beta * X + (1 - beta) * self.Yp
        self.Yp = Y

        return Y


# all you need is set the kp, ki, kd, it's really simple.
# you can set your setpoint and sample time too.
pid = PID(1, 10, 0.001, SetPoint=2, dt = 0.1)

y_list = []
e_list = []
d_list = []
i_list = []

y_list.append(0)

x=0
for i in range(7):
    
    # throw your feadback signal to pid
    x, e, d, i=pid.compute(x)
    
    # your system here, in this demo I use a lowpass filter to simulate a system.
    x=pid.lowpassFilter(x,0.5)
    
    # make a list to draw picture
    y_list.append(x)

    # also accumulate error, derivative and integral
    e_list.append(e)
    d_list.append(d)
    i_list.append(i)

# get the correct number of points
e_list.append(e_list[-1])
d_list.append(d_list[-1])
i_list.append(i_list[-1])

# draw picture
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d

x_list = range(8)
x_arr = np.array(x_list)
x_smooth = np.linspace(x_arr.min(), x_arr.max(), 300)

# y_list = spline(x_list, y_list, x_smooth) - no longer supported by scipy
y_list = interp1d(x_list, y_list, kind="cubic")(x_smooth)
e_list = interp1d(x_list, e_list, kind="cubic")(x_smooth)
d_list = interp1d(x_list, d_list, kind="cubic")(x_smooth)
i_list = interp1d(x_list, i_list, kind="cubic")(x_smooth)

# value against error
plt.plot(x_smooth, y_list, label="value")
plt.plot(x_smooth, e_list, label="error")
plt.legend(loc="center right")
plt.xlabel("time samples")
plt.ylabel("value")
plt.savefig('pid_value_error.png')
plt.clf()

plt.plot(x_smooth, e_list, label="error")
plt.plot(x_smooth, d_list, label="derivative")
plt.legend(loc="upper right")
plt.xlabel("time samples")
plt.ylabel("value")
plt.savefig('pid_error_d.png')
plt.clf()

#plt.plot(x_smooth, e_list, label="error")
plt.plot(x_smooth, i_list, label="integral")
plt.legend(loc="upper right")
plt.xlabel("time samples")
plt.ylabel("value")
plt.savefig('pid_error_i.png')
plt.clf()

#plt.show()

pid.clear()