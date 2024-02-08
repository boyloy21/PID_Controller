import numpy as np
import matplotlib.pyplot as plt
from bezier_path import calc_4points_bezier_path
from pid_controller import PID_controller

omega_min=-3.14
omega_max=3.14
lx = 0.165
ly = 0.225 
R = 0.076
start_x = 0.0
start_y = 0.0
start_yaw = 0
Omega = 3.14
end_x = 5.0
end_y = 5.0
end_yaw = 1.57
offset = 2
Vxd_prev = 0
Vyd_prev = 0

sampling_time=0.01  
sim_time=300


    
class mecanum:
    def __init__ (self,r,lx,ly):
        self.r=r
        self.lx=lx
        self.ly=ly
    def inverse_kinematic(self,vx,vy,w):
        #Motor1
        w1=(vx-vy-(self.lx+self.ly)*w)/self.r
        #Motror2
        w2=(vx+vy+(self.lx+self.ly)*w)/self.r
        #Motor3
        w3=(vx+vy-(self.lx+self.ly)*w)/self.r
        #Motor4
        w4=(vx-vy+(self.lx+self.ly)*w)/self.r
        return w1,w2,w3,w4
    def forward_kinematic(self,w1,w2,w3,w4):
        #Longitudinal_velocity
        Vx=(w1+w2+w3+w4)*self.r/4
        #Transversal Velocity
        Vy=(-w1+w2+w3-w4)*self.r/4
        #Angular Velocity
        Wz=(-w1+w2-w3+w4)*self.r/(4*(self.lx+self.ly))
        return Vx,Vy,Wz
    def discrete_state(self,x,y,yaw,w1,w2,w3,w4,dt):
        dx,dy,dyaw=self.forward_kinematic(w1,w2,w3,w4)
        x_next = x + dx * dt
        y_next = y + dy * dt
        yaw_next = yaw + dyaw * dt

        return x_next, y_next, yaw_next
# Arrow function
def plot_arrow(x, y, yaw, length=0.025 , width=0.165*4, fc="b", ec="k"):
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
            fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

#choosing kp,ki,kd for X
kp_x = 4.0
ki_x = 0.02
kd_x = 0.04

# Gain PID of y
kp_y = 4.0
ki_y = 0.02
kd_y = 0.04

# Gain PID of theta
kp_yaw = 4.0
ki_yaw = 0.02
kd_yaw = 0.04

# Initialze position
x0 = start_x
y0 = start_y
yaw0 = start_yaw

current_x = x0
current_y = y0
current_yaw = yaw0
current_xprev = 0.0
current_yprev = 0.0
current_yawprev = 0.0
Vyawd = 0.0
Vyd  =0.0

# Create mecanum_wheel robot class
mec=mecanum(R,lx,ly)
## Create PID for each x, y, yaw
integral= 3
output = 3
alpha = 0.5
pid_x=PID_controller(kp_x,ki_x,kd_x,sampling_time,alpha,-integral,integral,-output,output)
pid_y=PID_controller(kp_y,ki_y,kd_y,sampling_time,alpha,-integral,integral,-output,output)
pid_yaw=PID_controller(kp_yaw,ki_yaw,kd_yaw,sampling_time,alpha,-integral,integral,-output,output)
#Test trajectory normal
ref_path = np.array([end_x, end_y, end_yaw], dtype=np.float32)
#Test2:Trajectory_bezier_path
# path, _ = calc_4points_bezier_path(
#     start_x, start_y, start_yaw,
#     end_x, end_y, end_yaw,
#     offset
# )
# ax = path[:, 0]
# ay = path[:, 1]
# ayaw = np.append(np.arctan2(np.diff(ay), np.diff(ax)), end_yaw)
# ref_path = np.vstack([path[:, 0], path[:, 1], np.append(np.arctan2(np.diff(path[:, 1]), np.diff(path[:, 0])), end_yaw)])
# error_x = [ref_path[0, 0]-current_x]
# error_y = [ref_path[1, 0]-current_y]
# error_yaw = [ref_path[2, 0]-current_yaw]

error_x=[ref_path[0]-current_x]
error_y=[ref_path[1]-current_y]
error_yaw=[ref_path[2]-current_yaw]

# Save history

hist_x = [current_x]
hist_y = [current_y]
hist_yaw = [current_yaw]

if __name__== "__main__":
    for t in range(sim_time):

        #Test2
        # if index >=ref_path.shape[1]:
        #     index=ref_path.shape[1]-1
        # #loop  find error
        # error_x.append(ref_path[0,index]-current_x)
        # error_y.append(ref_path[1,index]-current_y)
        # error_yaw.append(ref_path[2,index]-current_yaw)
        
        #Test1
        error_x.append(ref_path[0]-current_x)
        error_y.append(ref_path[1]-current_y)
        error_yaw.append(ref_path[2]-current_yaw)
        
        #calculate Pid
        output_vx=pid_x.calculate_PID(error_x)
        output_vy=pid_y.calculate_PID(error_y)
        output_vyaw=pid_yaw.calculate_PID(error_yaw)
        # print(output_vx)
        # print(output_vx)
        Vd = np.sqrt((current_x - current_xprev)**2+(current_y - current_yprev)**2)/sampling_time
        #Calculate Speed disired 
        Vxd = (current_x -current_xprev)/sampling_time
        Vyd = (current_y -current_yprev)/sampling_time
        Vyawd = (current_yaw -current_yawprev)/sampling_time
    
        #Calculate Speed Rotation
        Vxr = Vxd - output_vx
        Vyr = Vyd - output_vy
        Vyawr = Vyawd - output_vyaw
        # print(Vxr)
        current_xprev = current_x
        current_yprev = current_y
        current_yawprev = current_yaw
        Vxd_prev = Vxd
        Vyd_prev = Vyd
        # print(output_vx)
        # print(Vxr)
        W1,W2,W3,W4=mec.inverse_kinematic(Vxr,Vyr,Vyawr)

        # print(W1)
        #find vx vy vyaw from inverse kinematic
        w1,w2,w3,w4=mec.inverse_kinematic(output_vx,output_vy,output_vyaw)
        
        print(w1)
        #discrete state
        Vx,Vy,Vyaw = mec.forward_kinematic(w1,w2,w3,w4)
        print(Vx)
        # print(Vx)
        x_next,y_next,yaw_next=mec.discrete_state(current_x,current_y,current_yaw,w1,w2,w3,w4,sampling_time)
        
        current_x=x_next
        current_y=y_next
        current_yaw=yaw_next
        
        
        hist_x.append(current_x)
        hist_y.append(current_y)
        hist_yaw.append(current_yaw)
        

        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
        plot_arrow(current_x, current_y, current_yaw)
        plt.plot(5, 5)
        plt.plot(ref_path[0], ref_path[1], marker="x", color="blue", label="Input Trajectory")
        plt.plot(hist_x, hist_y, color="blue", label="PID Track")
        plt.title("Velocity of robot [m/sec]:" + str(round(output_vx, 2)))
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        plt.pause(0.0001)
