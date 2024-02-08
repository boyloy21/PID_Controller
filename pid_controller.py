import numpy as np

class PID_controller:
    def __init__(self,kp,ki,kd,dt,alpha,integral_min,integral_max,output_min,output_max):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        self.dt=dt
        self.integral = 0
        self.derivative = 0
        self.alpha = alpha
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.output_max = output_max
        self.output_min = output_min
        self.derror = 0.0
    def calculate_PID(self,error):
        #Propotional error=[1,2,3,4,5]
        propotional=self.kp*error[-1]
        #Integral
        integral = self.integral + self.ki * 0.5 *(error[-1] + error[-2]) * self.dt
        self.integral=integral
        # Integral wineup
        if (integral > self.integral_max):
            integral = self.integral_max
        elif (integral < self.integral_min):
            integral = self.integral_min
        else:
            integral = integral
        #Derivative
        error_est = error[-1] - error[-2]
        self.derror = self.derror*(1-self.alpha) + (self.alpha*error_est)
        # derivative = self.kd * (errors[-1] - errors[-2])/self.dt
        derivative = self.kd*(self.derror/self.dt)
        #output
        output=propotional+integral+derivative
        #Output Satuartion
        if (output > self.output_max):
            output = self.output_max
        elif (output < self.output_min):
            output = self.output_min
        else:
            output = output
        return output
    
if __name__ == "__main__":
    PID_controller()