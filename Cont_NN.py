import numpy as np
import Parameters as params
import math


# import time as ti

class Calculus:
    def __init__(self):
        self.derivative = []
        self.second_derivative = []
        self.integral = []

    def get_real_derivative(self, y, t):
        #print('y',len(y))
        if len(y) == 1:
            self.derivative.append( 0)
        # elif t[-1]-t[-2]==0:
        #   self.derivative=np.append(self.derivative,0)#self.derivative[-1])
        else:
            #print('y',y)
            #print('T',t)
            if t[-1] - t[-2] == 0:
                self.derivative.append(0)
            else:
                self.derivative.append((y[-1] - y[-2]) / (t[-1] - t[-2]))

    def get_real_integral(self, y, t):
        if len(y) == 1:
            self.integral.append(0)
        else:
            self.integral.append(self.integral[-1] + y[-1] * (t[-1] - t[-2]))  # simpson(y,t))
'''
    def get_derivative(self, y):
        if len(y) == 1:
            self.derivative = np.append(self.derivative, y[0])
        else:
            self.derivative = np.append(self.derivative, y[-1] - y[-2])

    def get_integral(self, y):
        if len(y) == 1:
            self.integral = np.append(self.integral, y[0])
        else:
            self.integral = np.append(self.integral, self.integral[-1] + y[-1])
'''

class Analysis:
    def __init__(self):
        self.calc_1 = Calculus()
        self.calc_2 = Calculus()

    def analyse(self, y, t, ss_value=1):
        overshoot = (np.max(y) - ss_value) / ss_value
        max_y = np.max(y)
        # index_max_y=np.argwhere(y==max_y)[0][0]
        # min_after_step = np.min(y[index_max_y:])
        check = True
        index = -1
        while check:
            if y[index] >= 1.02 * ss_value or y[index] <= 0.98 * ss_value:
                settling_time = t[index]
                check = False
            index -= 1
        self.overshoot = overshoot
        self.settling_time = settling_time
        #self.summed_error = np.sum(np.abs(y - ss_value))
        self.summed_error=0
        for i in range(len(y)):
            self.summed_error+=np.abs(y[i]-ss_value)
        self.avg_error = self.summed_error / len(y)
        self.ss_value = ss_value

        for i in range(len(y)):
            self.calc_1.get_real_derivative(y[:i + 1], t[:i + 1])
            self.calc_2.get_real_derivative(self.calc_1.derivative[:i + 1], t[:i + 1])

        self.velocity = self.calc_1.derivative / (60 * 2 * math.pi)  # rpm
        self.acceleration = self.calc_2.derivative  # rad/s²
        self.torque = self.acceleration * math.pi * params.J  # N/m


class PID(Calculus, Analysis):
    def __init__(self, K=[9.778846736, 1.40789948,  0.177706101]):  # [97.78846736, 13.40789948,  1.77706101]  [283.41320667,   9.70358294,   1.92906719]
        self.Kp, self.Ki, self.Kd = K
        self.action = 0#np.array([])
        # self.error=np.array([])
        self.error = []
        Calculus.__init__(self)
        Analysis.__init__(self)

    def get_action(self, ref, current_value, times):
        #print('called')
        # self.error=np.append(self.error,ref[-1]-current_value[-1])
        self.error.append(ref[-1] - current_value[-1])
        self.get_real_derivative(self.error, times)
        self.get_real_integral(self.error, times)
        #print('error',self.error[-1])
        #print('derivative',self.derivative[-1])
        #print('integral',self.integral[-1])
        # Temporary#
        self.integral_action = self.Ki * self.integral[-1]
        self.derivative_action = self.Kd * self.derivative[-1]
        #print(self.error)
        self.proportional_action = self.Kp * self.error[-1]
        # </>#
        action = self.Kp * self.error[-1] + self.Ki * self.integral[-1] + self.Kd * self.derivative[-1]
        self.action = action #np.append(self.action, action)


class I_PD(Calculus, Analysis):
    def __init__(self, K=[21.8793059770378, 83.2760130980813, 1.27722711584499]):
        self.Kp, self.Ki, self.Kd = K
        self.action = np.array([0])
        self.error = np.array([0])

        Calculus.__init__(self)
        Analysis.__init__(self)

    def get_action(self, ref, current_value, times):
        self.error = np.append(self.error, [current_value[-1] - ref[-1]])
        self.get_derivative(current_value)
        self.get_integral(self.error)
        action = self.Kp * current_value[-1] + self.Ki * self.integral[-1] + self.Kd * self.derivative[-1]
        self.action = np.append(self.action, action)


class Estimator:
    def __init__(self, function, initial_values=[], initial_input=0, method='Euler'):

        if function == 'tip':
            self.coefs = params.coefs_beam_equation
        elif function == 'servo':
            self.coefs = params.coefs_servo_equation
        elif function == 'try':
            self.coefs = params.coefs_try
        else:
            raise Exception("Function argument must be either 'tip' or 'servo'.")

        self.ys = []
        if len(initial_values) == 0:
            for i in range(len(self.coefs) - 1):
                self.ys.append(0)
        else:
            if len(self.coefs) - 1 == len(initial_values):
                for value in initial_values:
                    self.ys.append(value)
            else:
                raise Exception('initial value dimension is equal to ys list dimension')

        new_highest_order = initial_input
        for i in range(len(self.coefs) - 1):
            new_highest_order -= self.ys[i] * self.coefs[i]
        new_highest_order /= self.coefs[-1]
        self.ys.append(new_highest_order)

        if method == 'Euler':
            self.estimator = euler_update

        elif method=='Modified Euler':
            self.estimator=modified_euler_update

        elif method == 'Aldrabated':
            self.estimator = aldrabated

    def estimate(self, times, latest_input):
        self.ys = self.estimator(times, self.ys, latest_input, self.coefs)
        #print('print from conts',self.ys)

# NOTES FOR FOLLOWING FUNCTIONS
# y = list of lists containing values of y dy ddy etc. [y,dy,ddy,...]
# input = list of values of motor angle or control action
# coefs = list of coefs: [C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
def euler_update(times, prev_y, latest_input, coefs, subit=5):
    step = (times[-1] - times[-2]) / subit
    # first iteration
    new_ys = []
    new_highest_order = latest_input
    for i in range(len(prev_y) - 1):
        new = prev_y[i] + step * prev_y[i + 1]
        new_highest_order -= coefs[i] * new
        new_ys.append(new)
    new_highest_order /= coefs[-1]
    new_ys.append(new_highest_order)
    # remaining iterations
    for i in range(1, subit):
        prev_y = new_ys
        new_ys = []
        new_highest_order = latest_input
        for i in range(len(prev_y) - 1):
            new = prev_y[i] + step * prev_y[i + 1]
            new_highest_order -= coefs[i] * new
            new_ys.append(new)
        new_highest_order /= coefs[-1]
        new_ys.append(new_highest_order)
    #print('from euler function',new_ys)
    return new_ys

def modified_euler_update(times, prev_y, latest_input, coefs,subit=10):
    step=(times[-1]-times[-2])/subit
    #first iteration
    pred_new_ys = []
    pred_new_highest_order = latest_input
    for i in range(len(prev_y) - 1):
        pred_new = prev_y[i] + step * prev_y[i + 1]
        pred_new_highest_order -= coefs[i] * pred_new
        pred_new_ys.append(pred_new)
    pred_new_highest_order /= coefs[-1]
    pred_new_ys.append(pred_new_highest_order)

    new_highest_order=latest_input
    new_ys=[]
    for i in range(1,len(prev_y)):
        avg_deriv=(prev_y[i]+pred_new_ys[i])/2
        new=prev_y[i-1]+step*avg_deriv
        new_highest_order-=coefs[i-1]*new
        new_ys.append(new)
    new_highest_order /= coefs[-1]
    new_ys.append(new_highest_order)

    #remaining iterations
    for i in range(1,subit):
        prev_y = new_ys
        pred_new_ys = []
        pred_new_highest_order = latest_input
        for i in range(len(prev_y)-1):
            pred_new = prev_y[i] + step * prev_y[i + 1]
            pred_new_highest_order -= coefs[i] * pred_new
            pred_new_ys.append(pred_new)
        pred_new_highest_order /= coefs[-1]
        pred_new_ys.append(pred_new_highest_order)

        new_highest_order=latest_input
        new_ys=[]
        for i in range(1,len(prev_y)):
            avg_deriv=(prev_y[i]+pred_new_ys[i])/2
            new=prev_y[i-1]+step*avg_deriv
            new_highest_order-=coefs[i-1]*new
            new_ys.append(new)
        new_highest_order /= coefs[-1]
        new_ys.append(new_highest_order)

    return new_ys
def aldrabated(times, y, inputs, coefs):
    if len(times) == 2:
        delta1 = times[-1] - times[-2]
        prev_y = y[0][-1]
        coef1 = coefs[0] + coefs[1] / delta1 + coefs[2] / delta1 ** 2
        coef2 = coefs[1] / delta1 + coefs[2] / delta1 ** 2
        new_y = (inputs[-1] + prev_y * coef2) / coef1

    else:
        delta1 = times[-1] - times[-2]
        delta2 = times[-2] - times[-3]
        prev_y = y[0][-1]
        prev2y = y[0][-2]
        coef1 = coefs[0] + coefs[1] / delta1 + coefs[2] / delta1 ** 2
        coef2 = coefs[1] / delta1 + coefs[2] * (1 / delta1 + 1 / delta2) / delta1
        coef3 = coefs[2] / (delta1 * delta2)
        new_y = (inputs[-1] + coef2 * prev_y - coef3 * prev2y) / coef1
    new_dy = (new_y - prev_y) / delta1
    new_d2y = (new_dy - y[1]) / delta1
    new_ys = []
    new_ys.append(new_y)
    new_ys.append(new_dy)
    new_ys.append(new_d2y)
    return new_ys


class Compensator(Calculus):
    def __init__(self, function='sigmoid'):
        self.comp_ref = np.array([])
        self.times_ref_change = np.array([])
        if function == 'cosine':
            self.comp_function = cosine_comp
            self.comp_update = cosine_comp_update
            self.calc = Calculus()
        elif function == 'sigmoid':
            self.comp_function = sigmoid_comp
            self.comp_update = sigmoid_comp_update
            self.calc = Calculus()

    def setup(self, syst_obj, optimized_coefs=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
        self.syst_obj = syst_obj
        self.optimized_coefs = optimized_coefs

    def update(self):
        self.times_ref_change = np.append(self.times_ref_change, self.syst_obj.times[-1])
        self.parameters = self.comp_update(self.syst_obj, self.optimized_coefs, self.calc)

    def compensate(self):
        new_comp = self.comp_function(self.syst_obj, self.times_ref_change, self.parameters)
        self.comp_ref = np.append(self.comp_ref, self.syst_obj.ref[-1] - new_comp)


def cosine_comp_update(syst_obj, optimized_coefs, calculus):
    calculus.get_real_derivative(syst_obj.motor_angle, syst_obj.times)
    last_ang_vel = calculus.derivative[-1]
    if isinstance(syst_obj.controller, (PID, I_PD)):
        delta_ref = syst_obj.ref[-1] - syst_obj.ref[-2] if len(syst_obj.ref) > 1 else syst_obj.ref[-1]
        arguments = [np.abs(delta_ref), (delta_ref / np.abs(delta_ref)) * last_ang_vel, \
                     syst_obj.controller.Kp, syst_obj.controller.Ki, syst_obj.controller.Kd]
    k = np.dot(optimized_coefs[:5], arguments)
    t_begin = np.dot(optimized_coefs[5:], arguments)
    return [k, t_begin]


def cosine_comp(syst_obj, times_ref_change, parameters):
    k, t_begin = parameters
    t = syst_obj.times[-1] - times_ref_change[-1]
    if t < t_begin:
        new_comp = k
    else:
        new_comp = k * math.cos(2 * math.pi * params.wn * (t - t_begin))
    return new_comp


def sigmoid_comp_update(syst_obj, optimized_coefs, calculus):
    calculus.get_real_derivative(syst_obj.motor_angle, syst_obj.times)
    last_ang_vel = calculus.derivative[-1]
    if isinstance(syst_obj.controller, (PID, I_PD)):
        delta_ref = syst_obj.ref[-1] - syst_obj.ref[-2] if len(syst_obj.ref) > 1 else syst_obj.ref[-1]
        arguments = [np.abs(delta_ref), (delta_ref / np.abs(delta_ref)) * last_ang_vel, \
                     syst_obj.controller.Kp, syst_obj.controller.Ki, syst_obj.controller.Kd]
    k1 = np.dot(optimized_coefs[:5], arguments)
    k2 = np.dot(optimized_coefs[5:10], arguments)
    t_offset = np.dot(optimized_coefs[10:], arguments)
    return [k1, k2, t_offset]


def sigmoid_comp(syst_obj, times_ref_change, parameters):
    k1, k2, t_offset = parameters
    t = syst_obj.times[-1] - times_ref_change[-1]
    new_comp = k1 * (1 - 1 / (1 + math.e ** (-k2 * (t - t_offset))))
    return new_comp







class tryer:
    def __init__(self):
        self.value=0

    def tries(self,ref,comp):
        self.value=ref-comp