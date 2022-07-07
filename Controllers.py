import numpy as np
import copy
import Parameters as params
import math
import matplotlib.pyplot as plt
import control as c

class Calculus:
    def __init__(self):
        self.derivative=np.array([])
        self.second_derivative=np.array([])
        self.integral=np.array([])

    def get_real_derivative(self,y,t):
        if len(y)==1:
            self.derivative=np.append(self.derivative,0)
        elif t[-1]-t[-2]==0:
            self.derivative=np.append(self.derivative,0)
        else:
            self.derivative=np.append(self.derivative,(y[-1]-y[-2])/(t[-1]-t[-2]))

    def get_real_integral(self,y,t):
        if len(y)==1:
            self.integral = np.append(self.integral, 0)
        else:
            self.integral=np.append(self.integral,self.integral[-1]+y[-1]*(t[-1]-t[-2]))

    def get_derivative(self,y):
        if len(y)==1:
            self.derivative = np.append(self.derivative,y[0])
        else:
            self.derivative=np.append(self.derivative,y[-1]-y[-2])

    def get_integral(self,y):
        if len(y)==1:
            self.integral=np.append(self.integral,y[0])
        else:
            self.integral=np.append(self.integral, self.integral[-1]+y[-1])


class Analysis:
    def __init__(self,syst=None,motor=False):
        self.calc_1=Calculus()
        self.calc_2=Calculus()

        self.real_time_vel=Calculus()
        self.real_time_acc=Calculus()

        self.syst=syst
        self.motor=motor

        if self.syst is not None and motor:
            self.torque=np.array([])

    def get_velocity(self,y,t):
        self.real_time_vel.get_real_derivative(y,t)
        self.velocity=self.real_time_vel.derivative

    def get_acceleration(self,vel,t):
        self.real_time_acc.get_real_derivative(vel,t)
        self.acceleration=self.real_time_acc.derivative

        if self.syst is not None and self.motor:
            if len(self.acceleration)>1:
                self.torque=np.append(self.torque,self.acceleration[-1]*self.syst.J_axis)
            else:
                self.torque=np.append(self.torque,0)
        
    def analyse(self,y,t,ss_value=1):
        overshoot = (np.max(y)-ss_value)/ss_value if (np.max(y)-ss_value)/ss_value>0 else 0
        check=True
        index = -1
        while check:
            if ss_value>0:
                if y[index] >= 1.02 * ss_value or y[index] <= 0.98 * ss_value:
                    settling_time = t[index]
                    check = False
            else:
                if y[index] <= 1.02 * ss_value or y[index] >= 0.98 * ss_value:
                    settling_time = t[index]
                    check = False
            index-=1
        self.overshoot=overshoot
        self.settling_time=settling_time
        self.summed_error=np.sum(np.abs(y-ss_value))
        self.avg_error=self.summed_error/len(y)
        self.ss_value=ss_value

        self.analysis={'Overshoot':self.overshoot,'Settling time':self.settling_time,'Average error':self.avg_error}

        if self.syst is not None and self.motor:
            self.max_torque=np.max(np.abs(self.torque))
            self.avg_torque=np.average(np.abs(self.torque))
            self.analysis['Max Torque']=self.max_torque


class PID(Calculus,Analysis):
    def __init__(self,K = [236.71653478, 24.59454428, 7.24508008]):
        self.Kp, self.Ki, self.Kd =K
        self.action=np.array([])
        self.error=np.array([])

        Calculus.__init__(self)
        Analysis.__init__(self)

    def link_system(self,syst):
        self.syst=syst

    def get_action(self,ref,current_value,times):
        self.error=np.append(self.error,ref[-1]-current_value[-1])
        self.get_real_derivative(self.error,times)
        self.get_real_integral(self.error,times)
        action = self.Kp*self.error[-1]+self.Ki*self.integral[-1]+self.Kd*self.derivative[-1]
        self.action=np.append(self.action,action)

class I_PD(Calculus,Analysis):
    def __init__(self,K = [21.8793059770378, 83.2760130980813, 1.27722711584499]):
        self.Kp, self.Ki, self.Kd =K
        self.action=np.array([0])
        self.error=np.array([0])

        Calculus.__init__(self)
        Analysis.__init__(self)

    def get_action(self,ref,current_value,times):
        self.error = np.append(self.error, [current_value[-1]-ref[-1]])
        self.get_derivative(current_value)
        self.get_integral(self.error)
        action = self.Kp * current_value[-1] + self.Ki * self.integral[-1] + self.Kd * self.derivative[-1]
        self.action = np.append(self.action, action)

class LQR:
    def __init__(self,syst=None ,A=None,B=None,Q=np.diag([3.71917414e+02,2.34247799e+02,6.33011862e-01,3.55617195e-02]),
                 R=np.array([1.00000000e-04])):
        self.calc_tip=Calculus()
        self.calc_motor=Calculus()
        self.Q=Q
        self.R=R
        if syst is None and A is not None and B is not None:
            self.A=A
            self.B=B
            self.K,S,E=c.lqr(self.A,self.B,self.Q,self.R)


        self.syst=syst

        self.action=np.array([0])

    def link_system(self,syst):
        self.syst=syst

    def get_action(self,ref,tip,motor,times):
        if not (self.syst is None):
            self.K,S,E=c.lqr(self.syst.A,self.syst.B,self.Q,self.R)
        self.calc_tip.get_real_derivative(tip,times)
        self.calc_motor.get_real_derivative(motor,times)
        ref=np.array([ref[-1],ref[-1],0,0])
        state=np.array([tip[-1],motor[-1],self.calc_tip.derivative[-1],self.calc_motor.derivative[-1]])
        inputs=ref-state
        inputs=np.reshape(inputs,(-1,1))
        new_action=np.dot(self.K[0],inputs)
        self.action=np.append(self.action,new_action)

class Bypass:
    def __init__(self):
        self.action=np.array([])

    def link_system(self,syst):
        self.syst=syst

    def get_action(self):
        self.action=np.append(self.action,self.syst.compensator.comp_ref[-1])

class Estimator:
    def __init__(self, function,syst=None, initial_values=[], initial_inputs=[], method='Modified Euler'):
        self.function=function
        self.syst=syst
        if not (syst is None):
            if function=='tip':
                self.coefs=syst.coefs_beam_equation
            elif function=='servo':
                self.coefs = syst.coefs_servo_equation
            else:
                self.coefs=function
        else:
            self.coefs=function

        self.ys = []
        if len(initial_values)==0:
            for i in range(len(self.coefs[0])-1):
                self.ys.append(0)
        else:
            if len(self.coefs[0])-1==len(initial_values):
                for value in initial_values:
                    self.ys.append(value)
            else:
                raise Exception('initial value dimension is equal to ys list dimension')
        if len(initial_inputs)==0:
            for i in range(1,len(self.coefs)):
                aux=[]
                for i in range(len(self.coefs[i])):
                    aux.append(0)
                initial_inputs.append(aux)
        else:

            if len(self.coefs)-1==len(initial_inputs):
                for i in range(1,len(self.coefs)):
                    if not len(self.coefs[i])==len(initial_inputs[i-1]):
                        raise Exception('initial inputs dimension is not coherent with coefs')
            else:
                raise Exception('initial inputs dimension is not coherent with coefs')

        initial_input=0
        for i in range(1,len(self.coefs)):
            initial_input+=np.dot(initial_inputs[i-1],self.coefs[i])
        new_highest_order = initial_input
        for i in range(len(self.coefs[0]) - 1):
            new_highest_order -= self.ys[i] * self.coefs[0][i]
        new_highest_order /= self.coefs[0][-1]
        self.ys.append(new_highest_order)

        if method=='Euler':
            self.estimator=euler_update

        elif method=='Modified Euler':
            self.estimator=modified_euler_update

        elif method=='Aldrabated':
            self.estimator = aldrabated

        self.stored_ys=[self.ys]

    def estimate(self,times,latest_inputs=[1]):

        if not (self.syst is None):
            if self.function=='tip':
                self.coefs=self.syst.coefs_beam_equation
            elif self.function=='servo':
                self.coefs = self.syst.coefs_servo_equation
            else:
                self.coefs=self.function

        output_coefs=self.coefs[0]
        input_coefs=self.coefs[1:]
        send_input=0

        for coef_list,input_list in zip(input_coefs,latest_inputs):
            send_input+=np.dot(coef_list,input_list)
        self.ys=self.estimator(times,self.ys,send_input,output_coefs)
        self.stored_ys.append(self.ys)

def euler_update(times, prev_y, latest_input, coefs,subit=10):
    step=(times[-1]-times[-2])/subit
    #first iteration
    new_ys = []
    new_highest_order = latest_input
    for i in range(len(prev_y) - 1):
        new = prev_y[i] + step * prev_y[i + 1]
        new_highest_order -= coefs[i] * new
        new_ys.append(new)
    new_highest_order /= coefs[-1]
    new_ys.append(new_highest_order)
    #remaining iterations
    for i in range(1,subit):
        prev_y = new_ys
        new_ys = []
        new_highest_order=latest_input
        for i in range(len(prev_y)-1):
            new=prev_y[i]+step*prev_y[i+1]
            new_highest_order-=coefs[i]*new
            new_ys.append(new)
        new_highest_order/=coefs[-1]
        new_ys.append(new_highest_order)

    return new_ys

def modified_euler_update(times, prev_y, latest_input, coefs,subit=10):
    step=(times[-1]-times[-2])/subit
    pred_new_ys = []
    pred_new_highest_order = latest_input
    for i in range(len(prev_y) - 1):
        pred_new = prev_y[i] + step * prev_y[i + 1]
        pred_new_highest_order = pred_new_highest_order-coefs[i] * pred_new
        pred_new_ys.append(pred_new)
    pred_new_highest_order /= coefs[-1]
    pred_new_ys.append(pred_new_highest_order)
    #first iteration
    new_highest_order=latest_input
    new_ys=[]
    for i in range(1,len(prev_y)):
        avg_deriv=(prev_y[i]+pred_new_ys[i])/2
        new=prev_y[i-1]+step*avg_deriv
        new_highest_order=new_highest_order-coefs[i-1]*new
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
            pred_new_highest_order = pred_new_highest_order-coefs[i] * pred_new
            pred_new_ys.append(pred_new)
        pred_new_highest_order /= coefs[-1]
        pred_new_ys.append(pred_new_highest_order)

        new_highest_order=latest_input
        new_ys=[]
        for i in range(1,len(prev_y)):
            avg_deriv=(prev_y[i]+pred_new_ys[i])/2
            new=prev_y[i-1]+step*avg_deriv
            new_highest_order=new_highest_order-coefs[i-1]*new
            new_ys.append(new)
        new_highest_order /= coefs[-1]
        new_ys.append(new_highest_order)
        new_ys=np.reshape(new_ys,(len(new_ys),))
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
    new_dy=(new_y-prev_y)/delta1
    new_d2y=(new_dy-y[1])/delta1
    new_ys=[]
    new_ys.append(new_y)
    new_ys.append(new_dy)
    new_ys.append(new_d2y)
    return new_ys

class Compensator(Calculus):
    def __init__(self, function='trace tf',optimized_coefs=[1.00759717,2*0.42202695/60.3151681,1/(60.3151681**2)]):
        self.comp_ref=np.array([])
        self.times_ref_change=np.array([])
        self.function=function
        if function == 'cosine':
            self.comp_function=cosine_comp
            self.comp_update=cosine_comp_update
            self.calc=Calculus()
        elif function == 'sigmoid':
            self.comp_function=sigmoid_comp
            self.comp_update=sigmoid_comp_update
            self.calc=Calculus()
        elif function == 'trace tf':
            self.comp_function=trace_tf_comp
            self.comp_update=trace_tf_comp_update
            self.calc=Calculus
        elif function == 'trace tf particular':
            self.comp_function=trace_tf_comp_particular
            self.comp_update=trace_tf_comp_update
            self.calc=Calculus
        else:
            self.comp_function=return_zero
            self.comp_update=return_zero

        self.optimized_coefs=optimized_coefs
        self.calc=Calculus
    
    def link_system(self,syst_obj):
        self.syst_obj=syst_obj

            
    def update(self):
        self.times_ref_change=np.append(self.times_ref_change,self.syst_obj.times[-1])
        self.parameters=self.comp_update(self.syst_obj,self.optimized_coefs,self.calc)
        
    def compensate(self):
        new_comp=self.comp_function(self.syst_obj,self.times_ref_change,self.parameters)
        self.comp_ref=np.append(self.comp_ref,self.syst_obj.ref[-1]-new_comp)
        
def cosine_comp_update(syst_obj,optimized_coefs,calculus):
    calculus.get_real_derivative(syst_obj.motor_angle,syst_obj.times)
    last_ang_vel=calculus.derivative[-1]
    if isinstance(syst_obj.controller,(PID,I_PD)):
        delta_ref=syst_obj.ref[-1]-syst_obj.ref[-2] if len(syst_obj.ref)>1 else syst_obj.ref[-1]
        arguments=[np.abs(delta_ref),(delta_ref/np.abs(delta_ref))*last_ang_vel,\
                   syst_obj.controller.Kp,syst_obj.controller.Ki,syst_obj.controller.Kd]
    k=np.dot(optimized_coefs[:5],arguments)
    t_begin=np.dot(optimized_coefs[5:],arguments)
    return [k,t_begin]

def cosine_comp(syst_obj,times_ref_change,parameters):
    k,t_begin=parameters
    t=syst_obj.times[-1]-times_ref_change[-1]
    if t<t_begin:
        new_comp=k
    else:
        new_comp=k*math.cos(2*math.pi*params.wn*(t-t_begin))        
    return new_comp


def sigmoid_comp_update(syst_obj,optimized_coefs,calculus):
    calculus.get_real_derivative(syst_obj.motor_angle,syst_obj.times)
    last_ang_vel=calculus.derivative[-1]
    if isinstance(syst_obj.controller,(PID,I_PD)):
        delta_ref=syst_obj.ref[-1]-syst_obj.ref[-2] if len(syst_obj.ref)>1 else syst_obj.ref[-1]
        arguments=[np.abs(delta_ref),(delta_ref/np.abs(delta_ref))*last_ang_vel,\
                   syst_obj.controller.Kp,syst_obj.controller.Ki,syst_obj.controller.Kd]
    elif isinstance(syst_obj.controller,LQR):
        arguments=[0,0,0,0,0]
    k1=np.dot(optimized_coefs[:5],arguments)
    k2=np.dot(optimized_coefs[5:10],arguments)
    t_offset=np.dot(optimized_coefs[10:],arguments)
    return [k1,k2,t_offset]

def sigmoid_comp(syst_obj,times_ref_change,parameters):
    k1,k2,t_offset=parameters
    t=syst_obj.times[-1]-times_ref_change[-1]
    new_comp=k1*(1-1/(1+math.e**(-k2*(t-t_offset))))   
    return new_comp

def trace_tf_comp_update(syst_obj,optimized_coefs,calc=Calculus):
    times_from_ref_change = [0]
    return [times_from_ref_change,optimized_coefs]

def trace_tf_comp(syst_obj,times_ref_change,parameters):
    t,params_tf_trace=parameters
    t.append(syst_obj.times[-1] - times_ref_change[-1])
    init_val=syst_obj.tip_estimator.ys[:len(params_tf_trace)-1]
    estimator=Estimator(function=[params_tf_trace,[1]], initial_values=init_val, initial_inputs=[[syst_obj.ref[-1]]],
                        method='Modified Euler')
    estimator.estimate(times=t, latest_inputs=[syst_obj.ref[-1]])
    ys_copy=copy.deepcopy(estimator.ys)
    if len(ys_copy)<len(syst_obj.coefs_beam_equation_output):
        for i in range(len(syst_obj.coefs_beam_equation_output)-len(ys_copy)):
            if len(syst_obj.tip_estimator.stored_ys)>1:
                ys_copy.append((syst_obj.tip_estimator.stored_ys[-1][len(ys_copy)+i]-
                                syst_obj.tip_estimator.stored_ys[-1][len(ys_copy)+i])
                               /(syst_obj.times[-1]-syst_obj.times[-2]))
            else:
                ys_copy.append(0)

    elif len(ys_copy)>len(syst_obj.coefs_beam_equation_output):
        ys_copy=ys_copy[:len(syst_obj.coefs_beam_equation_output)]
    total_input=np.dot(ys_copy,syst_obj.coefs_beam_equation_output)
    ref_to_motor=total_input-np.dot(syst_obj.coefs_beam_equation_inertia_input,[0,0,syst_obj.link_base_acceleration[-1]])
    return syst_obj.ref[-1]-ref_to_motor

def trace_tf_comp_particular(syst_obj,times_ref_change,parameters):
    t,params_tf_trace=parameters
    params_tf_trace=[params_tf_trace[2]*syst_obj.J_axis,2*params_tf_trace[0]/params_tf_trace[1],1/((params_tf_trace[1]*syst_obj.J_axis)**2)]
    t.append(syst_obj.times[-1] - times_ref_change[-1])
    init_val=syst_obj.tip_estimator.ys[:len(params_tf_trace)-1]
    print(syst_obj.ref)
    estimator=Estimator(function=[params_tf_trace,[1]], initial_values=init_val, initial_inputs=[[syst_obj.ref[-1]]],
                        method='Modified Euler')
    estimator.estimate(times=t, latest_inputs=[syst_obj.ref[-1]])
    ys_copy=copy.deepcopy(estimator.ys)
    if len(ys_copy)<len(syst_obj.coefs_beam_equation_output):
        for i in range(len(syst_obj.coefs_beam_equation_output)-len(ys_copy)):
            if len(syst_obj.tip_estimator.stored_ys)>1:
                ys_copy.append((syst_obj.tip_estimator.stored_ys[-1][len(ys_copy)+i]-
                                syst_obj.tip_estimator.stored_ys[-1][len(ys_copy)+i])
                               /(syst_obj.times[-1]-syst_obj.times[-2]))
            else:
                ys_copy.append(0)

    elif len(ys_copy)>len(syst_obj.coefs_beam_equation_output):
        ys_copy=ys_copy[:len(syst_obj.coefs_beam_equation_output)]
    total_input=np.dot(ys_copy,syst_obj.coefs_beam_equation_output)
    ref_to_motor=total_input-np.dot(syst_obj.coefs_beam_equation_inertia_input,[0,0,syst_obj.link_base_acceleration[-1]])
    return syst_obj.ref[-1]-ref_to_motor

def return_zero(*nonimportantargs):
    return 0

class Plotter:
    def __init__(self,method='Modified Euler'):
        if method=='Modified Euler':
            self.plot_fnc=modified_euler_update

    def simulate_tf(self,coefs,sim_duration,input,initial_values=[],iterations=1000,sub_iterations=3):
        ys = []
        if len(initial_values) == 0:
            for i in range(len(coefs) - 1):
                ys.append(0)
        else:
            if len(coefs) - 1 == len(initial_values):
                for value in initial_values:
                    ys.append(value)
            else:
                raise Exception('initial value dimension is equal to ys list dimension')
        new_highest_order = input
        for i in range(len(coefs) - 1):
            new_highest_order -= ys[i] * coefs[i]
        new_highest_order /= coefs[-1]
        ys.append(new_highest_order)

        time_step=sim_duration/iterations
        self.t=[0]
        self.y=[ys]

        for i in range(iterations):
            self.t.append(self.t[-1]+time_step)
            self.y.append(self.plot_fnc(self.t,self.y[-1],input,coefs,sub_iterations))

        self.y=np.transpose(self.y)

    def show_plot(self,up_to_order=0):
        up_to_order=np.clip(up_to_order,0,len(self.y)-1)
        for order in range(up_to_order+1):
            plt.plot(self.t,self.y[order,:],label='order '+str(order))
        plt.legend()
        plt.show()