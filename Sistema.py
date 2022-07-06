import keyboard
import matplotlib.pyplot as plt
import numpy as np
import Controllers as cont
import time
import Parameters as params
import math
import copy
#import RPi.GPIO as GPIO

class Link:
    def __init__(self, link_parameters,controller=None,compensator=None,limit_action=False,constant_time_interval=True,):

        self.parameters=link_parameters

        #Propriedades do bloco
        self.hb=link_parameters[0][0]
        self.tb=link_parameters[0][1]
        self.wb=link_parameters[0][2]
        self.ro_b = link_parameters[0][3]
        self.mb=self.ro_b*self.hb*self.wb*self.tb*10**(-9)
        self.centroid_b=np.array([0,0]) #Em relação ao eixo do motor deste link
        self.Jb_particular_g = self.mb*(self.wb**2+self.hb**2)*(10**(-6))/12*(10**(-6))
        self.Jb_axis = self.Jb_particular_g+self.mb*np.dot(self.centroid_b,self.centroid_b)*(10**(-6))

        # Propriedades da viga
        self.lc = link_parameters[1][0]
        self.lg=link_parameters[1][1]
        self.tc = link_parameters[1][2]
        self.wc = link_parameters[1][3]
        self.ro_c = link_parameters[1][4]
        self.mc = self.ro_c*(self.lc+self.wb)*self.wc*self.tc*10**(-9)
        self.centroid_c=np.array([self.lc/2,(self.hb+self.tc)/2]) #Em relação ao eixo do motor deste link
        self.Jc_particular_g=self.mc*(self.tc**2+(self.lc+self.wb)**2)/12*(10**(-6))
        self.Jc_axis=self.Jc_particular_g+self.mc*np.dot(self.centroid_c,self.centroid_c)*(10**(-6))

        self.lr=self.lc+self.wb/2

        #Propriedades totais
        self.m_total_link=self.mb+self.mc
        self.centroid_link=(self.centroid_c*self.mc+self.centroid_b*self.mb)/self.m_total_link
        self.Jb_link_g=self.Jb_particular_g+self.mb*np.sum((self.centroid_b-self.centroid_link)**2)*(10**(-6))
        self.Jc_link_g=self.Jc_particular_g+self.mc*np.sum((self.centroid_c-self.centroid_link)**2)*(10**(-6))
        self.J_link_g=self.Jb_link_g+self.Jc_link_g
        self.J_link_axis=self.Jb_axis+self.Jc_axis

        self.tip_coordinates=np.array([self.lr,(self.hb+self.tc)/2])

        # Propriedades do material
        self.E = link_parameters[1][5]

        # Propriedades do sistema a controlar
        self.Is = ((self.tc * 10 ** -3) ** 3) * (self.wc      * 10 ** -3) / 12  # Momento de área da secção da viga
        self.k = 3 * self.E * 10 ** 9 * self.Is / ((self.lc * 10 ** -3) ** 3)  # Rigidez N/m

        #Sub-sistemas
        self.controller = controller
        self.limit_control_action_fnc=limit_control_action if limit_action else no_control_limit
        self.compensator = compensator
        self.motor_analyser=cont.Analysis(self,motor=True)
        self.tip_analyser=cont.Analysis(self,motor=False)

        #Initial values except derivatives
        self.motor_angle=np.array([0])
        self.tip_angle=np.array([0])
        self.times=np.array([0])
        self.ref=[]
        self.controller_output=np.array([0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)
        self.motor_velocity=self.motor_analyser.velocity
        self.motor_analyser.get_acceleration(self.motor_velocity,self.times)
        self.motor_acceleration=self.motor_analyser.acceleration
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        self.tip_velocity=self.tip_analyser.velocity
        self.tip_analyser.get_acceleration(self.tip_velocity,self.times)
        self.tip_acceleration=self.tip_analyser.acceleration
        self.tip_linear_delta=self.lr*10**(-3)*self.tip_angle
        self.tip_linear_velocity=self.lr*10**(-3)*self.tip_velocity
        self.tip_linear_acceleration=self.lr*10**(-3)*self.tip_acceleration
        self.link_base_acceleration=[0]
        self.coefs_beam_equation_motor_action_input=[1]    #INPUT
        self.coefs_servo_equation_controller_action_input=[1]      #INPUT

        #AUX

        self.numit=0
        self.ints=[]
        #print('WARNING!!\nCheck value time for first action limitation in limit action function')


    def connect_links(self,previous,next):
        self.previous_link=previous
        self.next_link=next


    def set_base(self):
        self.link_base_acceleration=[0]

    def set_end_effector(self):
        self.m = params.m
        self.m_total=self.m_total_link+self.m
        self.centroid_m=np.array([self.lr,(self.hb+self.tc)/2]) #Em relação ao eixo do motor deste link]
        self.centroid=(self.centroid_m*self.m_total+self.centroid_b*self.mb+self.centroid_c*self.mc)/self.m_total
        self.Jm_axis=self.m*np.sum((self.centroid_m)**2)*(10**(-6))
        self.Jm_g=self.m*np.sum((self.centroid_m-self.centroid)**2)*(10**(-6))
        self.J_axis=self.J_link_axis+self.Jm_axis
        self.J_g=self.J_link_g+self.Jm_g

        self.wn_sq = self.k / self.m  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)


        self.T=(params.Jrotor+self.J_axis)/(params.kt*params.kb*120*math.pi/1000)

        self.coefs_beam_equation_output=[1,0,1/self.wn_sq] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_servo_equation_output=[0,1,self.T] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_beam_equation_inertia_input=[0,0,-1/(self.wn_sq*self.lr*10**(-3))]

        self.coefs_servo_equation=[self.coefs_servo_equation_output,
                                   self.coefs_servo_equation_controller_action_input]
        self.coefs_beam_equation=[self.coefs_beam_equation_output,self.coefs_beam_equation_motor_action_input,
                                  self.coefs_beam_equation_inertia_input]

        self.A=np.array([[0,0,1,0],[0,0,0,1],[-self.wn_sq,self.wn_sq,0,0]
                                     ,[0,0,0,-1/self.T]])
        self.B=np.array([[0],[0],[0],[1/self.T]])
        self.C=np.array([1,0,0,0])
        self.D=np.array([0])

    def update_geometry(self):

        self.m_total=self.m_total_link+self.next_link.m_total
        self.centroid_m=self.tip_coordinates+np.array([math.cos(self.next_link.motor_angle[-1]),
                                    math.sin(self.next_link.motor_angle[-1])])*np.linalg.norm(self.next_link.centroid)
        self.centroid=(self.centroid_m*self.next_link.m_total+
                       self.centroid_link*self.m_total_link)/self.m_total
        self.Jm_axis=self.next_link.J_g+self.next_link.m_total*np.sum(self.centroid_m**2)*(10**(-6))
        self.Jm_g=self.next_link.J_g+self.next_link.m_total* \
                          np.sum((self.centroid_m-self.centroid)**2)*(10**(-6))


        self.J_g=self.Jm_g+self.J_link_axis+ \
                 self.m_total_link*np.sum(self.centroid**2)*(10**(-6))

        self.J_axis=self.Jm_axis+self.J_link_axis

        self.wn_sq = self.k / self.m_total  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)
        self.T=(params.Jrotor+self.J_axis)/(params.kt*params.kb*120*math.pi/1000)

        self.coefs_beam_equation_output=[1,0,1/self.wn_sq] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_servo_equation_output=[0,1,self.T] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_beam_equation_inertia_input=[0,0,-1/(self.wn_sq*self.lr*10**(-3))]
        self.coefs_servo_equation=[self.coefs_servo_equation_output,
                                   self.coefs_servo_equation_controller_action_input]
        self.coefs_beam_equation=[self.coefs_beam_equation_output,self.coefs_beam_equation_motor_action_input,
                                  self.coefs_beam_equation_inertia_input]

        #State space
        self.A=np.array([[0,0,1,0],[0,0,0,1],[-self.wn_sq,self.wn_sq,0,0]
                                     ,[0,0,0,-1/self.T]])
        self.B=np.array([[0],[0],[0],[1/self.T]])
        self.C=np.array([1,0,0,0])
        self.D=np.array([0])

    def compensate(self):
        self.compensator.compensate()

    def control(self):
        if isinstance(self.controller,cont.PID):
            self.controller.get_action(self.compensator.comp_ref,self.motor_angle,self.times)
        elif isinstance(self.controller,cont.LQR):
            self.controller.get_action(self.compensator.comp_ref,self.tip_angle,self.motor_angle,self.times)
        elif isinstance(self.controller,cont.Bypass):
            self.controller.get_action()
        self.controller_output=np.append(self.controller_output,self.limit_control_action_fnc(self))

    def update_motor_angle(self):
        pass

    def estimate_motor_angle(self):
        self.motor_estimator.estimate(times=self.times[-2:],latest_inputs=[[self.controller_output[-1]]])
        self.motor_angle=np.append(self.motor_angle,self.motor_estimator.ys[0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)
        self.motor_velocity=self.motor_analyser.velocity
        self.motor_analyser.get_acceleration(self.motor_velocity,self.times)
        self.motor_acceleration=self.motor_analyser.acceleration

    def update_tip_angle(self):
        pass

    def estimate_tip_angle(self):
        if self.previous_link is not None:
            self.link_base_acceleration.append(self.previous_link.tip_linear_acceleration[-1]*math.cos(self.motor_angle[-1])
                                               -math.sin(self.motor_angle[-1])*self.previous_link.tip_velocity[-1]**2*self.previous_link.lr*10**(-3))
        self.tip_estimator.estimate(times=self.times[-2:],
                                    latest_inputs=[[self.motor_angle[-1]],[0,0,self.link_base_acceleration[-1]]])
        self.tip_angle=np.append(self.tip_angle,self.tip_estimator.ys[0])
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        self.tip_velocity=self.tip_analyser.velocity
        self.tip_analyser.get_acceleration(self.tip_velocity,self.times)
        self.tip_acceleration=self.tip_analyser.acceleration
        self.tip_linear_delta=self.lr*10**(-3)*self.tip_angle
        self.tip_linear_velocity=self.lr*10**(-3)*self.tip_velocity
        self.tip_linear_acceleration=self.lr*10**(-3)*self.tip_acceleration

    def get_analysis(self,ss_value=1):
        self.motor_analyser.analyse(self.motor_angle,self.times,ss_value)
        self.tip_analyser.analyse(self.tip_angle,self.times,ss_value)
        self.analysis={'Overshoot':self.tip_analyser.overshoot,'Settling time':self.tip_analyser.settling_time,
                       'Average error':self.tip_analyser.avg_error,'Max torque':self.motor_analyser.max_torque}

class Robot:
    def __init__(self,const_t_int=True,num_samples=100):
        self.links=[]
        self.pre_links=[]
        self.ref=[]
        self.angular_refs=[]
        self.times=[0]
        self.time_interval=[0.02]
        self.const_t_int=const_t_int
        self.num_samples=num_samples


    def add_link(self,link_controller,link_compensator,link_parameters):
        self.pre_links.append([link_controller,link_compensator,link_parameters])

    def compile(self,details_inv_kin=None):
        print('Compiling system...')
        self.set_inverse_kinematics(details=details_inv_kin)
        for i in range(len(self.pre_links)):
            self.links.append(Link(controller=self.pre_links[i][0],compensator=self.pre_links[i][1],
                                   link_parameters=self.pre_links[i][-1]))

        for i in range(len(self.links)):

            if i==0 and len(self.links)!=1:
                self.links[i].connect_links(previous=None,next=self.links[i+1])
            elif i==len(self.links)-1 and len(self.links)!=1:
                self.links[i].connect_links(previous=self.links[i-1],next=None)
            elif len(self.links)==1:
                self.links[i].connect_links(previous=None,next=None)
            else:
                self.links[i].connect_links(previous=self.links[i-1],next=self.links[i+1])

        self.links[-1].set_end_effector()
        self.links[0].set_base()

        self.update_geometry()

        initial_x=0

        for i in range(len(self.links)):
            initial_x+=self.links[i].lg
            self.links[i].times=self.times
            self.links[i].controller.link_system(self.links[i])
            self.links[i].compensator.link_system(self.links[i])
            self.links[i].tip_estimator=cont.Estimator(function='tip',syst=self.links[i],initial_values=[],
                                                       initial_inputs=[],method='Modified Euler')
            self.links[i].motor_estimator=cont.Estimator(function='servo',syst=self.links[i],initial_values=[],
                                                         initial_inputs=[],method='Modified Euler')
        self.position=[[initial_x,0]]

        self.set_inverse_kinematics(details=details_inv_kin)

        if type(self.const_t_int)==bool:
            if self.const_t_int:
                self.times_updater=self.update_times_constant_interval
                self.get_initial_time_gap(num_samples=self.num_samples)
            else:
                self.times_updater=self.update_real_times

        else:
            self.times_updater=self.update_times_constant_interval
            self.time_interval=[self.const_t_int]

    def set_inverse_kinematics(self,details=None):
        if len(self.links)==2:
            self.get_angles=two_link_inv_kin
        if len(self.links)==1:
            self.get_angles=one_link_kinematics

    def start(self,input_ref=True):
        if input_ref:
            print('Reference Input')
            new_ref_x=float(input('Input Initial Target in x (mm): '))
            new_ref_y=float(input('Input Initial Target in y (mm): '))
            new_ref=[new_ref_x,new_ref_y]
            new_angles=self.get_angles(new_ref,robot=self)
            self.angular_refs.append(new_angles)
        else:
            if len(self.links)==2:
                new_ref=[100,100]
            elif len(self.links)==1:
                new_ref=[1]
            new_angles=self.get_angles(new_ref,robot=self)
            self.angular_refs.append(new_angles)
        self.ref=[new_ref]

        print('Running...')
        self.start_time=time.time()

        for i in range(len(self.links)):
            self.links[i].start_time=self.start_time

            self.links[i].ref.append(self.angular_refs[-1][i])
            self.links[i].compensator.update()

    def update_geometry(self):
        for i in range(1,len(self.links)):
            self.links[-i-1].update_geometry()

    def update_ref(self,bypass=True):
        if bypass:
            self.ref.append(self.ref[-1])
            self.angular_refs.append(self.angular_refs[-1])
            for i in range(len(self.links)):
                self.links[i].ref.append(self.angular_refs[-1][i])
        else:

            if keyboard.is_pressed('ctrl+n'):
                new_ref_x=float(input('Input Initial Target in x (mm): '))
                new_ref_y=float(input('Input Initial Target in y (mm): '))
                new_ref=[new_ref_x,new_ref_y]
                self.ref.append(new_ref)
                self.angular_refs.append(self.get_angles(self.ref[-1]))
                for i in range(len(self.links)):
                    self.links[i].ref.append(self.angular_refs[-1][i])
                    self.links[i].compensator.update()
            else:
                self.ref.append(self.ref[-1])
                self.angular_refs.append(self.angular_refs[-1])
                for i in range(len(self.links)):
                    self.links[i].ref.append(self.angular_refs[-1][i])

    def control(self):
        for i in range(len(self.links)):
            self.links[i].compensate()
            self.links[i].control()

    def update_real_times(self):

        self.times.append(time.time()-self.start_time)

    def update_times_constant_interval(self):
        self.times=np.append(self.times,self.times[-1]+self.time_interval[-1])


    def update_times(self):
        self.times_updater()
        for i in range(len(self.links)):
            self.links[i].times=self.times

    def get_initial_time_gap(self,num_samples=100):
        print('Calculating average time with', num_samples, 'samples...')
        robot_test=Robot(const_t_int=0.1)
        for i in range(len(self.links)):
            cont_test=type(self.links[i].controller)()
            comp_test=type(self.links[i].compensator)(function=self.links[i].compensator.function,
                                                      optimized_coefs=self.links[i].compensator.optimized_coefs)
            robot_test.add_link(link_controller=cont_test,link_compensator=comp_test,
                                link_parameters=self.links[i].parameters)
        robot_test.compile()
        robot_test.start(input_ref=False)
        while robot_test.times[-1]<0.5:
            robot_test.control()
            robot_test.update_times()
            robot_test.estimate_positions()
            robot_test.update_ref()
            robot_test.update_geometry()
        current=time.time()
        self.time_interval=np.array([(current-robot_test.start_time)/num_samples])


    def estimate_positions(self):
        total_angle=0
        x=0
        y=0
        for i in range(len(self.links)):
            self.links[i].estimate_motor_angle()
            self.links[i].estimate_tip_angle()
            total_angle+=self.links[i].tip_angle[-1]
            x+=self.links[-i].lg*math.cos(total_angle)
            y+=self.links[-i].lg*math.sin(total_angle)

        self.position.append([x,y])

    def analyse(self,show_performance=True,plot_coordinates=True,plot_angles=True):
        self.analysis=[]
        for i in range(len(self.links)):
            self.links[i].get_analysis(ss_value=self.links[i].ref[-1])
            self.analysis.append(self.links[i].analysis)

        if show_performance:
            for i in range(len(self.analysis)):
                print('Performance of link',i+1)
                if i == len(self.links)-1:
                    print(self.analysis[i])
                else:
                    print(self.analysis[i],'\n')

        if plot_coordinates and plot_angles:

            fig, axs = plt.subplots(len(self.links)+1,2,constrained_layout=True)
            position=np.array(self.position)
            position=np.transpose(position)
            axs[0,0].plot(self.times, position[0])
            axs[0,0].set_title('X Position')
            axs[0,1].plot(self.times, position[1])
            axs[0,1].set_title('Y Position')
            for i in range(len(self.links)):
                axs[i+1,0].plot(self.times,self.links[i].tip_angle,label='Link '+ str(i+1)+ ' Tip angle')
                axs[i+1,0].set_title('Link ' + str(i+1) + ' Tip angle')

                axs[i+1,1].plot(self.times,self.links[i].motor_angle,label='Link '+ str(i+1)+ ' Motor angle')
                axs[i+1,1].set_title('Link ' + str(i+1) + ' Motor angle')


        elif plot_angles:

            fig, axs = plt.subplots(len(self.links),2,constrained_layout=True)
            for i in range(len(self.links)):
                axs[i,0].plot(self.times,self.links[i].tip_angle,label='Link '+ str(i+1)+ ' Tip angle')
                axs[i,0].set_title('Link ' + str(i+1) + ' Tip angle')
                axs[i,1].plot(self.times,self.links[i].motor_angle,label='Link '+ str(i+1)+ ' Motor angle')
                axs[i,1].set_title('Link ' + str(i+1) + ' Motor angle')

        elif plot_coordinates:
            fig, axs = plt.subplots(2,constrained_layout=True)
            position=np.array(self.position)
            position=np.transpose(position)
            axs[0].plot(self.times, position[0])
            axs[0].set_title('X Position')
            axs[1].plot(self.times, position[1])
            axs[1].set_title('Y Position')

        if plot_coordinates or plot_angles:

            plt.show()

def one_link_kinematics(angle,robot):
    return angle

def two_link_inv_kin(cartesian_coords,robot):
    x,y=cartesian_coords
    d=np.sqrt(x**2+y**2)
    l1=robot.links[0].lg
    l2=robot.links[1].lg
    alpha_abs=math.acos((d**2+l1**2-l2**2)/(2*d*l1))
    alpha_list=[-alpha_abs,alpha_abs]
    theta1_list=[]
    theta1_dif_list=[]
    prev_theta1=robot.links[0].motor_angle[-1]
    for alpha in alpha_list:
        sum_t1_a=math.acos(x/d)
        if d*math.sin(sum_t1_a)==-y:
            sum_t1_a=-sum_t1_a
        new_theta=sum_t1_a-alpha
        theta1_list.append(new_theta)
        theta1_dif_list.append(np.abs(new_theta-prev_theta1))
    theta1=theta1_list[np.argmin(theta1_list)]
    sum_t1_t2=math.acos((x-l1*math.cos(theta1))/l2)
    if l1*math.sin(theta1)+l2*math.sin(sum_t1_t2)==-y:
        sum_t1_t2=-sum_t1_t2
    theta2=sum_t1_t2-theta1

    return [theta1,theta2]


def limit_control_action_old(action_y,pos,t,limit_values,time_interval,limit_bool):
    value=action_y[-1]
    if limit_bool:

        if np.abs(value)>limit_values[0]:
            value=limit_values[0]*np.sign(value)
        if len(pos)==1:

            if np.abs(value/time_interval)*params.J>limit_values[1]:
                value=(limit_values[1]/params.J)*time_interval*np.sign(value)
        else:
            prev_velocity=(pos[-1]-pos[-2])/(t[-1]-t[-2])
            if np.abs((value-prev_velocity)/(t[-1]-t[-2]))*params.J>limit_values[1]:
                value=limit_values[1]*(t[-1]-t[-2])*((value-prev_velocity)/np.abs(value-prev_velocity))\
                      /params.J+prev_velocity
    return value

def limit_control_action(syst_obj):
    value=syst_obj.controller.action[-1]
    if np.abs(value)>params.limit_values[0]:
        value=params.limit_values[0]*np.sign(value)
    if len(syst_obj.motor_angle)==1:
        try:
            if np.abs(value/syst_obj.time_interval[-1])*params.J>params.limit_values[1]:
                value=(params.limit_values[1]/params.J)*syst_obj.time_interval[-1]*np.sign(value)
        except:
            pass
    else:
        prev_velocity=syst_obj.motor_velocity[-1]
        if np.abs((value-prev_velocity)/(syst_obj.times[-1]-syst_obj.times[-2]))*params.J>params.limit_values[1]:
            value=params.limit_values[1]*(syst_obj.times[-1]-syst_obj.times[-2])\
                  *np.sign(value-prev_velocity)/params.J+prev_velocity
    return value



def no_control_limit(syst_obj):
    return syst_obj.controller.action[-1]

class Barry:  # Classe para cenas de eletronica tipo receber dados do encoder e outros sensores
    def __init__(self, ppr, angular_velocity, duty_cycle=50, pins=[37,35,31,29,18,16,15,13,11], mode=0):
        self.ppr = ppr  # Pulses per rotation
        self.angular_velocity = angular_velocity  # In rot per sec
        max_pps = 250 * 10 ** 3
        self.pps = self.angular_velocity * self.ppr
        #print('freq = ',self.pps)
        #print('period = ',1/self.pps)
        if self.pps > max_pps:
            max_velocity= max_pps/self.ppr
            raise Exception(f"Pulses per second are set too high (limit = {max_velocity} rot/s or = 250 kpps)")

        self.duty_cycle = duty_cycle
        if mode == 0:
            self.send_pulses = mode_0
            self.high_level_motion_function=self.send_position_increment
        elif mode == 1:
            self.send_pulses = mode_1
            self.high_level_motion_function=self.send_position_increment
        elif mode=='PWM':
            self.send_pulses = mode_pwm
            self.high_level_motion_function=self.send_angular_velocity
        else:
            raise Exception("Its either Pulse Counter or PWM Counter. Mode can't be whatever you feel like.")
        '''
        # Define pins
        self.pc1 = params.pc1
        self.pc1_=params.pc1_
        self.pc2=params.pc2
        self.pc2_=params.pc2_
        self.dev_count_reset=params.dev_count_reset
        self.dev_count_reset_=params.dev_count_reset_
        self.positioning_completed=params.positioning_completed
        self.phase_z=params.phase_z
        self.alarm = params.alarm
        self.encoder_A=params.encoder_A
        self.encoder_B=params.encoder_B

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pc1, GPIO.OUT)
        GPIO.setup(self.pc1_, GPIO.OUT)
        GPIO.setup(self.pc2, GPIO.OUT)
        GPIO.setup(self.pc2_, GPIO.OUT)
        GPIO.setup(self.dev_count_reset, GPIO.OUT)
        GPIO.setup(self.dev_count_reset_, GPIO.OUT)
        GPIO.setup(self.positioning_completed, GPIO.IN)
        GPIO.setup(self.phase_z, GPIO.IN)
        GPIO.setup(self.alarm, GPIO.OUT)

        self.pwm_pin=GPIO.PWM(params.pwm_pin, 10000)
        '''
        #Encoder txt
        f=open(params.file_path,'w')
        f.write('')
        f.close()
        
        #print('setup done')
    def send_position_increment(self, increment):
        no_pulses = self.ppr * increment / 360
        self.send_pulses(no_pulses, self.pps, [self.pc1, self.pc1_, self.pc2, self.pc2_], duty_cycle=self.duty_cycle)
        
    def send_angular_velocity(self,angular_velocity):
        pulse_freq=self.ppr*angular_velocity
        self.send_pulses(freq=pulse_freq,pins=[self.pc1, self.pc1_, self.pc2, self.pc2_],pwm_pin=self.pwm_pin, duty_cycle=self.duty_cycle)
        
    def send_motion(self,pos_or_vel):
        self.high_level_motion_function(pos_or_vel)
                
    def send_motion_FAKE(self, increment):
        time.sleep(0.001)
        
def time_wait(duration):
    timer = time.time()
    while timer + duration > time.time():
        pass

def mode_0(no_pulses, freq, pins, duty_cycle=50):
    period = 1 / freq
    active_time = period * duty_cycle / 100
    pulse_counter = 0
    puls, puls_, sign, sign_ = pins
    if no_pulses > 0:
        print('sending')
        GPIO.output(sign, 1)
        GPIO.output(sign_, 0)
    elif no_pulses<0:
        no_pulses = -no_pulses
        GPIO.output(sign, 0)
        GPIO.output(sign_, 1)
        
    while pulse_counter < no_pulses:

        GPIO.output(puls,1)
        GPIO.output(puls_, 0)
        time_wait(active_time)
        GPIO.output(puls, 1)
        GPIO.output(puls_, 1)
        time_wait(period-active_time)
        #pulse_counter+=1

def mode_1(no_pulses, freq, pins, duty_cycle=50):
    period = 1 / freq
    active_time = period * duty_cycle / 100
    pulse_counter = 0
    cw, cw_, ccw, ccw_ = pins
    if no_pulses > 0:
        GPIO.output(cw, 0)
        GPIO.output(cw_, 1)
        while pulse_counter < no_pulses:
            GPIO.output(ccw, 1)
            GPIO.output(ccw_, 0)
            timer = time.time()
            while timer + active_time > time.time():
                pass
            GPIO.output(ccw, 0)
            GPIO.output(ccw_, 1)
            timer = time.time()
            while timer + period-active_time > time.time():
                pass
            pulse_counter += 1
    if no_pulses < 0:
        no_pulses = -no_pulses
        GPIO.output(ccw, 0)
        GPIO.output(ccw_, 1)
        while pulse_counter < no_pulses:
            GPIO.output(cw, 1)
            GPIO.output(cw_, 0)
            time_wait(active_time)
            GPIO.output(cw, 0)
            GPIO.output(cw_, 1)
            time_wait(period-active_time)
            pulse_counter += 1
            
def mode_pwm(freq, pins,pwm_pin, duty_cycle=50):
    puls, puls_, sign, sign_ = pins
    if freq > 0:
        GPIO.output(sign, 1)
        GPIO.output(sign_, 0)        
    if freq < 0:
        GPIO.output(sign, 0)
        GPIO.output(sign_, 1)
    GPIO.output(puls, 1)
    pwm_pin.stop()
    pwm_pin.ChangeFrequency(freq)
    pwm_pin.start(duty_cycle)