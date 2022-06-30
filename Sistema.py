import keyboard
import numpy as np
import Controllers as cont
import time
import Parameters as params
import math
import copy
#import RPi.GPIO as GPIO

class System:
    def __init__(self, controller,compensator='trace tf',limit_action=False,constant_time_interval=True,link_parameters=[]):
        # Propriedades do material
        self.E = params.E  # Módulo de Young              INPUT GPa
        # Dimensões da viga
        self.lc = params.lc  # Comprimento da viga       INPUT mm
        self.tc = params.tc  # Espessura da viga           INPUT mm
        self.wc = params.wc  # Largura da viga            INPUT mm
        self.Is = ((self.tc * 10 ** -3) ** 3) * (self.wc      * 10 ** -3) / 12  # Momento de área da secção da viga
        # Propriedades do sistema a controlar
        self.k = 3 * self.E * 10 ** 9 * self.Is / ((self.lc * 10 ** -3) ** 3)  # Rigidez N/m
        self.m = params.m  # Massa                        INPUT Kg
        self.wn_sq = self.k / self.m  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)
        #Sub-sistemas
        self.controller = controller
        self.limit_control_action_fnc=limit_control_action if limit_action else no_control_limit
        self.tip_estimator=cont.Estimator(function='tip',method='Modified Euler')
        self.motor_estimator=cont.Estimator(function='servo',method='Modified Euler')
        self.compensator = cont.Compensator(compensator)
        self.motor_analyser=cont.Analysis()
        self.tip_analyser=cont.Analysis()
        self.times_updater=self.update_times_constant_interval if constant_time_interval else self.update_real_times
        #Initial values
        self.motor_angle=np.array([0])
        self.tip_angle=np.array([0])
        self.times=np.array([0])
        self.controller_output=np.array([0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)
        self.motor_velocity=self.motor_analyser.velocity
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        self.tip_velocity=self.tip_analyser.velocity
        #AUX
        #self.time_interval=np.array([0.0001])
        self.numit=0
        self.ints=[]
        #print('WARNING!!\nCheck value time for first action limitation in limit action function')

    def setup_compensator(self,optimized_params=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
        self.compensator.setup(self,optimized_params)
        
    def start(self,input_ref=False):
        if input_ref:
            new_ref=float(input('Input Initial Target: '))
        else:
            new_ref=1
        self.ref=np.array([new_ref])
        self.start_time=time.time()
        self.compensator.update()
        
    ### Control Loop Begins Here ###    
    def update_real_times(self,useless_arg):
        new_time=time.time()-self.start_time
        if new_time-self.times[-1]==0:
            #current=time.time()
            #time.sleep(0.0001)
            #self.ints.append(time.time()-current)
            pass
        self.times=np.append(self.times,time.time()-self.start_time)

    def update_times_constant_interval(self,num_samples=100):
        self.times=np.append(self.times,self.times[-1]+self.time_interval[-1])
        '''
        self.numit+=1
        
        if self.numit==num_samples:
            self.numit=0
            current_time=time.time()
            self.time_interval=np.append(self.time_interval,current_time-self.start_time)
            self.start_time=current_time
        '''

    def update_times(self,num_samples=100):
        self.times_updater(num_samples)

    def get_initial_time_gap_old(self,test_syst,num_samples=10000):
        controller = cont.PID(K=[206.71653478, 24.59454428, 7.24508008])
        syst_test=test_syst(controller)
        #print(syst_test)
        syst_test.setup_compensator(syst_test, optimized_params=[1,2*0.6/20,1/(20**2)])
        syst_test.start()
        for i in range(num_samples):
            syst_test.compensate_ref()
            syst_test.control()
            syst_test.update_times_constant_interval()
            syst_test.estimate_motor_angle()
            syst_test.estimate_tip_angle()
            syst_test.update_ref()
        current=time.time()
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])

    def get_initial_time_gap_LQR(self,test_syst,num_samples=10000):
        controller = cont.LQR()
        syst_test=test_syst(controller,compensator='ignore')
        #print(syst_test)
        #syst_test.setup_compensator(syst_test, optimized_params=[1,2*0.6/20,1/(20**2)])
        syst_test.setup_compensator(syst_test, optimized_params=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        syst_test.start()
        for i in range(num_samples):
            syst_test.compensate_ref()
            syst_test.control()
            syst_test.update_times_constant_interval()
            syst_test.estimate_motor_angle()
            syst_test.estimate_tip_angle()
            syst_test.update_ref()
        current=time.time()
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])

    def get_initial_time_gap(self,syst,num_samples=10000):  #NOT YET READY BUT SHOULD BE VERY USEFUL
        controller_test=type(syst.controller)()
        syst_test=type(syst)(controller_test,syst.compensator.name)
        syst_test.setup_compensator(optimized_params=syst.compensator.optimized_coefs)
        syst_test.time_interval=[0.0001]
        syst_test.start()
        for i in range(num_samples):
            syst_test.compensate_ref()
            syst_test.control()
            syst_test.update_times()
            syst_test.estimate_motor_angle()
            syst_test.estimate_tip_angle()
            syst_test.update_ref()
        current=time.time()
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])

    def compensate_ref(self):
        self.compensator.compensate()
        
    def control(self):
        if isinstance(self.controller,cont.PID):
            self.controller.get_action(self.compensator.comp_ref,self.motor_angle,self.times)
        if isinstance(self.controller,cont.LQR):
            self.controller.get_action(self.compensator.comp_ref,self.tip_angle,self.motor_angle,self.times)  #created for lqr
        self.controller_output=np.append(self.controller_output,self.limit_control_action_fnc(self))

    def update_motor_angle(self):
        pass

    def estimate_motor_angle(self):
        self.motor_estimator.estimate(times=self.times[-2:],latest_input=self.controller_output[-1])
        self.motor_angle=np.append(self.motor_angle,self.motor_estimator.ys[0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)

    def update_tip_angle(self):
        pass

    def estimate_tip_angle(self):
        self.tip_estimator.estimate(times=self.times[-2:], latest_input=self.motor_angle[-1])
        self.tip_angle = np.append(self.tip_angle,self.tip_estimator.ys[0])
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        
    def update_ref(self):
        '''
        if keyboard.is_pressed('ctrl+n'):
            new_ref=float(input('Input new ref'))
            self.ref=np.append(self.ref,new_ref)
        else:
            new_ref=self.ref[-1]
            self.ref=np.append(self.ref,new_ref)
            self.compensator.update()
        '''
    #def update_ref_prev(self):
        new_ref=self.ref[-1]
        self.ref=np.append(self.ref,new_ref)

    def get_analysis(self,ss_value=1):
        self.motor_analyser.analyse(self.motor_angle,self.times,ss_value)
        self.tip_analyser.analyse(self.tip_angle,self.times,ss_value)

class Link:
    def __init__(self, link_parameters,controller=None,compensator='trace tf',limit_action=False,constant_time_interval=True,
                 isolated=True):

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




        '''
        self.m_centroid=[]
        self.m = params.m  # Massa                        INPUT Kg
        self.wn_sq = self.k / self.m  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)
        '''


        #Sub-sistemas
        self.controller = controller
        self.limit_control_action_fnc=limit_control_action if limit_action else no_control_limit

        self.compensator = cont.Compensator(compensator)
        self.motor_analyser=cont.Analysis()
        self.tip_analyser=cont.Analysis()
        self.times_updater=self.update_times_constant_interval if constant_time_interval else self.update_real_times
        #Initial values
        self.motor_angle=np.array([0])
        self.tip_angle=np.array([0])
        self.times=np.array([0])
        self.controller_output=np.array([0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)
        self.motor_velocity=self.motor_analyser.velocity
        self.motor_analyser.get_acceleration(self.motor_velocity,self.times)
        self.motor_acceleration=self.motor_analyser.acceleration
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        self.tip_velocity=self.tip_analyser.velocity
        self.tip_analyser.get_acceleration(self.tip_velocity,self.times)
        self.tip_acceleration=self.tip_analyser.acceleration
        self.tip_linear_delta=self.lr*self.tip_angle
        self.tip_linear_velocity=self.lr*self.tip_velocity
        self.tip_linear_acceleration=self.lr*self.tip_acceleration

        self.coefs_beam_equation_motor_action_input=[1]    #INPUT
        self.coefs_servo_equation_controller_action_input=[1]      #INPUT

        #AUX
        #self.time_interval=np.array([0.0001])
        self.numit=0
        self.ints=[]
        #print('WARNING!!\nCheck value time for first action limitation in limit action function')
        if isolated:
            self.set_base()
            self.set_end_effector()
            self.controller=controller
            self.controller.link_system(self)
            self.coefs_servo_equation=[self.coefs_servo_equation_output,
                                       self.coefs_servo_equation_controller_action_input]
            self.coefs_beam_equation=[self.coefs_beam_equation_outout,self.coefs_beam_equation_motor_action_input,
                                      self.coefs_beam_equation_inertia_input]
            self.tip_estimator=cont.Estimator(function='tip',syst=self,method='Modified Euler')
            self.motor_estimator=cont.Estimator(function='servo',syst=self,method='Modified Euler')

    def connect_links(self,previous,next):
        self.previous_link=previous
        self.next_link=next
        if previous is not None:
            self.link_base_acceleration=self.previous_link.tip_linear_acceleration


    def setup_compensator(self,optimized_params=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
        self.compensator.setup(self,optimized_params)

    def set_base(self):
        self.link_base_acceleration=[0]

    def set_end_effector(self):
        self.m = params.m
        self.m_total=self.m_total_link+self.m
        self.centroid_m=np.array([self.lc/2,(self.hb+self.tc)/2]) #Em relação ao eixo do motor deste link]
        self.centroid=(self.centroid_m*self.m_total+self.centroid_b*self.mb+self.centroid_c*self.mc)/self.m_total
        self.Jm_axis=self.m*np.sum((self.centroid_m)**2)*(10**(-6))
        self.Jm_g=self.m*np.sum((self.centroid_m-self.centroid)**2)*(10**(-6))
        self.J_axis+=self.Jm_axis
        self.J_g+=self.Jm_g

        self.wn_sq = self.k / self.m  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)

        self.T=(params.Jrotor+self.J_axis)/(params.kt*params.kb*120*math.pi/1000)

        self.coefs_beam_equation_outout=[1,0,1/self.wn_sq] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_servo_equation_output=[0,1,self.T] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
        self.coefs_beam_equation_inertia_input=[0,0,-1/(self.wn_sq*self.lr)]

        self.A=np.array([[0,0,1,0],[0,0,0,1],[-self.wn_sq,self.wn_sq,0,0]
                                     ,[0,0,0,-1/self.T]])
        self.B=np.array([[0],[0],[0],[1/self.T]])
        self.C=np.array([1,0,0,0])
        self.D=np.array([0])


    def start(self,input_ref=False):
        if input_ref:
            new_ref=float(input('Input Initial Target: '))
        else:
            new_ref=1
        self.ref=np.array([new_ref])
        self.start_time=time.time()
        self.compensator.update()

    def udpate_geometry(self):
        self.m_total=self.m_total_link+self.next_link.m_total
        self.centroid_m=self.tip_coordinates+[math.cos(self.next_link.motor_angle[-1]),
                                                              math.sin(self.next_link.motor_angle[-1])]*np.linalg.norm(self.next_link.centroid)
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
        self.coefs_beam_equation_inertia_input=[0,0,-1/(self.wn_sq*self.lr)]
        self.coefs_servo_equation=[self.coefs_servo_equation_output,
                                   self.coefs_servo_equation_controller_action_input]
        self.coefs_beam_equation=[self.coefs_beam_equation_outout,self.coefs_beam_equation_motor_action_input,
                                  self.coefs_beam_equation_inertia_input]

        self.A=np.array([[0,0,1,0],[0,0,0,1],[-self.wn_sq,self.wn_sq,0,0]
                                     ,[0,0,0,-1/self.T]])
        self.B=np.array([[0],[0],[0],[1/self.T]])
        self.C=np.array([1,0,0,0])
        self.D=np.array([0])



    ### Control Loop Begins Here ###
    def update_real_times(self,useless_arg):
        new_time=time.time()-self.start_time
        if new_time-self.times[-1]==0:
            #current=time.time()
            #time.sleep(0.0001)
            #self.ints.append(time.time()-current)
            pass
        self.times=np.append(self.times,time.time()-self.start_time)

    def update_times_constant_interval(self,num_samples=100):
        self.times=np.append(self.times,self.times[-1]+self.time_interval[-1])
        '''
        self.numit+=1
        
        if self.numit==num_samples:
            self.numit=0
            current_time=time.time()
            self.time_interval=np.append(self.time_interval,current_time-self.start_time)
            self.start_time=current_time
        '''

    def update_times(self,num_samples=100):
        self.times_updater(num_samples)

    def get_initial_time_gap(self,syst,num_samples=10000):  #NOT YET READY BUT SHOULD BE VERY USEFUL
        controller_test=type(syst.controller)()
        syst_test=type(syst)(controller_test,syst.compensator.name)
        syst_test.setup_compensator(optimized_params=syst.compensator.optimized_coefs)
        syst_test.time_interval=[0.0001]
        syst_test.start()
        for i in range(num_samples):
            syst_test.compensate_ref()
            syst_test.control()
            syst_test.update_times()
            syst_test.estimate_motor_angle()
            syst_test.estimate_tip_angle()
            syst_test.update_ref()
        current=time.time()
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])

    def compensate_ref(self):
        self.compensator.compensate()

    def control(self):
        if isinstance(self.controller,cont.PID):
            self.controller.get_action(self.compensator.comp_ref,self.motor_angle,self.times)
        if isinstance(self.controller,cont.LQR):
            self.controller.get_action(self.compensator.comp_ref,self.tip_angle,self.motor_angle,self.times)  #created for lqr
        self.controller_output=np.append(self.controller_output,self.limit_control_action_fnc(self))

    def update_motor_angle(self):
        pass

    def estimate_motor_angle(self):
        self.motor_estimator.estimate(times=self.times[-2:],latest_inputs=[[self.controller_output[-1]]])
        self.motor_angle=np.append(self.motor_angle,self.motor_estimator.ys[0])
        self.motor_analyser.get_velocity(self.motor_angle,self.times)
        self.motor_analyser.get_acceleration(self.motor_velocity,self.times)

    def update_tip_angle(self):
        pass

    def estimate_tip_angle(self):
        self.tip_estimator.estimate(times=self.times[-2:],
                                    latest_inputs=[[self.motor_angle[-1]],[0,0,self.link_base_acceleration[-1]]])
        self.tip_angle = np.append(self.tip_angle,self.tip_estimator.ys[0])
        self.tip_analyser.get_velocity(self.tip_angle,self.times)
        self.tip_analyser.get_acceleration(self.tip_velocity,self.times)
        self.tip_linear_delta=self.lr*self.tip_angle
        self.tip_linear_velocity=self.lr*self.tip_velocity
        self.tip_linear_acceleration=self.lr*self.tip_acceleration

    def update_ref(self):
        '''
        if keyboard.is_pressed('ctrl+n'):
            new_ref=float(input('Input new ref'))
            self.ref=np.append(self.ref,new_ref)
        else:
            new_ref=self.ref[-1]
            self.ref=np.append(self.ref,new_ref)
            self.compensator.update()
        '''
        #def update_ref_prev(self):
        new_ref=self.ref[-1]
        self.ref=np.append(self.ref,new_ref)

    def get_analysis(self,ss_value=1):
        self.motor_analyser.analyse(self.motor_angle,self.times,ss_value)
        self.tip_analyser.analyse(self.tip_angle,self.times,ss_value)

class Robot:
    def __init__(self):
        self.links=[]
        self.pre_links=[]
        self.ref=[]
        self.angular_positions=[]



    def add_link(self,link_controller,link_parameters):
        #current_len=len(self.links)
        #try:
        #    self.links.append(Link(controller=link_controller,link_parameters=link_parameters,
        #                             previous_link=self.links[current_len-1]))
        #except:
        #    self.links.append(Link(controller=link_controller,link_parameters=link_parameters,previous_link=None))
        self.pre_links.append([link_controller,link_parameters])


    def compile(self,details=None):
        self.set_inverse_kinematics(details=details)
        for i in range(len(self.pre_links)):
            self.links.append(Link(controller=self.pre_links[0],link_parameters=self.pre_links[i][-1],isolated=False))
        for i in range(len(self.links)):

            if i==0:
                self.links[i].connect_links(previous=None,next=self.links[i+1])
            elif i==len(self.links)-1:
                self.links[i].connect_links(previous=self.links[i-1],next=None)
            else:
                self.links[i].connect_links(previous=self.links[i-1],next=self.links[i+1])

        self.links[0].set_base()
        self.links[-1].set_end_effector()

        self.update_geometry()
        for i in range(len(self.links)):
            self.links[i].controller.link_system(self.links[i])
            self.links[i].tip_estimator=cont.Estimator(function='tip',syst=self.links[i],method='Modified Euler')
            self.links[i].motor_estimator=cont.Estimator(function='servo',syst=self.links[i],method='Modified Euler')


    def set_inverse_kinematics(self,details=None):
        if len(self.links)==2:
            self.get_angles=two_link_inv_kin

    def update_geometry(self):
        for i in range(1,len(self.links)):
            self.links[-i-1].update_geometry()

    def update_ref(self,bypass=True):
        self.ref.append([10,10])
        if bypass:
            self.angular_positions.append([1,1])

    def estimate_positions(self):
        for i in range(len(self.links)):
            self.links[i].estimate_motor_angle()
            self.links[i].estimate_tip_angle()

def two_link_inv_kin(cartesian_coords,robot):
    x,y=cartesian_coords
    d=np.sqrt(x**2,y**2)
    lr1=robot.links[0].lr
    lr2=robot.links[1].lr
    cos_theta2=(d**2+lr2**2-lr1**2)/(2*d*lr2)
    alpha_abs=math.acos((d-lr2*cos_theta2)/lr1)
    alphas=[-alpha_abs,alpha_abs]
    theta1_list=[]
    for alpha_value in alphas:
        aux=math.acos(x/d)
        if y!=d*math.sin(aux):
            aux*=-1
        theta1_list.append(aux-alpha_value)
    theta1=np.min(np.abs(np.array(theta1_list)-robot.links[0].motor_angle[-1]))
    alpha=alphas[np.argmin(np.abs(np.array(theta1_list)-robot.links[0].motor_angle[-1]))]
    theta2=np.sign(alpha)*math.acos(cos_theta2)
    return [theta1,theta2]


def limit_control_action_old(action_y,pos,t,limit_values,time_interval,limit_bool):
    value=action_y[-1]
    if limit_bool:
        #print('action',value,'max_speed',limit_values[0])

        if np.abs(value)>limit_values[0]:
            #print('checked1')
            value=limit_values[0]*np.sign(value)
        if len(pos)==1:

            if np.abs(value/time_interval)*params.J>limit_values[1]:
                #print('checked2')
                value=(limit_values[1]/params.J)*time_interval*np.sign(value)
        else:
            prev_velocity=(pos[-1]-pos[-2])/(t[-1]-t[-2])
            if np.abs((value-prev_velocity)/(t[-1]-t[-2]))*params.J>limit_values[1]:
                #print('checked3',prev_velocity)
                value=limit_values[1]*(t[-1]-t[-2])*((value-prev_velocity)/np.abs(value-prev_velocity))\
                      /params.J+prev_velocity
            #print('torque',(value-prev_velocity)*params.J/(t[-1]-t[-2]))
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