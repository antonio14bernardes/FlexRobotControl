import keyboard
import numpy as np
import Controllers as cont
import time
import Parameters as params
#import RPi.GPIO as GPIO

class System:
    def __init__(self, controller,compensator='trace tf',limit_action=False):
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
        self.limit_action = limit_action
        self.tip_estimator=cont.Estimator(function='tip',method='Modified Euler')
        self.motor_estimator=cont.Estimator(function='servo',method='Modified Euler')
        self.compensator = cont.Compensator(compensator)
        self.motor_analyser=cont.Analysis()
        self.tip_analyser=cont.Analysis()
        #Initial values
        self.motor_angle=np.array([0])
        self.tip_angle=np.array([0])
        self.times=np.array([0])
        self.controller_output=np.array([0])
        #AUX
        self.time_interval=np.array([0.0001])
        self.numit=0
        self.ints=[]
        #print('WARNING!!\nCheck value time for first action limitation in limit action function')

    def setup_compensator(self,syst_obj,optimized_params=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]):
        self.compensator.setup(syst_obj,optimized_params)
        
    def start(self,input_ref=False):
        if input_ref:
            new_ref=float(input('Input Initial Target: '))
        else:
            new_ref=1
        self.ref=np.array([new_ref])
        self.start_time=time.time()
        self.compensator.update()
        
    ### Control Loop Begins Here ###    
    def update_times(self):
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

    def get_initial_time_gap(self,test_syst,num_samples=10000):
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

    def get_initial_time_gap_flexible(self,syst_test,num_samples=10000):  #NOT YET READY BUT SHOULD BE VERY USEFUL

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
    def compensate_ref(self):
        self.compensator.compensate()
        
    def control(self):
        if isinstance(self.controller,cont.PID):
            self.controller.get_action(self.compensator.comp_ref,self.motor_angle,self.times)
        if isinstance(self.controller,cont.LQR):
            self.controller.get_action(self.compensator.comp_ref,self.tip_angle,self.motor_angle,self.times)  #created for lqr
        self.controller_output=np.append(self.controller_output,limit_control_action(self.controller.action,
                        self.motor_angle,self.times,params.limit_values,self.time_interval,self.limit_action))

    def update_motor_angle(self):
        pass

    def estimate_motor_angle(self):
        self.motor_estimator.estimate(times=self.times[-2:],latest_input=self.controller_output[-1])
        self.motor_angle=np.append(self.motor_angle,self.motor_estimator.ys[0])

    def update_tip_angle(self):
        pass

    def estimate_tip_angle(self):
        self.tip_estimator.estimate(times=self.times[-2:], latest_input=self.motor_angle[-1])
        self.tip_angle = np.append(self.tip_angle,self.tip_estimator.ys[0])
        
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

def limit_control_action(action_y,pos,t,limit_values,time_interval,limit_bool):
    value=action_y[-1]
    if limit_bool:
        #print('action',value,'max_speed',limit_values[0])

        if np.abs(value)>limit_values[0]:
            #print('checked1')
            value=limit_values[-1]*value/np.abs(value)
        if len(pos)==1:

            if np.abs(value/time_interval)*params.J>limit_values[1]:
                #print('checked2')
                value=limit_values*time_interval*value/np.abs(value)
        else:
            prev_velocity=(pos[-1]-pos[-2])/(t[-1]-t[-2])
            if np.abs(((value-prev_velocity)/(t[-1]-t[-2])))*params.J>limit_values[1]:
                #print('checked3',prev_velocity)
                value=limit_values[1]*(t[-1]-t[-2])*((value-prev_velocity)/np.abs(value-prev_velocity))\
                      /params.J+prev_velocity
            #print('torque',(value-prev_velocity)*params.J/(t[-1]-t[-2]))
    return value

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