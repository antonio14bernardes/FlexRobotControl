import numpy as np
import Cont_NN as cont
import time
import Parameters as params
import tensorflow as tf

# import RPi.GPIO as GPIO

class System:
    def __init__(self, controller):
        # Propriedades do material
        self.E = params.E  # Módulo de Young              INPUT GPa
        # Dimensões da viga
        self.lc = params.lc  # Comprimento da viga       INPUT mm
        self.tc = params.tc  # Espessura da viga           INPUT mm
        self.wc = params.wc  # Largura da viga            INPUT mm
        self.Is = ((self.tc * 10 ** -3) ** 3) * (self.wc * 10 ** -3) / 12  # Momento de área da secção da viga
        # Propriedades do sistema a controlar
        self.k = 3 * self.E * 10 ** 9 * self.Is / ((self.lc * 10 ** -3) ** 3)  # Rigidez N/m
        self.m = params.m  # Massa                        INPUT Kg
        self.wn_sq = self.k / self.m  # Quadrado da freq natural (rad/s)^2
        self.wn = np.sqrt(self.wn_sq)
        # Sub-sistemas
        self.controller = controller
        self.tip_estimator = cont.Estimator(function='tip', method='Modified Euler')
        self.motor_estimator = cont.Estimator(function='servo', method='Modified Euler')
        self.compensator = cont.Compensator()
        self.motor_analyser = cont.Analysis()
        self.tip_analyser = cont.Analysis()
        # Initial values
        self.motor_angle = [0]
        self.tip_angle = [0]
        self.times = [0]
        self.controller_output = [0]
        #AUX
        self.time_interval=[0.001]
        self.numit=0
        self.ints=[]


        self.tryer=cont.tryer()

    def setup_compensator(self, syst_obj, optimized_params=[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]):
        self.compensator.setup(syst_obj, optimized_params)

    def start(self):
        # new_ref=float(input('Input Initial Target: '))
        new_ref = 1
        self.ref = [new_ref]
        self.start_time = time.process_time()
        #self.compensator.update()

    ### Control Loop Begins Here ###
    def compensate_ref(self):
        self.compensator.compensate()

    def try_dif(self,ref,cont):
        self.tryer.tries(ref,cont)

    def control(self):
        self.controller.get_action(self.comp_ref, self.motor_angle, self.times)  # CHanged to motor angle

    def update_motor_angle(self):
        pass

    def update_tip_angle(self):
        pass

    def update_times(self):
        self.times.append(time.process_time() - self.start_time)

    def update_times_constant_interval(self,num_samples=100):
        self.times.append(self.times[-1]+self.time_interval[-1])
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
        #syst_test.setup_compensator(syst_test, [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        syst_test.start()
        for i in range(num_samples):
            #syst_test.compensate_ref()
            syst_test.control()
            syst_test.update_times_constant_interval()
            syst_test.estimate_motor_angle()
            syst_test.estimate_tip_angle()
            syst_test.update_ref()
        current=time.time()
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])

    def get_initial_time_gap_NN_comp(self,model,test_syst,data,prep_input_fnc,const_time_int,num_samples=50):

        refs=np.ones((1,1))
        syst_test=test_syst
        syst_test.start()
        ref=refs[0]
        if const_time_int:
            update_times_fnc=syst_test.update_times_constant_interval
        else:
            update_times_fnc=syst_test.update_times
        #for step, x_point_train in enumerate(Xtrain):
        for i in range(num_samples):
            with tf.GradientTape() as tape:
                data_to_prepare=data
                X_input=prep_input_fnc(data_to_prepare)
                comp = model(X_input)
                #print(comp)
                syst_test.update_ref(ref,comp)
                syst_test.control()
                #print(syst.controller.action[-1])
                update_times_fnc()
                syst_test.estimate_motor_angle()
                syst_test.estimate_tip_angle()

        current=time.process_time()
        #print('current',current)
        #print('dif times',current-syst_test.start_time)
        self.time_interval=np.array([(current-syst_test.start_time)/num_samples])
        #print(self.time_interval)
    def estimate_motor_angle(self):
        #print('action from sistema',self.controller.action[-1])
        self.motor_estimator.estimate(times=self.times[-2:], latest_input=self.controller.action[-1])
        self.motor_angle.append(self.motor_estimator.ys[0])
        #print('from sistNN',self.motor_angle)

    def estimate_motor_angle_NN_cont(self,action):
        self.motor_estimator.estimate(times=self.times[-2:], latest_input=action)
        self.motor_angle.append(self.motor_estimator.ys[0])

    def estimate_tip_angle(self):
        self.tip_estimator.estimate(times=self.times[-2:], latest_input=self.motor_angle[-1])
        self.tip_angle.append(self.tip_estimator.ys[0])

    def update_ref(self,ref,comp):
        '''
        if keyboard.is_pressed('alt'):
            new_ref=float(input('Input new ref'))
            self.ref=np.append(self.ref,new_ref)
        else:
            new_ref=self.ref[-1]
            self.ref=np.append(self.ref,new_ref)
            elf.compensator.update()

        '''
        #new_ref = self.ref[-1]
        self.comp_ref=ref-comp

    def get_analysis(self, ss_value=1):
        self.motor_analyser.analyse(self.motor_angle, self.times, ss_value)
        self.tip_analyser.analyse(self.tip_angle, self.times, ss_value)


class Barry:  # Classe para cenas de eletronica tipo receber dados do encoder e outros sensores
    def __init__(self, ppr, angular_velocity, duty_cycle=50, pins=[37, 35, 31, 29, 18, 16, 15, 13, 11], mode=0):
        self.ppr = ppr  # Pulses per rotation
        self.angular_velocity = angular_velocity  # In rot per sec
        max_pps = 250 * 10 ** 3
        self.pps = self.angular_velocity * self.ppr
        print('freq = ', self.pps)
        print('period = ', 1 / self.pps)
        if self.pps > max_pps:
            max_velocity = max_pps / self.ppr
            raise Exception(f"Pulses per second are set too high (limit = {max_velocity} rot/s or = 250 kpps)")

        self.duty_cycle = duty_cycle
        if mode == 0:
            self.send_pulses = mode_0
            self.high_level_motion_function = self.send_position_increment
        elif mode == 1:
            self.send_pulses = mode_1
            self.high_level_motion_function = self.send_position_increment
        elif mode == 'PWM':
            self.send_pulses = mode_pwm
            self.high_level_motion_function = self.send_angular_velocity
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
        # Encoder txt
        f = open(params.file_path, 'w')
        f.write('')
        f.close()

        print('setup done')

    def send_position_increment(self, increment):
        no_pulses = self.ppr * increment / 360
        self.send_pulses(no_pulses, self.pps, [self.pc1, self.pc1_, self.pc2, self.pc2_], duty_cycle=self.duty_cycle)

    def send_angular_velocity(self, angular_velocity):
        pulse_freq = self.ppr * angular_velocity
        self.send_pulses(freq=pulse_freq, pins=[self.pc1, self.pc1_, self.pc2, self.pc2_], pwm_pin=self.pwm_pin,
                         duty_cycle=self.duty_cycle)

    def send_motion(self, pos_or_vel):
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
    elif no_pulses < 0:
        no_pulses = -no_pulses
        GPIO.output(sign, 0)
        GPIO.output(sign_, 1)

    while pulse_counter < no_pulses:
        GPIO.output(puls, 1)
        GPIO.output(puls_, 0)
        time_wait(active_time)
        GPIO.output(puls, 1)
        GPIO.output(puls_, 1)
        time_wait(period - active_time)
        # pulse_counter+=1


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
            while timer + period - active_time > time.time():
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
            time_wait(period - active_time)
            pulse_counter += 1


def mode_pwm(freq, pins, pwm_pin, duty_cycle=50):
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
