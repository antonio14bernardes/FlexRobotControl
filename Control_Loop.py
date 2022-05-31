import Sistema as sist
import Controllers as cont
import matplotlib.pyplot as plt
import numpy as np

controller = cont.PID(K = [236.71653478, 24.59454428, 7.24508008]) #[392.69721817, 33.78080878, 9.00057794][206.71653478, 24.59454428, 7.24508008][25.29891655, 11.74971456,  2.27491559])#[50, 13.40789948,  1.77706101]

barry = sist.Barry(ppr=1000,angular_velocity=10)
syst=sist.System(controller,limit_action=1)
syst.setup_compensator(syst,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
syst.get_initial_time_gap(sist.System,num_samples=100)
syst.start(input_ref=True)
i=0
while syst.times[-1]<100:

    i+=1

    syst.compensate_ref()
    syst.control()
    #syst.update_times()
    syst.update_times_constant_interval()
    syst.estimate_motor_angle()
    syst.estimate_tip_angle()
    syst.update_ref()


syst.get_analysis()


print('max velocity',np.max(np.abs(syst.motor_analyser.velocity)))
print('max acceleration',np.max(np.abs(syst.motor_analyser.acceleration)))
print('max torque',np.max(np.abs(syst.motor_analyser.torque)))
print('number of iterations',i)
print('\n\n\n')
print('Performance:\n')
print('overshoot',syst.motor_analyser.overshoot)
print('settling time',syst.motor_analyser.settling_time)
print('times',np.count_nonzero(np.array(syst.times[1:])-np.array(syst.times[:len(syst.times)-1]))/len(syst.times))
print(syst.time_interval)
plt.plot(syst.times,syst.motor_angle)
plt.show()