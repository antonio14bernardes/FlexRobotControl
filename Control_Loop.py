import Sistema as sist
import Controllers as cont
import matplotlib.pyplot as plt
import numpy as np

controller = cont.PID(K = [236.71653478, 24.59454428, 7.24508008]) #[392.69721817, 33.78080878, 9.00057794][206.71653478, 24.59454428, 7.24508008][25.29891655, 11.74971456,  2.27491559])#[50, 13.40789948,  1.77706101]

barry = sist.Barry(ppr=1000,angular_velocity=10)
syst=sist.System(controller,limit_action=None)
syst.setup_compensator(syst_obj=syst,optimized_params=[1,2*0.37/149,1/(149**2)])#[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
syst.get_initial_time_gap(sist.System,num_samples=100)
syst.start(input_ref=False)
i=0
while syst.times[-1]<0.3:

    i+=1

    syst.compensate_ref()
    syst.control()
    #syst.update_times()
    syst.update_times_constant_interval()
    syst.estimate_motor_angle()
    syst.estimate_tip_angle()
    syst.update_ref()


syst.get_analysis()

print('Max Values')
print('max velocity',np.max(np.abs(syst.tip_analyser.velocity)))
print('max acceleration',np.max(np.abs(syst.tip_analyser.acceleration)))
print('max torque',np.max(np.abs(syst.tip_analyser.torque)))
print('\n\n\n')
print('Process Run Info')
print('non zero times',np.count_nonzero(np.array(syst.times[1:])-np.array(syst.times[:len(syst.times)-1]))/len(syst.times))
try:
    print('times constant interval',syst.time_interval)
except:
    print('times were being updated with times.times() or times.process_time()')
print('number of iterations',i)
print('\n\n\n')
print('Performance:\n')
print('overshoot',syst.tip_analyser.overshoot)
print('settling time',syst.tip_analyser.settling_time)

y=[0]
inputs=syst.ref[-1]
estimator=cont.Estimator(function=[1,2*0.6/45,1/(45**2)], initial_values=[], initial_input=inputs, method='Modified Euler')
for i in range(1,len(syst.times)):
    estimator.estimate(syst.times[:i+1],inputs)
    y.append(estimator.ys[0])
plt.plot(syst.times,syst.motor_angle,label='motor')
plt.plot(syst.times,syst.tip_angle,label='tip')
plt.plot(syst.times,y,label='trace')
plt.legend()
plt.show()