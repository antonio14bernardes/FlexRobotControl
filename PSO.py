import numpy as np
import Sistema as sist
import Controllers as cont
import Parameters as params
import matplotlib.pyplot as plt
import keyboard

def main():
    #INPUT VALUES

    #Optimization
    num_its=100       #number of iterations

    #Particles
    num_particles=20    #number of particles
    w=[0.9,0.4]         #inertia
    c1=[2.5,0.5]        #memory
    c2=[0.5,2.5]        #cooperation

    w_update=linear_update
    c1_update=linear_update
    c2_update=linear_update

    #Limits
    pos_upper_limit=[400,300,2,2,2]#[0.9,200,30]
    pos_lower_limit=[300,200,0,0,0.0001]#[0,100,0]
    vel_upper_limit=[3,3,0.4,0.4,0.4]#[0.05,2,1]
    vel_lower_limit=[-3,-3,-0.4,-0.4,-0.4]#[-0.05,-2,-1]

    #Fitness
    fitness_fnc=fitness

    #System
    system_fnc=system_LQR
    #Initial Velocity
    initially_static=True

    best_particle=optimize(num_particles,num_its,w,c1,c2,pos_upper_limit,pos_lower_limit,vel_upper_limit,
                           vel_lower_limit,fitness_fnc,system_fnc,w_update,c1_update,c2_update,initially_static)

    print('Best Position',best_particle.best_position[-1])

    syst_out=system_fnc(best_particle.best_position[-1])
    print('Average Error',syst_out.tip_analyser.avg_error)
    print('Overshoot',syst_out.tip_analyser.overshoot)
    print('Settling Time',syst_out.tip_analyser.settling_time)
    plt.plot(syst_out.times,syst_out.tip_angle,label='tip')
    plt.plot(syst_out.times,syst_out.motor_angle,label='motor')
    plt.legend()
    plt.show()



class Particle:
    def __init__(self,num_its,w,c1,c2,pos_upper_limit,pos_lower_limit,vel_upper_limit,vel_lower_limit,fitness_fnc,
                 system_fnc,w_update_fnc,c1_udpate_fnc,c2_update_fnc,initially_static=False):
        self.fitness_fnc=fitness_fnc
        self.system_fnc=system_fnc
        self.w_init,self.w_final=w
        self.c1_init,self.c1_final=c1
        self.c2_init,self.c2_final=c2
        self.w_update_fnc=w_update_fnc
        self.c1_update_fnc=c1_udpate_fnc
        self.c2_update_fnc=c2_update_fnc

        self.pos_upper_limit=pos_upper_limit
        self.pos_lower_limit=pos_lower_limit
        self.vel_upper_limit=vel_upper_limit
        self.vel_lower_limit=vel_lower_limit

        self.position=np.array(
            [[np.random.uniform(low=low,high=high) for low,high in zip(pos_lower_limit,pos_upper_limit)]])
        if initially_static:
            self.velocity=np.zeros_like(self.position)
        else:
            self.velocity=np.array(
                [[np.random.uniform(low=low,high=high) for low,high in zip(vel_lower_limit,vel_upper_limit)]])
        init_fit=fitness_fnc(self.position[0],system_fnc)
        self.fitness=[init_fit]
        self.best_fitness=[init_fit]
        self.best_position=[self.position[0]]
        self.iteration=0
        self.num_its=num_its

    def update(self,latest_best_global_pos):
        self.w=self.w_update_fnc(self.num_its,self.iteration,self.w_init,self.w_final)
        self.c1=self.c1_update_fnc(self.num_its,self.iteration,self.c1_init,self.c1_final)
        self.c2=self.c2_update_fnc(self.num_its,self.iteration,self.c2_init,self.c2_final)

        new_velocity=self.w*self.velocity[-1]+self.c1*np.random.uniform(0,1)*(self.best_position[-1]-self.position[-1])\
            +self.c2*np.random.uniform(0,1)*(latest_best_global_pos-self.position[-1])
        new_velocity=clip(new_velocity,self.vel_upper_limit,self.vel_lower_limit)
        new_position=self.position[-1]+new_velocity
        new_position=clip(new_position,self.pos_upper_limit,self.pos_lower_limit)
        self.velocity=np.append(self.velocity,[new_velocity],axis=0)
        self.position=np.append(self.position,[new_position],axis=0)

        new_fit=self.fitness_fnc(self.position[-1],self.system_fnc)
        self.fitness.append(new_fit)
        if self.fitness[-1]<self.best_fitness[-1]:
            self.best_fitness.append(self.fitness[-1])
            self.best_position.append(self.position[-1])
        self.iteration+=1

def system_LQR(ks):
    Q=np.diag(ks[:len(ks)-1])
    R=np.array(ks[-1])
    controller = cont.LQR(A=params.A,B=params.B,Q=Q,R=R)
    syst = sist.System(controller,compensator='sigmoid')
    syst.setup_compensator(syst,optimized_params=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    syst.get_initial_time_gap_LQR(sist.System,num_samples=100)
    syst.start()
    i=0
    while syst.times[-1]<0.5:
        i+=1
        syst.compensate_ref()
        syst.control()
        syst.update_times_constant_interval()
        syst.estimate_motor_angle()
        syst.estimate_tip_angle()
        syst.update_ref()
    syst.get_analysis()

    return syst

def system_trace_tf(ks):

    coefs=[ks[2],2*ks[0]/ks[1],1/ks[1]**2]#[ks[2],2*ks[0]/ks[1],1/ks[1]**2]
    controller = cont.PID([236.71653478, 24.59454428, 7.24508008])
    syst = sist.System(controller,compensator='trace tf')
    syst.setup_compensator(syst,optimized_params=coefs)
    syst.get_initial_time_gap(sist.System,num_samples=100)
    syst.start()
    i=0
    while syst.times[-1]<0.5:
        i+=1
        syst.compensate_ref()
        syst.control()
        syst.update_times_constant_interval()
        syst.estimate_motor_angle()
        syst.estimate_tip_angle()
        syst.update_ref()
    syst.get_analysis()

    return syst


def fitness(pos,syst_fnc):
    syst=syst_fnc(pos)
    #Calculate score
    avg_error_penalty=1
    overshoot_penalty=1000
    settling_time_penalty=1

    score=np.dot([avg_error_penalty,overshoot_penalty,settling_time_penalty],
                 [syst.tip_analyser.avg_error,syst.tip_analyser.overshoot,syst.tip_analyser.settling_time])

    return score


def linear_update(num_its,its,initial,final):
    return (final-initial)*its/(num_its-1)+initial

def inertia_update(num_its,its,initial,final):
    return (initial-final)-((num_its-its)/num_its)*final

def clip(arr,max,min):
    return np.clip(arr,min,max)

def optimize(num_particles,num_its,w,c1,c2,pos_upper_limit,pos_lower_limit,vel_upper_limit,vel_lower_limit,fitness_fnc,
             system_fnc,w_update_fnc,c1_udpate_fnc,c2_update_fnc,initially_static=False):
    key_press=False
    particles=[]
    print('Initializing Particles')
    for i in range(num_particles):
        print('Creating particle',i,'out of',num_particles)
        particles.append(Particle(num_its,w,c1,c2,pos_upper_limit,pos_lower_limit,vel_upper_limit,vel_lower_limit,
                                  fitness_fnc,system_fnc,w_update_fnc,c1_udpate_fnc,c2_update_fnc,initially_static))
    best_global_fitness=[particles[0].best_fitness[-1]]
    best_global_position=[particles[0].best_position[-1]]
    best_particle=particles[0]
    for i in range(1,len(particles)):
        if particles[i].best_fitness[-1]<best_global_fitness[-1]:
            best_particle=particles[i]
            best_global_position.append(particles[i].position[-1])
            best_global_fitness.append(particles[i].best_fitness[-1])
    print('Initialization Completed. Proceding to Optimization')

    for it in range(num_its):
        print('Iteration', it, 'out of', num_its)
        for i in range(len(particles)):
            particles[i].update(best_global_position[-1])
            if keyboard.is_pressed('ctrl+3'):
                key_press=True
                print('Keyboard Interrupt (alt key pressed)')
                break
        if key_press:
            print('Keyboard Interrupt')
            break

        for i in range(len(particles)):
            if particles[i].best_fitness[-1]<best_global_fitness[-1]:
                best_particle=particles[i]
                best_global_position.append(particles[i].position[-1])
                best_global_fitness.append(particles[i].best_fitness[-1])

        print('Best fitness score in iteration', it,'=',best_global_fitness[-1])

    return best_particle

if __name__=='__main__':
    main()