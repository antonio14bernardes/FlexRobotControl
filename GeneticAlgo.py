from ypstruct import structure
import numpy as np
import Controllers as cont
import Sistema as sist
import matplotlib.pyplot as plt
import keyboard

#Problem definition
problem = structure()
problem.nvar=1 #15 #num of decision variables
problem.varmin = [0]#[100,0.1]#[-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100,-100] #0 #lower bound for decision variables
problem.varmax = [1]#[200,0.9]#[100,100,100,100,100,100,100,100,100,100,100,100,100,100,100]#500 #upper bound for decision variables

#GA Parameters
params = structure()
params.maxit = 30 #max num of iterations
params.npop = 15 #Initial population size
params.pc = 7
params.start_gamma=-0.5
params.end_gamma=0.4
params.start_mu = 0.5
params.end_mu=0.1
params.sigma= 0.2

def main():
    out=run(problem,params,system)
    sys_out=system(ks=out.bestsol.position)
    y,motor,t,overshoot,settling_time,avg_error=sys_out.tip_angle,sys_out.motor_angle,sys_out.times,\
                sys_out.tip_analyser.overshoot,sys_out.tip_analyser.settling_time,sys_out.tip_analyser.avg_error
    print('Best Position',out.bestsol.position)
    print('Overshoot',overshoot,'Settling time',settling_time,'Average error',avg_error)
    print('Max speed', np.max(np.abs(sys_out.motor_analyser.velocity)),
          'Max torque', np.max(np.abs(sys_out.motor_analyser.torque)))
    plt.plot(t,y,label='tip')
    plt.plot(t,motor,label='motor')
    plt.legend()
    plt.show()

#System to optimize
def system(ks):
    #coefs=[1,2*ks[1]/ks[0],1/(ks[0]**2)]
    coefs=[1,ks[0]]
    controller = cont.PID([236.71653478, 24.59454428, 7.24508008])
    syst = sist.System(controller)
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
    #return syst.motor_angle,syst.times,syst.controller.overshoot,\
          # syst.controller.settling_time,syst.controller.summed_error

#Cost function
def calculate_cost(x,sys,return_info=False,constraint_limits=None):
    overshoot_penalty=100000000
    settling_time_penalty=10
    avg_error_penalty=1
    #_,_,overshoot,settling_time,summed_error=sys(x)
    out = sys(x)
    overshoot,settling_time,avg_error = out.tip_analyser.overshoot, \
                        out.tip_analyser.settling_time, out.tip_analyser.avg_error

    cost=np.absolute(overshoot)*overshoot_penalty+settling_time*settling_time_penalty\
         +np.absolute(avg_error)*avg_error_penalty

    if return_info:
        return cost,overshoot,settling_time,avg_error

    elif not constraint_limits is None:
        check=True
        constrained_values=[np.max(np.abs(out.motor_analyser.velocity)), np.max(np.abs(out.motor_analyser.torque))]
        for value, limit in zip(constrained_values, constraint_limits):
            if value>limit:
                check = False
                break
        return cost,check

    else:
        return cost


#Set cost function
problem.costfunc = calculate_cost

def run(problem, params, sys,constraint_limits=[3000,0.159]):
    #Problem informarion
    costfunc=problem.costfunc
    nvar=problem.nvar
    varmin=problem.varmin
    varmax=problem.varmax

    key_press=False

    #Parameters
    maxit = params.maxit
    npop = params.npop
    pc = params.pc
    nc = int(np.round(pc*npop/2)*2)
    end_gamma = params.end_gamma
    start_gamma = params.start_gamma
    end_mu = params.end_mu
    start_mu=params.start_mu
    sigma = params.sigma

    #Empty individual template
    empty_individual=structure()
    empty_individual.position= None
    empty_individual.cost=None

    #Best solution yet
    bestsol=empty_individual.deepcopy()
    bestsol.cost=np.inf

    #Initialize Population
    pop = empty_individual.repeat(npop)
    print('pop init')
    for i in range(0,npop):
        print(i,'out of',npop)
        num_tries=0
        check_constraints=False
        while not check_constraints:
            num_tries+=1
            changed_varmax=linear_limit_var(varmax,[1,0.5,0.1],num_tries,300)
            #if num_tries%50==0:
                #print('try no',num_tries)
                #print('current varmax', changed_varmax)
            pop[i].position = np.random.uniform(varmin, changed_varmax,nvar)
            #pop[i].cost,check_constraints=costfunc(pop[i].position,sys,constraint_limits=constraint_limits)
            pop[i].cost = costfunc(pop[i].position, sys)
            check_constraints = True

        #print(pop[i].position)
        if pop[i].cost < bestsol.cost:
            bestsol = pop[i].deepcopy()

    #Best cost of iteration
    bestcost=np.empty(maxit)
    print('entering iteration loop')
    for it in range(maxit):
        print('iteration',it, 'out of', maxit)
        popc = []

        gamma=linear_update_param(start_gamma, end_gamma, it, maxit)
        mu = linear_update_param(start_mu, end_mu, it, maxit)
        print('gamma',gamma)
        print('mu',mu)

        for _ in range(nc//2):
            print('nc', _, 'out of', nc//2 )
            q = np.random.permutation(npop)
            p1=pop[q[0]]
            p2=pop[q[1]]

            #Perform Crossover
            c1,c2 = crossover(p1,p2,gamma)

            #Perform Mutation
            c1 = mutate(c1, mu, sigma)
            c2 = mutate(c2,mu,sigma)

            #Apply bounds
            c1 = apply_bound(c1, varmin,varmax)
            c2 = apply_bound(c2,varmin,varmax)

            #Evaluate offspring
            c1.cost = costfunc(c1.position,sys)
            c2.cost = costfunc(c2.position,sys)

            '''
            #Apply constraints
            if not check_constraints(c1,sys,constraint_limits):
                c1.cost=np.inf

            if not check_constraints(c2,sys,constraint_limits):
                c2.cost=np.inf
            '''
            #Update best solution
            if c1.cost < bestsol.cost:
                bestsol = c1.deepcopy()

            if c2.cost < bestsol.cost:
                bestsol = c2.deepcopy()

            #Append offspring to list
            popc.append(c1)
            popc.append(c2)

            if keyboard.is_pressed('ctrl+1'):
                key_press=True
                print('Keyboard Interrupt (alt key pressed)')
                break

        #Merge, Sort and Select
        pop += popc
        sorted(pop,key=lambda x: x.cost)
        pop = pop[0:npop]

        #Store and display best cost
        bestcost[it]=bestsol.cost
        print('Iteration {}: Best Cost = {}'.format(it, bestcost[it]))

        if key_press:
            break
    #Output
    out=structure()
    out.pop=pop
    out.bestsol=bestsol
    out.bestcost=bestcost
    return out

def crossover(p1,p2, gamma=0.1):
    c1 = p1.deepcopy()
    c2 = p1.deepcopy()
    alpha = np.random.uniform(-gamma, 1+gamma, *c1.position.shape)
    c1.position = alpha*p1.position+(1-alpha)*p2.position
    c2.position = alpha*p2.position+(1-alpha)*p1.position
    return c1, c2

def mutate(x,mu,sigma):
    y = x.deepcopy()
    flat = np.random.rand(*x.position.shape) <= mu
    ind=np.argwhere(flat)
    y.position[ind] += sigma*np.random.randn(*ind.shape)
    return y

def apply_bound(x,varmin,varmax):
    x.position = np.maximum(x.position, varmin)
    x.position =np.minimum(x.position, varmax)
    return x

def check_constraints(c,sys,constraint_limits=None):
    out = sys(c.position)
    constrained_values=[np.max(np.abs(out.motor_analyser.velocity)), np.max(np.abs(out.motor_analyser.torque))]
    
    if constraint_limits is None:
        return True
    else:
        check = True
        for value, limit in zip(constrained_values, constraint_limits):
            if value>limit:
                check = False
                break
        return check

def linear_update_param(start,end,it,max_it):
    m=(end-start)/(max_it-1)
    return m*it+start

def linear_limit_var(start, end, it, max_it):
    if it>max_it:
        return end
    else:
        changed_varmax=[]
        for start_value, end_value in zip(start, end):
            m=(end_value-start_value)/max_it
            changed_varmax.append(m*it+start_value)
        return changed_varmax

if __name__ == '__main__':
    main()