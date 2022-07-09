import Sistema as sist
import Controllers as cont
import matplotlib.pyplot as plt
import Parameters as params
import numpy as np


robot=sist.Robot()

robot.add_link(link_controller=cont.LQR(Q=np.diag([4.00000000e+02, 2.55488305e+02, 2.00000000e+00, 0.00000000e+00]),
                                        R=np.array([1.00000000e-04])),link_compensator=cont.Compensator('-'),
                                        link_parameters=params.link1)

robot.add_link(link_controller=cont.PID(K = [236.71653478, 24.59454428, 7.24508008]),
               link_compensator=cont.Compensator('trace tf'),link_parameters=params.link2)

robot.run(run_time=2.5,const_t_int=True,ref_generator=cont.Ramp(final_time=2.5,initial_xy=[100,100],final_xy=[200,120]),num_samples=100,
          position_give=0.1,velocity_give=0.1,acceleration_give=0.1)

robot.analyse(show_performance=0,plot_trajectory=1,from_settled=0,plot_coordinates=1,plot_angles=1)
