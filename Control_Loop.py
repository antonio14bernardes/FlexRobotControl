import Sistema as sist
import Controllers as cont
import matplotlib.pyplot as plt
import Parameters as params
import numpy as np


robot=sist.Robot(const_t_int=True,num_samples=100)

robot.add_link(link_controller=cont.LQR(Q=np.diag([4.00000000e+02, 2.55488305e+02, 2.00000000e+00, 0.00000000e+00]),
                                        R=np.array([1.00000000e-04])),link_compensator=cont.Compensator('-'),
                                        link_parameters=params.link1)

robot.add_link(link_controller=cont.PID(K = [236.71653478, 24.59454428, 7.24508008]),
               link_compensator=cont.Compensator('trace tf'),link_parameters=params.link2)

robot.compile()
robot.start(input_ref=True)
while robot.times[-1]<0.5:
    robot.control()
    robot.update_times()
    robot.estimate_positions()
    robot.update_ref()
    robot.update_geometry()

robot.analyse(show_performance=True,plot_coordinates=True,plot_angles=True)