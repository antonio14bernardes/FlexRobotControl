# FlexRobotControl
Code regarding research project on control of robots with a non-ridgid structure.

The control loop file is exactly what its name suggests. This file calls the Sistema file which includes high-level control, estimation and analysis operations as well as control for the micro-controller (currently a raspberry pi). This file, in turn, calls the Controllers file which is responsible for the lower-level mathematical operations.

The genetic algo file allows for the deployment of a genetic algorithm that has been used for the optimization of various parameters in the multiple procedures that have been developed up to the point of uploading.

The title for the neural network training file is also quite self-explanatory. Since I am using tensorflow's gradient tape, I had to create a modification of the Sistema and Controllers files so that gradient tracking would work properly.

More recently, I have also created a modified Particle Swarm Optimization algorithm which has produced better results than those of the genetic algorithm.

Up to this point, the controllers that produced the best results were the Linear Quadratic Regulator controller and a novel Transfer Function Tracing controller.

Thanks for checking out my code. If you have any comments, please do not hesitate to contact me at antonio14bernardes@outlook.com.
If this code is of any use to you feel free to use it! I would be glad to serve the science/engineering community! (If you have the time let me know what the application of the code was, just out of curiosity)
