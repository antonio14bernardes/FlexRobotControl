import keyboard
import Cont_NN as cont
import matplotlib.pyplot as plt
import Sistema_NN as sist
import tensorflow as tf
from tensorflow import keras
import numpy as np


def main():
    model=NN_Compensator()
    #model.add_LSTM(8)
    model.add_dense(5,'tanh')
    model.add_dense(3,'tanh')
    model.add_dense(1,'linear')
    optimizer = tf.keras.optimizers.Adam(learning_rate=1e-3)#,beta_1=0.98,beta_2=0.88)
    #optimizer = tf.keras.optimizers.RMSprop(learning_rate=1e-3,rho=0.9,momentum=0.0)
    #loss_fn = tf.keras.losses.MeanSquaredError()
    loss_fn=custom_loss_literalythesqrloss_scalar
    loss_metric = tf.keras.metrics.Mean()

    syst=sist.System
    controller=cont.PID

    refs=[1]

    trainer=NN_Coach(model,optimizer,loss_fn,loss_metric,syst,controller)
    trainer.train(refs_input=refs,train_mode='cont',update_weights_each_point=False,loss_thresh=0.01,epochs=1000000,break_combo='ctrl+2')

class NN_Compensator(keras.Model):
    def __init__(self):
        super(NN_Compensator,self).__init__()

        self.layer_list=[]

    def add_dense(self,num_units,activation):
        self.layer_list.append(keras.layers.Dense(num_units,activation=activation))

    def add_LSTM(self,num_cells):
        self.layer_list.append(keras.layers.LSTM(num_cells,dtype='float32'))

    def add_simpleRNN(self,num_cells):
        self.layer_list.append(keras.layers.SimpleRNN(num_cells,dtype='float32'))

    def call(self,inputs):
        x_pass=inputs
        for layer in self.layer_list:
            x_pass=layer(x_pass)
        return x_pass

class NN_Coach:
    def __init__(self,model,optimizer,loss_fn,loss_metric,syst,controller=None,const_t_int=True):
        if isinstance(model.layer_list[0],keras.layers.LSTM) or isinstance(model.layer_list[0],keras.layers.SimpleRNN):
            self.prepare_input=LSTM_prep_input
        elif isinstance(model.layer_list[0],keras.layers.Dense):
            self.prepare_input=dense_prep_input
        self.model=model
        self.optimizer=optimizer
        self.loss_fn=loss_fn
        self.loss_metric=loss_metric
        self.syst=syst
        self.controller=controller
        self.const_t_int=const_t_int

    def train(self, refs_input, epochs, train_mode,loss_thresh=0.0, update_weights_each_point=False, break_combo='ctrl+2'):
        # Iterate over epochs.
        key_press = False
        good_enough=False

        refs_array=np.random.choice(refs_input,size=epochs)

        if update_weights_each_point:
            self.get_grads=substitute_grads
            self.update_weights_in_loop=update_weights
            self.update_weights_out_loop=useless_function

        else:
            self.get_grads=update_grads
            self.update_weights_in_loop=useless_function
            self.update_weights_out_loop=update_weights

        if train_mode=='compensator' or train_mode=='comp':
            self.get_comp=comp_fnc
            self.get_ref_update=comp_update_ref
            self.get_control_action=comp_control_action
            self.get_motor_estimate=comp_estimate_motor

        elif train_mode=='controller' or train_mode=='cont':
            self.get_comp=useless_function
            self.get_ref_update=cont_update_ref
            self.get_control_action=cont_control_action
            self.get_motor_estimate=cont_estimate_motor

        for epoch in range(epochs):

            print("Start of epoch %d" % (epoch,))
            syst = self.syst(self.controller())
            self.data = syst.motor_angle

            syst_test = self.syst(self.controller())

            refs_test=[refs_array[epoch]]
            refs=[refs_array[epoch]]

            data_test = [refs_test,syst_test.motor_angle, syst_test.tip_angle,
                         syst_test.motor_velocity,syst_test.tip_velocity,syst_test.times] #NOTA: REFS E REFS_TEST TEM DE SER PRIMEIRO ELEMENTO NA LISTA
            data = [refs,syst.motor_angle, syst.tip_angle, syst.motor_velocity,syst.tip_velocity,syst.times]

            if self.const_t_int:
                self.times_update = syst.update_times_constant_interval
                self.get_interval = syst.get_initial_time_gap_NN_comp
            else:
                self.times_update = syst.update_times
                self.get_interval = useless_function

            self.get_interval(model=self.model, test_syst=syst_test, data=data_test,
                              prep_input_fnc=self.prepare_input, const_time_int=self.const_t_int, num_samples=100)

            syst.start()
            grads=np.zeros_like(self.model.trainable_weights)
            total_loss = 0
            # Iterate over the batches of the dataset. #WARNING!!!!! Not yet prepared for batch training
            while syst.times[-1] < 2:
                with tf.GradientTape() as tape:
                    X_input = self.prepare_input(data)
                    comp = self.get_comp(self.model,X_input)
                    self.get_ref_update(syst,refs[-1], comp)
                    action=self.get_control_action(syst,self.model,X_input)
                    self.times_update()
                    self.get_motor_estimate(syst,action)
                    syst.estimate_tip_angle()
                    refs.append(refs[0])
                    loss = self.loss_fn(refs[-1], syst.tip_angle[-1])
                    total_loss+=loss

                    grads=self.get_grads(grads,self.model,loss,tape)
                    self.update_weights_in_loop(grads,self.model,self.optimizer)


            self.update_weights_out_loop(grads,self.model,self.optimizer)


            if total_loss<loss_thresh:
                good_enough=True
                break

            elif keyboard.is_pressed(break_combo):
                key_press = True
                print('Keyboard Interrupt')
                break

            print("epoch %d: mean loss = %.4f" % (epoch, total_loss)) #self.loss_metric.result()

        if good_enough:
            #self.model.save("C:/Users/anton_n9xdx5h/PycharmProjects/RobotFlexivel/")
            print('good enough')
            print('Loss',total_loss)

        plt.plot(syst.times, syst.tip_angle)
        plt.show()

        if key_press:
            save=input('Save model? y/n')
            if save=='y':
                #self.model.save("C:/Users/anton_n9xdx5h/PycharmProjects/RobotFlexivel/")
                pass

def custom_loss_literalytheabsvalue(real,pred):
    if real-pred<0:
        return pred-real
    else:
        return real-pred

def custom_loss_literalythesqrloss_scalar(real,pred):
    return (real-pred)**2

def comp_estimate_motor(syst,action):
    syst.estimate_motor_angle()

def cont_estimate_motor(syst,action):
    syst.estimate_motor_angle_NN_cont(action)

def comp_control_action(syst,model,input):
    syst.control()

def cont_control_action(syst,model,input):
    return model(input)

def comp_update_ref(syst, refs, comp):
    syst.update_ref(refs, comp)

def cont_update_ref(syst,refs,comp):
    syst.update_ref(refs,0)

def comp_fnc(model,input):
    return model(input)

def update_grads(grads,model,loss,tape):
    return grads+tape.gradient(loss, model.trainable_weights)

def substitute_grads(grads,model,loss,tape):
    return  tape.gradient(loss, model.trainable_weights)

def update_weights(grads,model,optimizer):
    optimizer.apply_gradients(zip(grads, model.trainable_weights))

def train_and_loss_metric(loss,model,tape,optimizer,loss_metric):
    grads = tape.gradient(loss, model.trainable_weights)
    optimizer.apply_gradients(zip(grads, model.trainable_weights))
    #loss_metric(loss)

def LSTM_prep_input(input_data):

    input_array=[]
    for list in input_data:
        input_array.append(list)
    input_array=np.matrix.transpose(np.array(input_data))
    if len(input_array) > 1000:
        input_array = input_array[-1000:]

    X_input=np.reshape(input_array,(1,len(input_array),len(input_array[0]))).astype(np.float64)
    return X_input

def dense_prep_input(input_data):
    X_input=input_data
    X_input=np.matrix.transpose(np.array(X_input))
    X_input=np.reshape(X_input[-1],(1,len(X_input[0]))).astype(np.float64 )
    return X_input

def useless_function(*args):
    pass

def clip_gradient(gradient, max, min=None):
    for i in range(len(gradient)):
        gradient[i]=np.clip(a=gradient[i],a_min=-1,a_max=1)  #NOT READY YET

if __name__=='__main__':
    main()