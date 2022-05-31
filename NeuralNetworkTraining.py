import keyboard
import Cont_NN as cont
import matplotlib.pyplot as plt
import Sistema_NN as sist
import tensorflow as tf
from tensorflow import keras
import numpy as np


def main():
    model=NN_Compensator()
    model.add_LSTM(10)
    model.add_dense(4,'relu')
    model.add_dense(1,'sigmoid')
    optimizer = tf.keras.optimizers.SGD(learning_rate=1e-20)
    loss_fn = tf.keras.losses.MeanSquaredError()
    loss_metric = tf.keras.metrics.Mean()

    syst=sist.System
    controller=cont.PID

    refs=np.ones((1,1))

    trainer=NN_Coach(model,optimizer,loss_fn,loss_metric,syst,controller)
    trainer.train_as_comp(refs,update_weights_each_point=False,epochs=1000000,break_combo='ctrl+2')

class NN_Compensator(keras.Model):
    def __init__(self):
        super(NN_Compensator,self).__init__()

        self.layer_list=[]

    def add_dense(self,num_units,activation):
        self.layer_list.append(keras.layers.Dense(num_units,activation=activation))

    def add_LSTM(self,num_cells):
        self.layer_list.append(keras.layers.LSTM(num_cells,dtype='float32'))

    def call(self,inputs):
        x_pass=inputs
        for layer in self.layer_list:
            x_pass=layer(x_pass)
        return x_pass

class NN_Coach:
    def __init__(self,model,optimizer,loss_fn,loss_metric,syst,controller=None,const_t_int=True):
        if isinstance(model.layer_list[0],keras.layers.LSTM):
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

    def train_as_comp(self, refs, epochs, update_weights_each_point=False, break_combo='ctrl+2'):
        # Iterate over epochs.
        key_press = False
        if update_weights_each_point:
            gradient_fnc_in_loop=train_and_loss_metric
            gradient_fnc_out_loop=useless_function
        else:
            gradient_fnc_out_loop=train_and_loss_metric
            gradient_fnc_in_loop=useless_function

        for epoch in range(epochs):

            print("Start of epoch %d" % (epoch,))
            syst = self.syst(self.controller())
            self.data = syst.motor_angle

            syst_test = self.syst(self.controller())
            data_test = [syst_test.motor_angle, syst_test.tip_angle]
            data = [syst.motor_angle, syst.tip_angle]

            if self.const_t_int:
                self.times_update = syst.update_times_constant_interval
                self.get_interval = syst.get_initial_time_gap_NN_comp
            else:
                self.times_update = syst.update_times
                self.get_interval = useless_function

            self.get_interval(model=self.model, test_syst=syst_test, data=data_test,
                              prep_input_fnc=self.prepare_input, const_time_int=self.const_t_int, num_samples=100)

            syst.start()
            loss = 0
            # Iterate over the batches of the dataset. #WARNING!!!!! Not yet prepared for batch training

            with tf.GradientTape() as tape:
                while syst.times[-1] < 2:
                    X_input = self.prepare_input(data)
                    comp = self.model(X_input)
                    syst.update_ref(refs, comp)
                    syst.control()
                    self.times_update()
                    syst.estimate_motor_angle()
                    syst.estimate_tip_angle()
                    loss += self.loss_fn(refs, syst.tip_angle[-1])
                    gradient_fnc_in_loop(loss, self.model, tape, self.optimizer, self.loss_metric)

            gradient_fnc_out_loop(loss,self.model,tape,self.optimizer,self.loss_metric)

            if keyboard.is_pressed(break_combo):
                key_press = True
                print('Keyboard Interrupt')
                break

            print("epoch %d: mean loss = %.4f" % (epoch, self.loss_metric.result()))

        plt.plot(syst.times, syst.tip_angle)
        plt.show()
        save=input('Save model? y/n')
        if save=='y':
            self.model.save("C:/Users/anton_n9xdx5h/PycharmProjects/RobotFlexivel/saved_model_comp")

    def train_as_cont(self, refs, epochs, update_weights_each_point=False, break_combo='ctrl+2'):
        # Iterate over epochs.
        key_press = False
        if update_weights_each_point:
            gradient_fnc_in_loop = train_and_loss_metric
            gradient_fnc_out_loop = useless_function
        else:
            gradient_fnc_out_loop = train_and_loss_metric
            gradient_fnc_in_loop = useless_function

        for epoch in range(epochs):

            print("Start of epoch %d" % (epoch,))
            syst = self.syst(self.controller())
            self.data = syst.motor_angle

            syst_test = self.syst(self.controller())
            data_test = [syst_test.motor_angle, syst_test.tip_angle]
            data = [syst.motor_angle, syst.tip_angle]

            if self.const_t_int:
                self.times_update = syst.update_times_constant_interval
                self.get_interval = syst.get_initial_time_gap_NN_comp
            else:
                self.times_update = syst.update_times
                self.get_interval = useless_function

            self.get_interval(model=self.model, test_syst=syst_test, data=data_test,
                              prep_input_fnc=self.prepare_input, const_time_int=self.const_t_int, num_samples=100)

            syst.start()
            loss = 0
            # Iterate over the batches of the dataset. #WARNING!!!!! Not yet prepared for batch training

            with tf.GradientTape() as tape:
                while syst.times[-1] < 2:
                    X_input = self.prepare_input(data)
                    #comp = self.model(X_input)
                    syst.update_ref(refs, 0)
                    syst.control()
                    self.times_update()
                    action=self.model(X_input)
                    syst.estimate_motor_angle_NN_cont(action)
                    syst.estimate_tip_angle()
                    loss += self.loss_fn(refs, syst.tip_angle[-1])
                    gradient_fnc_in_loop(loss, self.model, tape, self.optimizer, self.loss_metric)

            gradient_fnc_out_loop(loss, self.model, tape, self.optimizer, self.loss_metric)

            if keyboard.is_pressed(break_combo):
                key_press = True
                print('Keyboard Interrupt')
                break

            print("epoch %d: mean loss = %.4f" % (epoch, self.loss_metric.result()))

        plt.plot(syst.times, syst.tip_angle)
        plt.show()
        save = input('Save model? y/n')
        if save == 'y':
            self.model.save("C:/Users/anton_n9xdx5h/PycharmProjects/RobotFlexivel/saved_model_cont")

def train_and_loss_metric(loss,model,tape,optimizer,loss_metric):
    grads = tape.gradient(loss, model.trainable_weights)
    optimizer.apply_gradients(zip(grads, model.trainable_weights))
    loss_metric(loss)

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
    X_input=X_input.reshape((1,len(X_input)))
    return X_input

def useless_function(*args):
    pass

def clip_gradient(gradient, max, min=None):
    for i in range(len(gradient)):
        gradient[i]=np.clip(a=gradient[i],a_min=-1,a_max=1)  #NOT READY YET

if __name__=='__main__':
    main()