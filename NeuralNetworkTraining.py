import keyboard
import Cont_NN as cont
import matplotlib.pyplot as plt
import Sistema_NN as sist
import tensorflow as tf
from tensorflow import keras
import numpy as np
import os
import pickle
import copy

def main():
    model=NN_Compensator()
    model.add_LSTM(20)
    model.add_dense(30,'tanh')
    model.add_dense(30,'tanh')
    model.add_dense(30,'tanh')
    model.add_dense(30,'tanh')
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
        self.model_name_general='none'

    def train(self, refs_input, epochs, train_mode,
              data_string=['refs','motor angle','tip angle','motor velocity','tip velocity'],
              data_to_plot=['tip angle','motor_angle'],loss_thresh=0.0, update_weights_each_point=False,
              time_loop_pass=1,save_thresh=30, break_combo='ctrl+2'):

        run(train=True,syst_class=self.syst,controller=self.controller,model=self.model,optimizer=self.optimizer,
            loss_fn=self.loss_fn,epochs=epochs,refs_input=refs_input,prepare_input=self.prepare_input,
            time_loop_pass=time_loop_pass,mode=train_mode,update_weights_each_point=update_weights_each_point,
            const_t_int=self.const_t_int,save_thresh=save_thresh,loss_thresh=loss_thresh,break_combo=break_combo,
            name=None,best_weights=None,data_string=data_string,data_to_plot=data_to_plot)


def run(train,syst_class,controller,model,optimizer,loss_fn,epochs,refs_input,prepare_input,time_loop_pass,mode,
        update_weights_each_point,const_t_int,save_thresh,loss_thresh,break_combo,name=None,best_weights=None,
        data_string=['refs','motor angle','tip angle','motor velocity','tip velocity'],
        data_to_plot=['tip angle','motor_angle']):
    # Iterate over epochs.
    key_press = False
    good_enough=False
    refs_array=prepare_ref_random_order(refs_input,epochs)

    if train:
        if update_weights_each_point:
            get_grads=substitute_grads
            update_weights_in_loop=update_weights
            update_weights_out_loop=useless_function

        else:
            get_grads=update_grads
            update_weights_in_loop=useless_function
            update_weights_out_loop=update_weights
    else:
        get_grads=useless_function
        update_weights_in_loop=useless_function
        update_weights_out_loop=useless_function
        epochs=1
        model.set_weights(best_weights)

    if mode=='compensator' or mode=='comp':
        get_comp=comp_fnc
        get_ref_update=comp_update_ref
        get_control_action=comp_control_action
        get_motor_estimate=comp_estimate_motor

    elif mode=='controller' or mode=='cont':
        get_comp=useless_function
        get_ref_update=cont_update_ref
        get_control_action=cont_control_action
        get_motor_estimate=cont_estimate_motor

    first_save=True
    best_total_loss=save_thresh
    try:
        for epoch in range(epochs):
            if train:
                print("Start of epoch %d" % (epoch,))
            else:
                print('Running best weights')

            syst = syst_class(controller())

            syst_test = syst_class(controller())

            refs=[refs_array[epoch]]

            if const_t_int:
                times_update = syst.update_times_constant_interval
                get_interval = syst.get_initial_time_gap_NN_comp
            else:
                times_update = syst.update_times
                get_interval = useless_function

            get_interval(model=model, test_syst=syst_test, data=data_string,
                              prep_input_fnc=prepare_input, const_time_int=const_t_int,
                              num_samples=100,aldrabate=True)

            syst.start(new_ref=refs[-1])
            data=get_data(syst,data_string)
            grads=np.zeros_like(model.trainable_weights)
            total_loss = 0

            # Iterate over the batches of the dataset. #WARNING!!!!! Not yet prepared for batch training
            while syst.times[-1] < time_loop_pass:
                with tf.GradientTape() as tape:
                    X_input = prepare_input(data)
                    comp = get_comp(model,X_input)
                    get_ref_update(syst,refs[-1], comp)
                    action=get_control_action(syst,model,X_input)
                    times_update()
                    get_motor_estimate(syst,action)
                    syst.estimate_tip_angle()
                    refs.append(refs[0])
                    loss = loss_fn(refs[-1], syst.tip_angle[-1])
                    total_loss+=loss

                    grads=get_grads(grads,model,loss,tape)
                    update_weights_in_loop(grads,model,optimizer)

            update_weights_out_loop(grads,model,optimizer)

            if train:
                name='StopedFirstEpoch'
                if total_loss<best_total_loss:
                    best_total_loss=total_loss
                    if first_save:
                        #path=os.getcwd()
                        general_name=''
                        for l in model.layer_list:
                            general_name+=l.name[0]+str(l.trainable_weights[-1].shape[0])
                        name=general_name+'-'+str(total_loss[0,0])
                        first_save=False
                        best_weights=copy.deepcopy(model.get_weights())

                    else:
                        name=general_name+'-'+str(total_loss.numpy()[0,0])
                        best_weights=copy.deepcopy(model.get_weights())

                if total_loss<loss_thresh:
                    good_enough=True
                    break

                elif keyboard.is_pressed(break_combo):
                    key_press = True
                    print('Keyboard Interrupt')
                    break

                print("epoch %d: mean loss = %.4f" % (epoch, total_loss)) #self.loss_metric.result()

            else:
                get_plot(syst,data=data_to_plot,title=name)
                save=input('Save model? y/n')
                if save=='y':
                    save_object(best_weights,name)

        if train and (good_enough or key_press):

            if good_enough:
                print('Good enough')
                save_object(best_weights,name)
                get_plot(syst,data=data_to_plot,title=name)
            elif key_press:
                print('Break combo pressed','('+break_combo+')')
                get_plot(syst,data=data_to_plot,title=name)
                save=input('Save model? y/n')
                if save=='y':
                    save_object(best_weights,name)
            print('Loss',total_loss)
    except KeyboardInterrupt:
        if train:
            print('Here')
            run(train=False,syst_class=syst_class,controller=controller,model=model,optimizer=optimizer,
                loss_fn=loss_fn,epochs=epochs,refs_input=refs_input,prepare_input=prepare_input,
                time_loop_pass=time_loop_pass,mode=mode,update_weights_each_point=update_weights_each_point,
                const_t_int=const_t_int,save_thresh=save_thresh,loss_thresh=loss_thresh,break_combo=break_combo,
                name=name,best_weights=best_weights,data_string=data_string,data_to_plot=data_to_plot)




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
    #for list in input_data:
        #input_array.append(list)
    #print('input data',input_data)
    input_array=np.matrix.transpose(np.array(input_data))
    #print('transposed',input_array)
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

def save_object(obj,name):
    with open(name, 'wb') as weight_save_file:
        pickle.dump(obj, weight_save_file)

def get_data(syst,data):
    return_data=[]
    #print(syst)
    if data[0]!='refs' and data[0]!='reference':
        return_data.append(syst.ref)
    for value in data:
        if value=='motor angle' or value=='motor position':
            return_data.append(syst.motor_angle)
        elif value=='motor velocity':
            return_data.append(syst.motor_velocity)
        elif value=='tip angle' or value=='tip position':
            return_data.append(syst.tip_angle)
        elif value=='tip velocity':
            return_data.append(syst.tip_velocity)
        elif value=='reference' or value=='ref' or value=='refs':
            return_data.append(syst.ref)

    return return_data

def prepare_ref_random_order(refs_input,epochs):
    refs_array=np.random.choice(refs_input,size=epochs)
    return refs_array

def get_plot(syst,data,title):
    for value in data:
        if value=='motor angle' or value=='motor position':
            plt.plot(syst.times,syst.motor_angle,label=value)
        elif value=='motor velocity':
            plt.plot(syst.times,syst.motor_velocity,label=value)
        elif value=='tip angle' or value=='tip position':
            plt.plot(syst.times,syst.tip_angle,label=value)
        elif value=='tip velocity':
            plt.plot(syst.times,syst.tip_velocity,label=value)
        elif value=='reference' or value=='ref' or value=='refs':
            plt.plot(syst.times,syst.ref,label=value)
    plt.title(title)
    plt.legend()
    plt.show()

if __name__=='__main__':
    main()