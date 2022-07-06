import numpy as np 
import math
#Propriedades do primeiro link

#Propriedades do bloco
hb1=20#40   # Comprimento do bloco       INPUT mm
tb1=10#30   # Altura do bloco       INPUT mm
wb1=20#40   # Largura do bloco       INPUT mm
ro_b1 = 2710 # Massa volúmica do material do bloco    INPUT Kg/m³

# Propriedades da viga
lc1 = 150  # Comprimento da viga       INPUT mm
lg1 = lc1  # Comprimento da viga até centro da massa       INPUT mm
tc1 = 3  # Espessura da viga           INPUT mm
wc1 = 10  # Largura da viga            INPUT mm
ro_c1 = 7700 # Massa volúmica do material da viga    INPUT Kg/m³
E1 = 68  # Módulo de Young              INPUT GPa

link1=[[hb1,tb1,wb1,ro_b1],[lc1,lg1,tc1,wc1,ro_c1,E1]]


#Propriedades do segundo link

#Propriedades do bloco
hb2=20   # Comprimento do bloco       INPUT mm
tb2=10   # Altura do bloco       INPUT mm
wb2=20   # Largura do bloco       INPUT mm
ro_b2 = 2710 # Massa volúmica do material do bloco    INPUT Kg/m³

# Propriedades da viga
lc2 = 150  # Comprimento da viga       INPUT mm
lg2 = lc2  # Comprimento da viga até centro da massa       INPUT mm
tc2 = 3  # Espessura da viga           INPUT mm
wc2 = 10  # Largura da viga            INPUT mm
E2 = 68  # Módulo de Young              INPUT GPa
ro_c2 = 7700 # Massa volúmica do material da viga    INPUT Kg/m³

link2=[[hb2,tb2,wb2,ro_b2],[lc2,lg2,tc2,wc2,ro_c2,E2]]

m = 1  # Massa                        INPUT Kg

# Propriedades do Motor
winding_res=9.64 #Resistência interna    INPUT Ohm
Jrotor=2.233e-6 #Momento de inércia do rotor
kt=0.286 #Constante de torque     INPUT Nm/A
kb=9.98 #Constante de força contraeletromotriz induzida  INPUT mV/rpm
max_velocity=4500 #INPUT rpm
max_torque=0.48 #INPUT Nm
limit_values=[max_velocity*math.pi/30,max_torque]

#Pins
pc1=37
pc1_=35
pc2=31
pc2_=29
dev_count_reset=18
dev_count_reset_=16
positioning_completed=15
phase_z=13
alarm = 11
encoder_A=5
encoder_B=3

pwm_pin=35

#Txt file
file_path='encoder.txt'