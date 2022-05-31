import numpy as np 
import math

#Propriedades do bloco
hb=20#40   # Comprimento do bloco       INPUT mm
tb=10#30   # Altura do bloco       INPUT mm
wb=20#40   # Largura do bloco       INPUT mm
ro_b = 2710 # Massa volúmica do material do bloco    INPUT Kg/m³
mb=ro_b*hb*wb*tb*10**(-9)

# Propriedades da viga
lc = 150#300  # Comprimento da viga       INPUT mm
tc = 3  # Espessura da viga           INPUT mm
wc = 10#30  # Largura da viga            INPUT mm
Is = ((tc * 10 ** -3) ** 3) * (wc * 10 ** -3) / 12  # Momento de área da secção da viga
E = 68  # Módulo de Young              INPUT GPa
ro_c = 7700 # Massa volúmica do material da viga    INPUT Kg/m³
mc = ro_c*(lc+wb)*wc*tc*10**(-9)

#Massa na ponta
m = 1  # Massa       INPUT Kg
lg = lc # Distância entre o centro gravítico da massa e o início da porção livre da viga       INPUT mm
        #(de momento está aproximado ao comprimento total da porção livre da viga)

#Momentos de inércia sobre o eixo do motor em Kgm²
Jb = mb*(wb**2+hb**2)*(10**(-6))/12
Jc= mc*((tc**2+(lc+wb)**2)/12+((hb+tc)**2+lc**2)/4)*(10**(-6))
Jm=m*((lg+wb/2)**2+((tc+hb)/2)**2)*(10**(-6))
J=Jb+Jc+Jm

# Propriedades do sistema dinâmico -- Viga
k = 3 * E * 10 ** 9 * Is / ((lc * 10 ** -3) ** 3)  # Rigidez N/m
m = 1  # Massa                        INPUT Kg
wn_sq = k / m  # Quadrado da freq natural (rad/s)^2
wn = np.sqrt(wn_sq)

# Propriedades do sistema dinâmico -- Motor
winding_res=9.64 #Resistência interna    INPUT Ohm
Jrotor=2.233e-6 #Momento de inércia do rotor
kt=0.286 #Constante de torque     INPUT Nm/A
kb=9.98 #Constante de força contraeletromotriz induzida  INPUT mV/rpm
T=(Jrotor+J)/(kt*kb*120*math.pi/1000)
max_velocity=4500 #INPUT rpm
max_torque=0.48 #INPUT Nm
limit_values=[max_velocity*math.pi/30,max_torque]

#Coefs
coefs_beam_equation=[1,0,1/wn_sq] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
coefs_servo_equation=[0,1,T] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
coefs_try=[1,T]

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