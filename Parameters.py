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

'''
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

#State space so para nao dar erro
A=np.array([[0,0,1,0],[0,0,0,1],[-1,1,0,0],[0,0,0,-1/1]])
B=np.array([[0],[0],[0],[1/1]])
C=np.array([1,0,0,0])
D=np.array([0])

#Coefs
coefs_beam_equation=[1,0,1/wn_sq] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
coefs_servo_equation=[0,1,T] #[C1,C2,C3,...,Cn] Cn*x^n+Cn-1*x^n-1+...+C1*x=input
coefs_try=[1,T]
'''


