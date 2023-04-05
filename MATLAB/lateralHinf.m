load("modelo_lin.mat")
latmod;
latmod.A(4,4) = 0;
latmod.A(5,4) = 0;

%Peso de las perturbaciones
Wp      = makeweight(1,[10 0.1],0.01);
%Peso Error Ángulo Roll
W1Phi      = makeweight(10,[1 0.1],0.01);
%Peso Error Ángulo Yaw
W1Psi      = makeweight(10,[1 0.1],0.01);
%Peso Señal de Control Aileron
W2Aileron      = makeweight(0.1,[32 0.32],1);
%Peso Señal de Control Rudder
W2Rudder      = makeweight(0.1,[32 0.32],1);
%Peso Salida Ángulo Roll
W3Phi      = makeweight(0.01,[1 0.1],10);
%Peso Salida Ángulo Yaw
W3Psi      = makeweight(0.01,[1 0.1],10);
%Peso del ruido
Wn      = 0.001;
%Función Delta
%delta = ss([3 0; 0 1.5]);


systemnames     = 'latmod Wp Wn W1Phi W1Psi W2Aileron W2Rudder W3Phi W3Psi';
inputvar        = '[per; n; rPhi; rPsi; uAileron; uRudder]';
outputvar       = '[W1Phi; W1Psi; W2Aileron; W2Rudder; W3Phi; W3Psi; -latmod(4) - Wn + rPhi;-latmod(5) - Wn + rPsi]';
input_to_latmod   = '[-Wp + uAileron; -Wp + uRudder]';
input_to_Wn     = '[n]';
input_to_Wp     = '[per]';
input_to_W1Phi     = '[-latmod(4) - Wn + rPhi]';
input_to_W1Psi     = '[-latmod(5) - Wn + rPsi]';
input_to_W2Phi     = '[rPhi]';
input_to_W2Psi     = '[rPsi]';
input_to_W3Phi     = '[latmod(4)]';
input_to_W3Psi     = '[latmod(5)]';
%input_to_delta     = '[latmod(4); latmod(5)]';

%Planta generalizada
Planta = sysic;

%Diseño controlador Hinf
ncont = 2; 
nmeas = 2; 
[KHinf,CL,gamma] = hinfsyn(Planta,nmeas,ncont);

%Función de Lazo abierto
L = latmod*KHinf;

%Sistema en Lazo cerrado
Go = feedback(L,eye(2),[1,2],[4,5]);

%Funciones de Transferencia
Ltf = tf(L);

%Funciones de Sensibilidad
S1phi = feedback(1,Ltf(4,1)); %phi/Aileron
S2phi = feedback(1,Ltf(4,2)); %phi/Rudder
S1psi = feedback(1,Ltf(5,1)); %psi/Aileron
S2psi = feedback(1,Ltf(5,2)); %psi/Rudder

%Funciones de Transferencia ganancias
Ktf = tf(KHinf);

%Funciones de K*Sensibilidad
KS1phi = Ktf(1,1)*S1phi; %phi/Aileron
KS2phi = Ktf(1,2)*S2phi; %phi/Rudder
KS1psi = Ktf(1,1)*S1psi; %psi/Aileron
KS2psi = Ktf(1,2)*S2psi; %psi/Rudder

%Funciones de Sensibilidad Complementaria
T1phi = feedback(Ltf(4,1),1); %phi/Aileron
T2phi = feedback(Ltf(4,2),1); %phi/Rudder
T1psi = feedback(Ltf(5,1),1); %psi/Aileron
T2psi = feedback(Ltf(5,2),1); %psi/Rudder

% 
figure(1)
step(Go);
% RespuestaEscalon = stepinfo(Go);
% 
% Función de Sensibilidad
% S = feedback(eye(2),L,[1,2],[1,2]);
% Función KS
% KS = S*KHinf;
% Función de Sensibilidad Complementaria
T = feedback(L,eye(2),[1,2],[4,5]);
% 
% %Comprobaciones W1
% %Phi
% figure (2)
% bodemag(S,'r');
% hold on
% bodemag(1/W1Phi,'g');
% label('S','1/W1Phi')
% %Psi
% figure (3)
% bodemag(S,'r');
% hold on
% bodemag(1/W1Psi,'g');
% label('S','1/W1Psi')
% 
% %Comprobaciones W2
% %Phi
% figure (4)
% bodemag(KS,'r');
% hold on
% bodemag(1/W2Phi,'g');
% label('KS','1/W2Phi')
% %Psi
% figure (5)
% bodemag(KS,'r');
% hold on
% bodemag(1/W2Psi,'g');
% label('KS','1/W1Psi')
% 
%Comprobaciones W3
% %Phi
% figure ()
% bodemag(T(4,1),'r');
% hold on
% bodemag(T(4,2),'b');
% bodemag(1/W3Phi,'g');
% label('T_Aileron','T_Rudder','1/W3Phi')
% %Psi
% figure ()
% bodemag(T(5,1),'r');
% hold on
% bodemag(T(5,2),'b');
% bodemag(1/W3Psi,'g');
% label('T_Aileron','T_Rudder','1/W3Psi')
