load("modelo_lin.mat")
latmod;
latmod.A(4,4) = 0;
latmod.A(5,4) = 0;

W1 = makeweight(db2mag(60),[15 db2mag(0)],db2mag(-10)); 
W2 = makeweight(db2mag(-50),[800 db2mag(-45)],db2mag(-40)); 
W3 = makeweight(db2mag(-10),[800 db2mag(0)],db2mag(35));
figure()
bodemag(W1,W2,W3)

%Peso de las perturbaciones
Wp      = makeweight(1,[10 0.1],0.01);
%Peso Error Ángulo Roll
W1Phi      = W1;
%Peso Error Ángulo Yaw
W1Psi      = W1;
%Peso Señal de Control Aileron
W2Aileron      = W2;
%Peso Señal de Control Rudder
W2Rudder      = W2;
%Peso Salida Ángulo Roll
W3Phi      = W3;
%Peso Salida Ángulo Yaw
W3Psi      = W3;
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

%Ancho de banda de los sistemas
BWPhi = bandwidth(Go(4,1));
BWPsi = bandwidth(Go(5,2));

% Respuesta al paso del sistema
figure()
step(Go);
RespuestaEscalon = stepinfo(Go);

%Función de Sensibilidad Complementaria
T = feedback(L,eye(2),[1,2],[4,5]);

% Función de Sensibilidad
% S = feedback(eye(2),L,[1,2],[1,2]);
S = ones(5,2) - T;

% Función KS
KS = S*KHinf;

%Comprobaciones W1
%Phi
figure ()
sigma(S(4,1),'r');
hold on
sigma(KS(4,1),'g');
sigma(T(4,1),'b');
sigma(1/W1Phi,'r--');
sigma(1/W2Aileron,'g--');
sigma(1/W3Phi,'b--');
legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
title('Comprobación ángulo \phi')

%Psi
figure ()
sigma(S(5,2),'r');
hold on
sigma(KS(5,2),'g');
sigma(T(5,2),'b');
sigma(1/W1Psi,'r--');
sigma(1/W2Rudder,'g--');
sigma(1/W3Psi,'b--');
legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
title('Comprobación ángulo \psi')