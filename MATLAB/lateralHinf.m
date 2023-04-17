load("modelo_lin.mat")
latmod;
latmod.A(4,4) = 0;
latmod.A(5,4) = 0;

W1 = makeweight(db2mag(23),[15 db2mag(0)],db2mag(-15)); %filtro pasa bajos
W2 = makeweight(db2mag(-28),[600 db2mag(-25)],db2mag(-5.2)); 
W3 = makeweight(db2mag(-20),[50 db2mag(0)],db2mag(20)); %filtro pasa altos

figure(1)
bodemag(W1,W2,W3)

%Peso de las perturbaciones
Wp      = makeweight(1,[10 0.1],0.01);
%Peso Error Ángulo Roll
W1Phi      = W1;
%Peso Error Ángulo Yaw
W1Psi      = W1;%makeweight(db2mag(23),[15 db2mag(0)],db2mag(-15));
%Peso Error velocidad de Roll (p)
W1p = W1;
%Peso Error velocidad de Yaw (r)
W1r = W1;
%Peso Señal de Control Aileron
W2Aileron      = W2;
%Peso Señal de Control Rudder
W2Rudder      = W2;%makeweight(db2mag(-28),[600 db2mag(-25)],db2mag(-5.2));
%Peso Salida Ángulo Roll
W3Phi      = W3;
%Peso Salida Ángulo Yaw
W3Psi      = W3;%makeweight(db2mag(-10),[50 db2mag(0)],db2mag(25));
%Peso Salida velocidad de Roll (p)
W3p = W3;
%Peso Salida velocidad de Yaw (r)
W3r = W3;
%Peso del ruido
Wn      = 0.001;
%Función Delta
%delta = ss([3 0; 0 1.5]);

systemnames     = 'latmod Wp Wn W1Phi W1Psi W1p W1r W2Aileron W2Rudder W3Phi W3Psi W3p W3r';
inputvar        = '[per; n; rPhi; rPsi; rP; rR; uAileron; uRudder]';
outputvar       = '[W1Phi; W1Psi; W1p; W1r; W2Aileron; W2Rudder; W3Phi; W3Psi; W3p; W3r; -latmod(2)-Wn+rP; -latmod(3)-Wn+rR; -latmod(4)-Wn+rPhi; -latmod(5)-Wn+rPsi]';
input_to_latmod   = '[-Wp + uAileron; -Wp + uRudder]';
input_to_Wn     = '[n]';
input_to_Wp     = '[per]';
input_to_W1Phi     = '[-latmod(4) - Wn + rPhi]';
input_to_W1Psi     = '[-latmod(5) - Wn + rPsi]';
input_to_W1p     = '[-latmod(2) - Wn + rP]';
input_to_W1r     = '[-latmod(3) - Wn + rR]';
input_to_W2Phi     = '[uAileron]';
input_to_W2Psi     = '[uRudder]';
input_to_W3Phi     = '[latmod(4)]';
input_to_W3Psi     = '[latmod(5)]';
input_to_W3p     = '[latmod(2)]';
input_to_W3r     = '[latmod(3)]';
%input_to_delta     = '[latmod(4); latmod(5)]';

%Planta generalizada
Planta = sysic;
%% Diseño controlador Hinf

ncont = 2; 
nmeas = 4; 
[K_LatDir,CL,gamma] = hinfsyn(Planta,nmeas,ncont);
%% Retroalimentación

%Función de Lazo abierto
L = latmod*K_LatDir;

%Sistema en Lazo cerrado
Go = feedback(L,eye(4),[1,2,3,4],[2,3,4,5]);
%% 

%Ancho de banda de los sistemas
BWPhi = bandwidth(Go(4,3));
BWPsi = bandwidth(Go(5,4));

% Respuesta al paso del sistema
figure(2)
step(Go(4,1));
RespuestaEscalon = stepinfo(Go(4,3));
figure(3)
step(Go(5,2));
RespuestaEscalon2 = stepinfo(Go(5,4));

%Función de Sensibilidad Complementaria
T = feedback(L,eye(4),[1,2,3,4],[2,3,4,5]);

% Función de Sensibilidad
%S = ones(5,4) - T;

% Función KS
% KS = S*K_LatDir;

% %Comprobaciones W1
% %Phi
% figure (4)
% sigma(S(4,1),'r');
% hold on
% sigma(KS(4,1),'g');
% sigma(T(4,1),'b');
% sigma(1/W1Phi,'r--');
% sigma(1/W2Aileron,'g--');
% sigma(1/W3Phi,'b--');
% legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
% title('Comprobación ángulo \phi')
% 
% %Psi
% figure (5)
% sigma(S(5,2),'r');
% hold on
% sigma(KS(5,2),'g');
% sigma(T(5,2),'b');
% sigma(1/W1Psi,'r--');
% sigma(1/W2Rudder,'g--');
% sigma(1/W3Psi,'b--');
% legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
% title('Comprobación ángulo \psi')
% 
% %Modelo con incertidumbre
% model2 = createUSSmodel(latmod,10);
% 
% L = model2*K_LatDir;
% 
% %Sistema en Lazo cerrado
% Go = feedback(L,eye(2),[1,2],[4,5]);
% 
% % Respuesta al paso del sistema
% figure(6)
% step(Go(4,1));
% RespuestaEscalon = stepinfo(Go(4,1));
% figure(7)
% step(Go(5,2));
% RespuestaEscalon2 = stepinfo(Go(5,2));
% 
% %Función de Sensibilidad Complementaria
% T = feedback(L,eye(2),[1,2],[4,5]);
% 
% % Función de Sensibilidad
% % S = feedback(eye(2),L,[1,2],[1,2]);
% S = ones(5,2) - T;
% 
% % Función KS
% KS = S*K_LatDir;
% 
% 
% %Comprobaciones W1
% %Phi
% figure (8)
% sigma(S(4,1),'r');
% hold on
% sigma(KS(4,1),'g');
% sigma(T(4,1),'b');
% sigma(1/W1Phi,'r--');
% sigma(1/W2Aileron,'g--');
% sigma(1/W3Phi,'b--');
% legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
% title('Comprobación ángulo \phi')
% 
% %Psi
% figure (9)
% sigma(S(5,2),'r');
% hold on
% sigma(KS(5,2),'g');
% sigma(T(5,2),'b');
% sigma(1/W1Psi,'r--');
% sigma(1/W2Rudder,'g--');
% sigma(1/W3Psi,'b--');
% legend({'S','KS','T','1/W_1','1/W_2','1/W_3'},'Location','southwest')
% title('Comprobación ángulo \psi')