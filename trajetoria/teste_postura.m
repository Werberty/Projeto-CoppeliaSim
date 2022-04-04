%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulador de Robo movel
%% Prof. Francisco Vanier de Andrade
%% IFCE - Campus Cedro
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
close
clc

 Ts = 0.03; % Tempo de amostragem
 t = 0:Ts:40; % Tempo de Simulacao
 xyRef = [3; 3]; % Posicao de Referencia
 q = [0; 0; 0]; % Estado Inicial 
 r=0.10;    %Raio das rodas
 L=0.15;    %Distancia entre eixos
 
 %FPB de velocidade
 ts=0.008;
 g1=tf(1,[ts 1]);
 g1d=c2d(g1,Ts);
 [num,den]=tfdata(g1d,'v');
 
 %Velocidades maximas e ganhos
 vmax=0.8;  
 wmax=pi/4;
 k1=.1;
 k2=.2;
 
 vast=0;
 x=q(1);
 y=q(2);
 phi=[];
 
 vr=[];
 wr=[];
 firef=[];
 
vrep=remApi('remoteApi'); 
vrep.simxFinish(-1); 
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
if (clientID>-1)
        disp('Conectado ao servidor API remoto');
        hd=0;
        he=0;
        [r]=vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
        [r,hrob]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
        [r,pos]=vrep.simxGetObjectPosition(clientID,hrob,-1,vrep.simx_opmode_streaming);
        x=pos(1);
        y=pos(2);
        z=pos(3);
        [r]=vrep.simxAddStatusbarMessage(clientID,'Vanier Andrade',vrep.simx_opmode_blocking)
        
        [r,hd]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_oneshot);
        [r,he]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_oneshot);
    
        
        for k = 1:length(t)
            phi_ref = atan2(xyRef(2)-q(2), xyRef(1)-q(1)); %Orientacao de Referencia
            qRef = [xyRef; phi_ref];

            e = qRef - q; % Erro de Posicao e orientacao

            % Controlador
             v = k1*sqrt(e(1)^2+e(2)^2);
             w = k2*e(3);

            %FPB
             vast=-den(2)*vast+num(2)*v;

        %  vast=v;
 
 % Physical constraints
 if abs(w)>wmax, w = wmax*sign(w); end
 if abs(v)>vmax, v = vmax*sign(v); end
%  if abs(vast)>vmax, vast = vmax*sign(vast); end

 wd=(v+w*L/2);
 we=(v-w*L/2);
 
%   [vd]=vrep.simxSetJointTargetVelocity(clientID,hd,wd,vrep.simx_opmode_oneshot);
%   [ve]=vrep.simxSetJointTargetVelocity(clientID,he,we,vrep.simx_opmode_oneshot);
%  
 vr=[vr vast];
 wr=[wr w];
 % Simulacaodo do movimento do Robo 
 dq = [v*cos(q(3)); v*sin(q(3)); w];
%  dq = [vast*cos(q(3)); vast*sin(q(3)); w];
 noise = 0.00; % Set to experiment with noise (e.g. 0.001)
 q = q + Ts*dq + randn(3,1)*noise; % Euler integration
 x=[x q(1)];
 y=[y q(2)];
 phi=[phi q(3)];
 firef=[firef phi_ref];
 [r,pos]=vrep.simxGetObjectPosition(clientID,hrob,-1,vrep.simx_opmode_buffer)
 vrep.simxSetObjectPosition(clientID,hrob,-1,[x(end) y(end) pos(3)],vrep.simx_opmode_oneshot);
 vrep.simxSetObjectOrientation(clientID,hrob,-1,[0 0 q(3)],vrep.simx_opmode_oneshot)
 end

       [vd]=vrep.simxSetJointTargetVelocity(clientID,hd,0,vrep.simx_opmode_blocking);
       [ve]=vrep.simxSetJointTargetVelocity(clientID,he,0,vrep.simx_opmode_blocking);
else
        disp('Falhou ao conectar ao servidor API remoto');
end
    [r]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)
    vrep.delete(); 
    
    disp('Programa finalizado');
 