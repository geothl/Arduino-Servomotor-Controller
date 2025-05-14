
%clear
%delete(instrfind({'Port'},{'COM'}));

%a = arduino;


% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %

writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)


%system model 
km=242.6966;
tm=0.53;
kmeiot=1/36;
kt=0.003691851;
naumtacho=0.896;
k0=0.2366;
x0 = [2 0];
%A=[0 -1.7802;0 -1/0.53;];
%B=[0;1.6906;];
%C=[1 0];
%D=0;
A=[0 -k0*kmeiot/kt;0 -1/tm;];
B=[0;kt*km/tm;];
C=[1 0];
D=0;

%desired position
des_pos=5;

l1=8;
l2=8;
L=[l1+l2-1.8868;+1.8868*(l1+l2-1.8868)/1.7802-l1*l2/1.7802;];
Eig_Of_Estimator=eig(A-L*C);

K1=-10; 
K2=6;  %k2=4
K=[K1 K2];
E=eig(A-B*K);


%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND

V_7805=5.474;
vref_arduino=5.1;




x1ek=2; %maybe set it afterwards to 2
x2ek=0;
xdotek=[0;0;];

u=0;
ureal=0;

positionData = [];
pos_ek=[];
velocityData = [];
vel_ek=[];
eData = [];
timeData = [];
uData=[];
t=0;

% CLOSE ALL PREVIOUS FIGURES FROM SCREEN

close all

reseter(a);


% WAIT A KEY TO PROCEED
disp(['Connect cable from Arduino to Input Power Amplifier and then press enter to start controller']);
pause()



%START CLOCK
tic
 
 
while(t<7)     
position = readVoltage(a, 'A5'); % position
velocity = readVoltage(a,'A3'); % velocity

theta = 3 * vref_arduino * position / 5;
vtacho = -2 * (2 * velocity * vref_arduino / 5 - V_7805); %V7805 is offset  diavazo to -vtacho ara bazo plin gia na exo x2=vtacho

xdotek=A*[x1ek;x2ek;]+B*u+L*(theta-x1ek);

x1ek=x1ek+xdotek(1)*(toc-t);

x2ek=x2ek+xdotek(2)*(toc-t);


u = -K1*x1ek-K2*x2ek+K1*des_pos;
 
if abs(u) > 10
 	u = sign(u) * 10;
 end



if u > 0

    if(u<1.3) 
        u=1.3;
    end
	writePWMVoltage(a, 'D9', 0)
	writePWMVoltage(a, 'D6', abs(u) / 2)
else

    if(u>-1.3) 
        u=-1.3;
    end
    writePWMVoltage(a, 'D6', 0)
	writePWMVoltage(a, 'D9', abs(u) / 2)
end



t=toc;

    
timeData = [timeData t];
positionData = [positionData theta];
pos_ek=[pos_ek x1ek];
velocityData = [velocityData vtacho];
vel_ek=[vel_ek x2ek];
uData = [uData u];

end

% OUTPUT ZERO CONTROL SIGNAL TO STOP MOTOR  %
writePWMVoltage(a, 'D6', 0)
writePWMVoltage(a, 'D9', 0)

disp(['End of control Loop. Press enter to see diagramms']);
pause();

ref=5*ones(size(positionData));

figure
plot(timeData,positionData);
title('position')
xlabel('Time (s)') 
ylabel('x1 (V)'); 
figure
plot(timeData,velocityData);
title('velocity')
xlabel('Time (s)') 
ylabel('x2 (V)'); 
figure
plot(timeData,uData);
title('input control')
xlabel('Time (s)') 
ylabel('u (V)'); 
figure 
plot(timeData,positionData,timeData,ref);
legend({'y = x1','y = desired postion'},'Location','southeast')
figure 
plot(timeData,positionData,timeData,pos_ek);
title('x1: Position and estimated Position')
legend({'x1 = Position','x1est = Postion Estimation'},'Location','southeast')
figure 
plot(timeData,velocityData,timeData,vel_ek);
title('x2: Velocity and Velocity Estimation')
legend({'x2 = Velocity','x2est = Velocity Estimation'},'Location','southeast')

disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();