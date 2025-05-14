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
A=[0 k0*kmeiot/kt;0 1/tm;];
B=[0;kt*km/tm;];
C=[1 0];
D=0;

K1=-10; %k1=-9 theory
K2=4; %k2=4.2 theory
KI=-15; %ki=-9 theory
K=[K1 K2];
%K=place(A,B,[-10 -15]);
%K1=K(1);
%K2=K(2);
E=eig(A-B*K);

%inverse=inv(A-B*K);
%kr1=-C*inverse*B;
%kr=1/kr1;   %kr=k1 wow
%krr=5*kr;


% Set the desired position
des_pos =5;

%   The input setpoint is in Volts and can vary from 0 to 10 Volts because the position pot is refered to GND

V_7805=5.474;
vref_arduino=5.1;






positionData = [];
velocityData = [];
eData = [];
timeData = [];
uData=[];
zdotData=[];
Z=0;
Zf=[];
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
vtacho = -2 * (2 * velocity * vref_arduino / 5 - V_7805); %V7805 is offset
x1=theta;
x2=vtacho;


zdotData=[zdotData theta-des_pos];
Z = Z+(theta-des_pos)*(toc-t); %it works for not constant sampling rate
u = -K1*theta-K2*vtacho-KI*(Z);
Zf=[Zf Z] ;
if abs(u) > 10
 	u = sign(u) * 10;
 end



if u > 0
%    analogWrite(a,6,0);
%    analogWrite(a,9,min(round(e/2*255/ Vref_arduino) , 255));
   if(u<1.4) 
        u=1.4;
    end
	writePWMVoltage(a, 'D9', 0)
	writePWMVoltage(a, 'D6', abs(u) / 2)
else
%    analogWrite(a,9,0);
%   analogWrite(a,6,min(round(-e/2*255/ Vref_arduino) , 255));
  if(u>-1.4) 
        u=-1.4;
    end
    writePWMVoltage(a, 'D6', 0)
	writePWMVoltage(a, 'D9', abs(u) / 2)
end

t=toc;

    
timeData = [timeData t];  %mhpos prepei na to pao poio pano? giati xronos jekina apo 0,17 sec kai oxi apo to mhden. den jero an ginetai kiolas

positionData = [positionData x1];
velocityData = [velocityData x2];
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
plot(timeData,Zf);
title('z')

disp('Disonnect cable from Arduino to Input Power Amplifier and then press enter to stop controller');
pause();
