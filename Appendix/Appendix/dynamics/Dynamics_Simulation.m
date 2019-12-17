close all; clc

g = 9.801; % gravity constant

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%  link properties
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link 1
l1 = 0.22; % length [m]
d1 = 0.143685775; % mass center
m1 = 0.233955; % mass [kg]
% moment of inertia
I1xx = 0.000037133; I1xy = 1.254*10^-6; I1xz = 3.18*10^-7;
I1yx = 1.254*10^-6; I1yy = 0.000719463; I1yz = -9*10^-9;
I1zx = 3.18*10^-7; I1zy = -9*10^-9; I1zz = 0.000703759;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link 2
l2 = 0.27; % length [m]
d2 = 0.125792094; % mass center
m2 = 0.340330406; % mass [kg]
% moment of inertia  
I2xx = 0.002058408; I2xy = -0.000037385; I2xz = -0.000090846;
I2yx = -0.000037385; I2yy = 0.002055589; I2yz = -0.000916918;
I2zx = -0.000090846; I2zy = -0.000916918; I2zz = 0.001711125;

%% %%%%%%%%%%%%%%%%%%% discrete time
T = 5; % second
N = 620; % resolution
i = 0; 
for t = linspace(0, T, N)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% instantaneous time
    i = i + 1; time(i) = t;
    %%%%%%%%%%%%%%% Joint 1: angular displacement, velocity and acceleration
    theta__1(i) = (3/40)*pi*t^2-(1/100)*pi*t^3;
    theta__dot1(i) = (3/20)*pi*t-(3/100)*pi*t^2;  
    theta__ddot1(i) = (3/20)*pi-(3/50)*pi*t;
    
    %%%%%%%%%%%%%%% Joint 2: angular displacement, velocity and acceleration
    theta__2(i) = (3/50)*pi*t^2-(1/125)*pi*t^3;
    theta__dot2(i) = (3/25)*pi*t-(3/125)*pi*t^2; 
    theta__ddot2(i) = (3/25)*pi-(6/125)*pi*t; 
    
    %%%%%%%%%%%%%%% Joint 3: angular displacement, velocityand acceleration
    theta__3(i) = (3/80)*pi*t^2-(1/200)*pi*t^3; 
    theta__dot3(i) = (3/40)*pi*t-(3/200)*pi*t^2;
    theta__ddot3(i) = (3/40)*pi-(3/100)*pi*t;
%% mass Matrics
 H11 = (1/2)*(2*I1zz+2*I2zz+d2.^2*m2+l1.^2*m2+l1.^2*m2*cos(2*theta__2(i) ...
  )+2*d2*l1*m2*cos(theta__3(i))+d2.^2*m2*cos(2*(theta__2(i)+theta__3(i)))+2* ...
  d2*l1*m2*cos(2*theta__2(i)+theta__3(i)));
 H12 = (1/2)*((-1)*(I1yz+I1zy+I2yz+ ...
  I2zy)*cos(theta__1(i))+(I1xz+I1zx+I2xz+I2zx)*sin(theta__1(i)));
 H13 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta__1(i))+(I2xz+I2zx)*sin(theta__1(i)));

 H21 = (1/2)*((-1)*(I1yz+I1zy+I2yz+I2zy)*cos(theta__1(i)) ...
     +(I1xz+I1zx+I2xz+I2zx)*sin(theta__1(i)));
 H22 = (1/2)*(I1xx+I1yy+I2xx+I2yy+2*d2.^2*m2+2*l1.^2*m2+((-1)* ...
  I1xx+I1yy+(-1)*I2xx+I2yy)*cos(2*theta__1(i))+4*d2*l1*m2*cos(theta__3(i) ...
  )+(-1)*I1xy*sin(2*theta__1(i))+(-1)*I1yx*sin(2*theta__1(i))+(-1)*I2xy* ...
  sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));
 H23 = (1/2)*(I2xx+I2yy+2* ...
  d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta__1(i))+2*d2*l1*m2*cos(theta__3(i)) ...
  +(-1)*I2xy*sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));

 H31 = (1/2)*((-1)*(I2yz+I2zy)*cos(theta__1(i))+(I2xz+I2zx)*sin(theta__1(i)));
 H32 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2*theta__1(i))+2*d2* ...
  l1*m2*cos(theta__3(i))+(-1)*I2xy*sin(2*theta__1(i))+(-1)*I2yx*sin(2*theta__1(i)));
 H33 = (1/2)*(I2xx+I2yy+2*d2.^2*m2+((-1)*I2xx+I2yy)*cos(2* ...
  theta__1(i))+(-1)*(I2xy+I2yx)*sin(2*theta__1(i)));

 C11 = (-2)*m2*(l1*cos(theta__2(i))+d2*cos(theta__2(i)+theta__3(i)))*((l1*sin(theta__2(i)) ...
  +d2*sin(theta__2(i)+theta__3(i)))*theta__dot2(i)+d2*sin(theta__2(i)+theta__3(i)) ...
  *theta__dot3(i));
 C12 = (1/2)*((I1xy+I1yx+I2xy+I2yx)*cos(2*theta__1(i) ...
  )+((-1)*I1xx+I1yy+(-1)*I2xx+I2yy)*sin(2*theta__1(i)))*theta__dot2(i);
 C13 = (1/2)*((I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I2xx+I2yy)* ...
  sin(2*theta__1(i)))*(2*theta__dot2(i)+theta__dot3(i));

 C21 = (1/2)*((I1xz+I1zx+I2xz+I2zx)*cos(theta__1(i))+(I1yz+I1zy+I2yz+I2zy)* ...
  sin(theta__1(i))+m2*(l1.^2*sin(2*theta__2(i))+d2.^2*sin(2*(theta__2(i)+theta__3(i)))+ ...
  2*d2*l1*sin(2*theta__2(i)+theta__3(i))))*theta__dot1(i)+(-1)*(( ...
  I1xy+I1yx+I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I1xx+I1yy+(-1)*I2xx+ ...
  I2yy)*sin(2*theta__1(i)))*theta__dot2(i)+(-1)*((I2xy+I2yx)* ...
  cos(2*theta__1(i))+((-1)*I2xx+I2yy)*sin(2*theta__1(i)))*theta__dot3(i);
 C22 = (-2)*d2*l1*m2*sin(theta__3(i))*theta__dot3(i);
 C23 = (-1)*d2*l1*m2*sin(theta__3(i))*theta__dot3(i);

 C31 = (1/2)*((I2xz+I2zx)*cos( ...
  theta__1(i))+(I2yz+I2zy)*sin(theta__1(i))+2*d2*m2*(l1*cos(theta__2(i))+d2*cos( ...
  theta__2(i)+theta__3(i)))*sin(theta__2(i)+theta__3(i)))*theta__dot1(i)+(-1)*(( ...
  I2xy+I2yx)*cos(2*theta__1(i))+((-1)*I2xx+I2yy)*sin(2*theta__1(i)))*( ...
  theta__dot2(i)+theta__dot3(i));
 C32 = d2*l1*m2*sin(theta__3(i))*theta__dot2(i);
 C33 = 0;
 
 F1 = 0.25*2*(1/(1 + exp(-2*theta__dot1(i))) - 0.5) + (0.005*(theta__dot1(i))); 
 F2 = 0.685*2*(1/(1 + exp(-2*theta__dot2(i))) - 0.5) + (0.02*(theta__dot2(i)));
 F3 = 0.39*2*(1/(1 + exp(-5*theta__dot3(i))) - 0.5) + (0.008*(theta__dot3(i)));

G1 = 0;
G2 = g*((d1*m1+l1*m2)*cos(theta__2(i))+d2*m2*cos(theta__2(i)+theta__3(i)));
G3 = d2*g*m2*cos(theta__2(i)+theta__3(i));        



%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  link positions
    %%%%%%%%%%%%%%%%%% link 1
    X1(i) = l1*cos(theta__1(i))*cos(theta__2(i));
    Y1(i) = l1*sin(theta__1(i))*cos(theta__2(i));
    Z1(i) = l1*sin(theta__2(i));
   % x1(i) = 0.5*X1(i); y1(i) = 0.5*Y1(i); 
    %%%%%%%%%%%%%%%%%% link 2
    X2(i) = X1(i) + l2*cos(theta__2(i) + theta__3(i))*cos(theta__1(i)); 
    Y2(i) = Y1(i) + l2*cos(theta__2(i) + theta__3(i))*sin(theta__1(i));
    Z2(i) = Z1(i) + l2*sin(theta__2(i) + theta__3(i));
    %x2(i) = X1(i) + 0.5*X2(i); %Y2(i) = Y1(i) + 0.5*Y2(i);

%% Dynamics Matrics
M = [H11, H12 , H13; H21 , H22 , H23; H31 , H32 ,H33];
V = [C11*theta__dot1(i)+C12*theta__dot2(i)+C13*theta__dot3(i);C21*theta__dot1(i)+C22*theta__dot2(i)+C23*theta__dot3(i);C31*theta__dot1(i)+C32*theta__dot2(i)+C33*theta__dot3(i)];
G = [G1 ; G2 ; G3];
Theta__ddot = [theta__ddot1(i);theta__ddot2(i);theta__ddot3(i)];
F = [F1;F2;F3];

beta =[C11 C12 C13; C21 C22 C23; C31 C32 C33]+[G1;G2;G3];


%% Torque
tauVector = M*Theta__ddot+beta;
tau1(i) = tauVector(1);
tau2(i) = tauVector(2);
tau3(i) = tauVector(3);

Tau_test = [0 0 0;1 0 0;0 0 0];

%% Acceleration 
acceleration = M\(tauVector-beta);
acc1(i) = acceleration(1);
acc2(i) = acceleration(2);
acc3(i) = acceleration(3);

end
time1 = 0:0.008:619*0.008;

figure(1)
clf
figure(1)
subplot(3, 1, 1)
hold on
plot(time, theta__1, 'b')
plot(time, theta__2, 'r')
plot(time, theta__3, 'g')
hold off
legend('theta 1', 'theta 2', 'theta 3')
grid on; 
xlabel('time [sec]'); ylabel('angular displacement [rad]'); 
subplot(3, 1, 2)
hold on
plot(time, theta__dot1, 'b')
plot(time, theta__dot2, 'r')
plot(time, theta__dot3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular velocity [rad/s]'); 
subplot(3, 1, 3)
hold on
plot(time, theta__ddot1, 'b')
plot(time, theta__ddot2, 'r')
plot(time, theta__ddot3, 'g')
hold off
grid on; 
xlabel('time [sec]'); ylabel('angular acceleration [rad/s^2]'); 

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% motor torque
figure(2)
clf
figure(2)
hold on
title('Torque of the joints')
plot(time, tau1, 'b')
plot(time, tau2, 'r')
plot(time, tau3, 'g')
hold off
legend('theta 1', 'theta 2', 'theta 3')
grid on; 
xlabel('Time [s]'); ylabel('torque [Nm]'); 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% motor acceleration
figure(3)
clf
figure(3)
hold on
plot(time, acc1, 'b')
plot(time, acc2, 'r')
plot(time, acc3, 'g')
hold off
legend(' joint 1', 'joint 2', 'joint 3')
grid on; 
xlabel('time [sec]'); ylabel('acc [RAD/s^2]'); 
