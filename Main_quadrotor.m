%% Main code to run simulation
clc;
clear all;
close all;
% Simulación en Simulink
[Tsim,X_sim,Ysim]=sim('Quadrotor_simulation'); 

length=(size(X_states(:,1)));
ref_0=zeros(1,length(1));

% Gráfica de la posición en x
figure(1);  
plot(X_states(:,13),X_states(:,1),'b');
hold on;
plot(X_states(:,13),EKF(:,1),'g');
plot(X_states(:,13),Traj(:,1),'r');
title('Posición en x');
xlabel('tiempo [s]');
ylabel('metros [m]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de la posición en y
figure(2)
plot(X_states(:,13),X_states(:,2),'b');
hold on;
plot(X_states(:,13),EKF(:,2),'g');
plot(X_states(:,13),Traj(:,2),'r');
title('Posición en y');
xlabel('tiempo [s]');
ylabel('metros [m]');
legend(' Actual','Estimada','y Deseada');
grid on;

% Gráfica de la posición en z
figure(3)
plot(X_states(:,13),X_states(:,3),'b');
hold on;
plot(X_states(:,13),EKF(:,3),'g');
plot(X_states(:,13),-Traj(:,3),'r');
title('Posición en z');
xlabel('tiempo [s]');
ylabel('metros [m]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de la velocidad en x
figure(4)
plot(X_states(:,13),X_states(:,4),'b');
hold on;
plot(X_states(:,13),EKF(:,4),'g');
plot(X_states(:,13),Traj(:,4),'r');
title('Velocidad en x');
xlabel('tiempo[s]');
ylabel('velocidad [m/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de la velocidad en y
figure(5)
plot(X_states(:,13),X_states(:,5),'b');
hold on;
plot(X_states(:,13),EKF(:,5),'g');
plot(X_states(:,13),Traj(:,5),'r');
title('Velocidad en y');
xlabel('tiempo [s]');
ylabel('velocidad [m/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de la velocidad en z
figure(6)
plot(X_states(:,13),X_states(:,6),'b');
hold on;
plot(X_states(:,13),EKF(:,6),'g');
plot(X_states(:,13),-Traj(:,6),'r');
title('Velocidad en z');
xlabel('tiempo [s]');
ylabel('velocidad [m/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de phi
figure(7)
plot(X_states(:,13),X_states(:,7)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,7)*(180/pi),'g');
plot(X_states(:,13),des_ang(:,1)*(180/pi),'r');
title('Orientación angular roll (\phi)');
xlabel('tiempo [s]');
ylabel('grados[°]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de theta
figure(8)
plot(X_states(:,13),X_states(:,8)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,8)*(180/pi),'g');
plot(X_states(:,13),des_ang(:,2)*(180/pi),'r');
title('Orientación angular pitch (\theta)');
xlabel('tiempo [s]');
ylabel('grados[°]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de psi
figure(9)
plot(X_states(:,13),X_states(:,9)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,9)*(180/pi),'g');
plot(X_states(:,13),ref_0,'r');
title('Orientación angular yaw (\psi)');
xlabel('tiempo [s]');
ylabel('grados [°]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de phi_dot
figure(10)
plot(X_states(:,13),X_states(:,10)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,10)*(180/pi),'g');
plot(X_states(:,13),ref_0,'r');
title('velocidad angular de \phi (p)');
xlabel('tiempo [s]');
ylabel(' vel angular [°/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de theta_dot
figure(11)
plot(X_states(:,13),X_states(:,11)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,11)*(180/pi),'g');
plot(X_states(:,13),ref_0,'r');
title('Velocidad angular de \theta (q)');
xlabel('tiempo [s]');
ylabel(' vel angular [°/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica de psi_dot
figure(12)  
plot(X_states(:,13),X_states(:,12)*(180/pi),'b');
hold on;
plot(X_states(:,13),EKF(:,12)*(180/pi),'g');
plot(X_states(:,13),ref_0,'r');
title('Velocidad angular de \psi (r)');
xlabel('tiempo [s]');
ylabel('vel angular [°/s]');
legend(' Actual','Estimada','Deseada');
grid on;

% Gráfica del error en el seguimiento de la trayectoria
figure(13)
subplot(2,2,1)
plot(X_states(:,13),Traj(:,1)-X_states(:,1),'r');
hold on;
title('Error en x');
xlabel('time [s]');
ylabel('error [m]');
grid on;

subplot(2,2,2)
plot(X_states(:,13),Traj(:,2)-X_states(:,2),'r');
hold on;
title('Error en y');
xlabel('time [s]');
ylabel('error [m]');
grid on;

subplot(2,2,[3,4]) 
plot(X_states(:,13),-Traj(:,3)-X_states(:,3),'r');
hold on;
title('Error en z');
xlabel('time [s]');
ylabel('error [m]');
grid on;

% Gráfica de las señales de control
figure(14)
subplot(2,2,1)
plot(X_states(:,13),Control_out(:,1),'b');
hold on;
axis([0 30 15 18]);
title('Fuerza (U1)');
xlabel('time [s]');
ylabel('Newtons [N]');
grid on;

subplot(2,2,2)
plot(X_states(:,13),Control_out(:,2),'b');
hold on;
title('Torque \phi (U2)');
xlabel('time [s]');
ylabel('Newtons metro [Nm]');
grid on;

subplot(2,2,3)
plot(X_states(:,13),Control_out(:,3),'b');
hold on;
title('Torque \theta (U3)');
xlabel('time [s]');
ylabel('Newtons metro [Nm]');
grid on;

subplot(2,2,4)
plot(X_states(:,13),Control_out(:,4),'b');
hold on;
title('Torque \psi (U4)');
xlabel('time [s]');
ylabel('Newtons metro [Nm]');
grid on;

% Trayectoria helicoidal en 3D
figure(15)  
plot3(Traj(:,1),Traj(:,2),Traj(:,3),'r')
hold on;
plot3(X_states(:,1),X_states(:,2),-X_states(:,3),'b','LineWidth',1)

Quadrotor_draw([X_states(length(1),1),X_states(length(1),2),X_states(length(1),3),X_states(length(1),4),...
    X_states(length(1),5),X_states(length(1),6),X_states(length(1),7),X_states(length(1),8),...
    X_states(length(1),9),X_states(length(1),10),X_states(length(1),11),X_states(length(1),12),0]);
title('Trayectoria');
legend('Ideal','Real');
grid on;


