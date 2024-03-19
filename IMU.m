function out=IMU(rot,t,P)
g=P.gravity;
phi=rot(1);
theta=rot(2);
psi=rot(3);      % valores reales  de la dinamica rotacional
phi_dot=rot(4);  % del quadrotor
theta_dot=rot(5);
psi_dot=rot(6);

%% Accelerómetro
y_accel_x= g*sin(theta)+ randn*P.sigma_accel;
y_accel_y= - g*cos(theta)*sin(phi)+ rand*P.sigma_accel;
y_accel_z= -g*cos(theta)*cos(phi) + randn*P.sigma_accel;

% Calculo de lo ángulos phi y theta
phi_hat = atan(y_accel_y/y_accel_z); 
theta_hat = asin(y_accel_x/g);

% Giroscopio
y_gyro_x = phi_dot+randn*P.sigma_gyro;
y_gyro_y = theta_dot+randn*P.sigma_gyro;
y_gyro_z = psi_dot+randn*P.sigma_gyro;

   phi_dot_hat = y_gyro_x;
   theta_dot_hat = y_gyro_y;
   psi_dot_hat = y_gyro_z;

% Magnetómetro
magn= psi+ randn*P.sigma_mag;
psi_hat= magn; % ángulo psi

out=[phi_hat+1.1e-4,theta_hat,psi_hat, phi_dot_hat, ...
    theta_dot_hat,psi_dot_hat];
end