global Kp_y_O Kd_y_O
%global Kp_phi_O Kd_phi_O 
% Kp_phi_O=7;
% Kd_phi_O= 1;
Kp_y_O=2;
Kd_y_O= 2;

x0=[Kp_y_O;Kd_y_O];
%x0=[Kp_phi_O;Kd_phi_O];
x=fminsearch('Optimal_PD',x0);

% Kp_phi_O  = x(1);
% Kd_phi_O =  x(2);
Kp_y_O  = x(1);
Kd_y_O =  x(2);


% global Kp Kd 
% 
% format short e
%Kp_z_O=15;
%Kd_z_O= 5;
% x0=[Kp;Kd];
% x=fminsearch('Optimal_PD',x0);
% Kp=x(1);
% Kd=x(2);
