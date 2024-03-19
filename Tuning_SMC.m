%clear all
%global lambda_y_O k_y_O 
%global k1_theta_O k2_theta_O lambda_theta_O;
% k1_theta_O=8;
% k2_theta_O=2;
% lambda_theta_O=15;  
% lambda_phi=28.8420;%tunning using LQR
% k_phi=2.4729;   %tunning using LQR
global lambda_z_O k1_z_O k2_z_O k3_z_O k4_z_O

    lambda_z_O=7.1;
    k1_z_O=3.1;
    k2_z_O=0.3;
    k3_z_O=1.5;
    k4_z_O=0.5;
    
    
x0=[k1_z_O;k2_z_O;k3_z_O;k4_z_O;lambda_z_O];  
x=fminsearch('Optimal_SMC',x0);
k1_z_O=x(1);
k2_z_O=x(2);
k3_z_O=x(3);
k4_z_O=x(4);
lambda_z_O=x(5); 

%%Tunning z
% lambda_z_O= 5; 
% k_z_O=10;

% lambda_phi_O= 20; 
% k_phi_O=50;  
