function [J]=Optimal_SMC(x)
%global lambda_y_O k_y_O 
%global k1_theta_O k2_theta_O lambda_theta_O;
global lambda_z_O k1_z_O k2_z_O k3_z_O k4_z_O
k1_z_O=x(1);
k2_z_O=x(2);
k3_z_O=x(3);
k4_z_O=x(4);
lambda_z_O=x(5);

% Kf= 2.4495e-5; %3.13e-5; % Aerodinamic force constant 
% Km= 7.8833e-7;  %7.5e-7; % Aerodinamic moment constant 
% v_max= 435;%rad/s

[Tsim,Xsim,Ysim]=sim('Quad_Sim_final_v3');
Q=100; R=0.01;
error=(Ysim(:,2)-Ysim(:,1));
F=Ysim(:,3);

%U2=Ysim(:,12);
% U3=Ysim(:,8);
% U4=Ysim(:,9);
 %Uy=Ysim(:,15);
% Omega1=sqrt(  (1/(4*Kf))*F +  (1/(2*Kf))*U3 + (1/(4*Km))*U4 );
% Omega2=sqrt(  (1/(4*Kf))*F -  (1/(2*Kf))*U2 - (1/(4*Km))*U4 );
% Omega3=sqrt(  (1/(4*Kf))*F -  (1/(2*Kf))*U3 + (1/(4*Km))*U4 );
% Omega4=sqrt(  (1/(4*Kf))*F +  (1/(2*Kf))*U2 - (1/(4*Km))*U4 );
%J=trapz(Q*error.*error+ R*U.*U)*0.01;

% if ( isempty(find(Omega1>=v_max)) && isempty(find(Omega2>=v_max)) && isempty(find(Omega3>=v_max)) ...
%    && isempty(find(Omega4>=v_max)) && isempty(find(Omega1<=0))&& isempty(find(Omega2<=0))...
%    && isempty(find(Omega3<=0)) && isempty(find(Omega4<=0)) )

    J=trapz(Q*error.*error+ R*F.*F);
% else
%     J=100;
% end
end