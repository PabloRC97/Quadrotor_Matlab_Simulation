function [J]=Optimal_PD(x)
%global Kp_phi_O Kd_phi_O
global Kp_y_O Kd_y_O 

Kp_y_O= x(1);
Kd_y_O= x(2);
% Kp_z_O= x(1);
% Kd_z_O= x(2);

% Kp_phi_O= x(1);
% Kd_phi_O= x(2);


%Kf= 2.4495e-5; %3.13e-5; % Aerodinamic force constant 
%Km= 7.8833e-7;  %7.5e-7; % Aerodinamic moment constant 
%v_max= 435;%rad/s

[Tsim,Xsim,Ysim]=sim('Quad_Sim_final');
Q=10; R=0.1;

%errorPhi=(Ysim(:,2)-Ysim(:,1));
error=(Ysim(:,6)-Ysim(:,5));
%errorPsi= (Ysim(:,6)-Ysim(:,5));
%error=[errorPhi,errorTheta,errorPsi];
F=Ysim(:,8);
%if U3
% U3=Ysim(:,8);
% U4=Ysim(:,9);
% F=Ysim(:,10);



    J=trapz(Q*error.*error + R*F.*F);


end

%%
% function [J]=Optimal_PD(x)
% global Kp Kd 
% 
% Kp=x(1);
% Kd=x(2);
% 
% 
% [Tsim,Xsim,Ysim]=sim('Quad_Sim_v5');
% Q=1000; R=0.001;
% error=(Ysim(:,2)-Ysim(:,1));
% U=Ysim(:,3);
% %J=trapz(Q*error.*error+ R*U.*U)*0.01;
% J=trapz(Q*error.*error+ R*U.*U)*1;
% end

%Q=40; R=0.1; _PHI