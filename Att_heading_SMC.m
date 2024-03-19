function Torque=Att_heading_SMC(x_states,des_Ang,Omega_r,t,a,P)

    persistent  T_phi T_theta T_psi
    persistent t_prev cont
    
    if t==0
        t_prev=5;
        Omega_r=0; % inicializacion de la velocidad relativa
        cont=0;
    end
  
    
    Ix=P.Ix; % momento de inercia sobre el eje x
    Iy=P.Iy; % momento de inercia sobre el eje y
    Iz=P.Iz; % momento de inercia sobre el eje z
    Jr=P.Jr; % momento de inercia del motor
    l=P.l;   % longitud de brazo
    
    if (t_prev ~= t) % Condición para prevenir que el control se ejecute
                     % mas veces de lo inicado 
                     
        phi=x_states(7);
        theta=x_states(8);
        psi=x_states(9);
        phi_dot= x_states(10);   %% Estados del quadrotor
        theta_dot= x_states(11);
        psi_dot= x_states(12);

        phi_des=des_Ang(1);    % angulo phi deseado
        theta_des= des_Ang(2); % angulo theta deseado
        psi_des=0;             % angulo psi deseado

        phid_des=0;
        thetad_des=0; %% velocidades angulares deseadas 
        psid_des=0;

        phidd_des=0;
        thetadd_des=0; %% aceleraciones angulares deseadas
        psidd_des=0;
   
        lambda_phi=15.42;
        k_phi=29.97;     %% parametros de control SMC para U2
        epsilon_phi=0.7; 

        lambda_theta=15.42;
        k_theta=29.97;     %% parametros de control SMC para U3
        epsilon_theta=0.7;

         lambda_psi=15.8;
         k_psi=18.3;      %% parametros de control SMC para U4
         epsilon_psi=0.7;
         
        %% Señales de control SMC
        error_phi= phi_des- phi;
        error_d_phi= phid_des- phi_dot;
        % superficie deslizante s_phi
        s_phi=error_d_phi + lambda_phi*error_phi; 
        % función de saturación 
        sat_phi= s_phi/(abs(s_phi)+epsilon_phi);
        Usw_phi=k_phi*sat_phi; 
        % señal de control U2
        T_phi=(Ix/l)*(phidd_des-((Iy-Iz)/Ix)*theta_dot*psi_dot+ ...
            (Jr/Ix)*theta_dot*Omega_r +  lambda_phi*error_d_phi+ Usw_phi);
        
        error_theta= theta_des- theta;
        error_d_theta= thetad_des- theta_dot;
        % superficie deslizante s_theta
        s_theta=error_d_theta + lambda_theta*error_theta;
        % función de saturación 
        sat_theta= s_theta/(abs(s_theta)+epsilon_theta);
        Usw_theta=k_theta*sat_theta;
        % señal de control U3
        T_theta=(Iy/l)*(thetadd_des-((Iz-Ix)/Iy)*phi_dot*psi_dot - ...
            (Jr/Iy)*phi_dot*Omega_r + lambda_theta*error_d_theta+ Usw_theta);
       
        error_psi= psi_des- psi;
        error_d_psi= psid_des- psi_dot;
        % superficie deslizante s_phi
        s_psi=error_d_psi + lambda_psi*error_psi;
        % funcion de saturación 
        sat_psi= s_psi/(abs(s_psi)+epsilon_psi);
        Usw_psi=k_psi*sat_psi;
        % señal de control U4
        T_psi=Iz*(psidd_des-((Ix-Iy)/Iz)*theta_dot*phi_dot + ...
            lambda_psi*error_d_psi +Usw_psi);
   
        t_prev=t;
        cont=cont+1;
    else 
    end
    
    Torque=[T_phi,T_theta,T_psi];
end