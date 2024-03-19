function Torque=Att_heading_MST_SMC(x_states,des_Ang,Omega_r,t,a,x2_phi,x2_theta,x2_psi,P)
    persistent u_phi x2_phi_dot
    persistent  u_theta x2_theta_dot 
    persistent  u_psi x2_psi_dot
    persistent  T_phi T_theta T_psi
    persistent t_prev
    
    if(t==0)
        Omega_r=0;   % inicializacion de la velocidad relativa
        x2_phi_dot= 0;
        x2_theta_dot= 0;
        x2_psi_dot= 0;
        t_prev=5;
    end
    
    Ix=P.Ix; % momento de inercia sobre el eje x
    Iy=P.Iy; % momento de inercia sobre el eje y
    Iz=P.Iz; % momento de inercia sobre el eje z
    Jr=P.Jr; % momento de inercia del motor
    l=P.l;   % longitud de brazo
     
    phi=x_states(7);
    theta=x_states(8);
    psi=x_states(9);
    phi_dot= x_states(10);    %% Estados del quadrotor
    theta_dot= x_states(11);
    psi_dot= x_states(12);

    phi_des=des_Ang(1);       % angulo phi deseado
    theta_des= des_Ang(2);    % angulo theta deseado  
    psi_des=0;                % angulo psi deseado

    phid_des=0;
    thetad_des=0;       %% velocidades angulares deseadas 
    psid_des=0;

    phidd_des=0;
    thetadd_des=0;      %% aceleraciones angulares deseadas
    psidd_des=0;
   
    if (t_prev ~= t) % Condición para prevenir que el control se ejecute
                     % mas veces de lo inicado 
        dist_theta=0;

        k1_phi=4.9;
        k2_phi=4.1;
        k3_phi=6.5;         %% parametros de control MST-SMC para U2
        k4_phi=1.7;
        lambda_phi=15.65;
                
        error_phi= phi_des- phi;
        error_d_phi= phid_des- phi_dot;
        % superficie deslizante s_phi
        s_phi=error_d_phi + lambda_phi*error_phi;
        
        k1_theta=4.9;
        k2_theta=4.1;
        k3_theta=6.5;       %% parametros de control MST-SMC para U3
        k4_theta=1.7;
        lambda_theta=15.65;
        
        error_theta= theta_des- theta;
        error_d_theta= thetad_des- theta_dot;
        % superficie deslizante s_phi
        s_theta=error_d_theta + lambda_theta*error_theta;

        k1_psi=4;
        k2_psi=5.5;
        k3_psi=4;           %% parametros de control MST-SMC para U4
        k4_psi=1.5;
        lambda_psi=15.71;
        
        error_psi= psi_des- psi;
        error_d_psi= psid_des- psi_dot;
        % superficie deslizante s_psi
        s_psi=error_d_psi + lambda_psi*error_psi;

        %% Señales de control MST-SMC
        
        u_phi=k1_phi*sqrt( abs(s_phi) )*sign(s_phi)+ k2_phi*s_phi+ x2_phi;
        % Señal de control U2
        T_phi=(Ix/l)*(phidd_des-((Iy-Iz)/Ix)*theta_dot*psi_dot + ...
        lambda_phi*error_d_phi + (Jr/Ix)*theta_dot*Omega_r + u_phi);
        x2_phi_dot= k3_phi*sign(s_phi)+k4_phi*s_phi;

        u_theta= k1_theta*sqrt( abs(s_theta) )*sign(s_theta) + ...
        k2_theta*s_theta - x2_theta;
        % Señal de control U3
        T_theta=(Iy/l)*(thetadd_des-((Iz-Ix)/Iy)*phi_dot*psi_dot + ...
        lambda_theta*error_d_theta - (Jr/Iy)*phi_dot*Omega_r + u_theta);
        x2_theta_dot= -k3_theta*sign(s_theta)-k4_theta*s_theta   + dist_theta;
 
        u_psi= k1_psi*sqrt( abs(s_psi) )*sign(s_psi)+k2_psi*s_psi+ x2_psi;
        % Señal de control U4
        T_psi= Iz*(psidd_des-((Ix-Iy)/Iz)*theta_dot*phi_dot + ...
        lambda_psi*error_d_psi + u_psi);
        x2_psi_dot= k3_psi*sign(s_psi) + k4_psi*s_psi;

        t_prev=t;
        
    else 
    end
    Torque=[T_phi,T_theta,T_psi,x2_phi_dot,x2_theta_dot,x2_psi_dot];
end