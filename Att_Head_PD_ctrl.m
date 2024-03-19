function out=Att_Head_PD_ctrl(x_states,des_Ang,a,P)
    phi=x_states(7);
    theta=x_states(8);
    psi=x_states(9);       %% Estados retroalimentados
    phi_dot=x_states(10);
    theta_dot=x_states(11);
    psi_dot=x_states(12);
    
    phi_des=des_Ang(1);   % Angulo calculado por el control de posicion PD
    theta_des=des_Ang(2); % Angulo calculado por el control de posicion PD
    psi_des=0;            % Angulo de direccion deseado

    phid_des=0;
    thetad_des=0; % Velocidades angulares deseadas
    psid_des=0;

    Kp_phi=10.3; Kd_phi=0.99; % Ganancias para controlar ángulo phi
    Kp_theta=10.3; Kd_theta=0.99; % Ganancias para controlar ángulo theta
    Kp_psi=5.1; Kd_psi=0.56;  % Ganancias para controlar ángulo psi      
    
    % Señales de control PD (torques)  U= kp(e) + Kd(e_dot)
    U2= Kp_phi*( phi_des - phi)+ Kd_phi*(phid_des-phi_dot);
    U3= Kp_theta*( theta_des - theta) + Kd_theta*(thetad_des-theta_dot);
    U4= Kp_psi*( psi_des - psi) + Kd_psi*(psid_des-psi_dot);
    
out=[U2;U3;U4];
end