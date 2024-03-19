function out=Position_PD_Ctrl(des_states,x_states,P)
    mass=P.mass; % masa 
    g=P.gravity; % gravedad
    
    x_=x_states(1);
    y_=x_states(2);
    z_=x_states(3);
    x_dot=x_states(4); %% Estados realimentados del quadrotor
    y_dot=x_states(5);
    z_dot=x_states(6);
    psi=x_states(9);
    
    x_des=des_states(1);      
    y_des=des_states(2);    %% Posiciones deseadas
    z_des=-des_states(3);   
    
    xd_des=des_states(4);    
    yd_des=des_states(5);   %% Velocidades deseadas
    zd_des=-des_states(6);  
   
    xdd_desR=des_states(7); 
    ydd_desR=des_states(8); %% Aceleraciones deseadas
    zdd_desR=-des_states(9);
     
    % Ganancias para los controles de posición
    Kp_z=17.3237;  Kd_z=6.3548;
    Kp_x=2.965; Kd_x=3.13; 
    Kp_y=2.965; Kd_y=3.13;
    
    % Fuerza de empuje 
    F= mass*(g  -(zdd_desR + Kp_z*(z_des - z_)+ Kd_z*(zd_des - z_dot)));
    
    % Aceleraciones calculadas
    xdd_des= xdd_desR + Kp_x*( x_des - x_)+ Kd_x*(xd_des-x_dot);
    ydd_des= ydd_desR + Kp_y*( y_des - y_ )+ Kd_y*(yd_des-y_dot); 
    
    % Calculo de  ángulos phi y theta deseados
    phi_des= (mass/F)* (-xdd_des*sin(psi)+ydd_des*cos(psi));    
    theta_des= (mass/F)* (-xdd_des*cos(psi)-ydd_des*sin(psi));

    out=[F, phi_des, theta_des];
end