function out=Position_SMC(des_states,x_states,P)
    mass=P.mass; % masa
    g=P.gravity; % gravedad

    x_des= des_states(1);
    y_des= des_states(2);    %% Posiciones deseadas
    z_des= -des_states(3);
    
    xd_des= des_states(4);
    yd_des= des_states(5);   %% Velocidades deseadas
    zd_des= -des_states(6);
    
    xdd_des= des_states(7);
    ydd_des= des_states(8);  %% Aceleraciones deseadas
    zdd_des= -des_states(9);

    x_= x_states(1);
    y_= x_states(2);
    z_= x_states(3);
    x_dot= x_states(4);
    y_dot= x_states(5);     %% Estados del quadrotor
    z_dot= x_states(6);
    phi=x_states(7);
    theta=x_states(8);
    psi= x_states(9);

    lambda_z= 9.21;
    k_z=11.1;           % Parámetros de control SMC para F
    delta_z=0.7;

    error_z= z_des-z_;
    error_d_z= zd_des- z_dot;
    % superficie deslizante s_z
    s_z= error_d_z + lambda_z*error_z;
    % función de saturación
    sat_z= s_z/(abs(s_z)+delta_z);
    Usw_z=-k_z*sat_z;   
    % Señal de control F
    F=(mass/cos(theta)*cos(phi))*(g-zdd_des-lambda_z*error_d_z+ Usw_z );
    
    lambda_x=2.5;
    k_x=4.5;      %% Parámetros de control SMC para control virtual Ux
    delta_x=0.7;

    lambda_y=2.5;
    k_y=4.5;      %% Parámetros de control SMC para control virtual Uy
    delta_y=0.7;
    
    
    %% Controles virtuales Ux y Uy
    error_x= x_des-x_;
    error_d_x= xd_des- x_dot;      
    % superficie deslizante sx
    s_x= error_d_x + lambda_x*error_x;
    % función de saturación
    sat_x= s_x/(abs(s_x)+delta_x);
    Usw_x=k_x*sat_x;
    % Señal de control virtual Ux
    Ux=(mass/F)*(xdd_des + lambda_x*error_d_x+ Usw_x );

    
    error_y= y_des-y_;
    error_d_y= yd_des- y_dot; 
    % superficie deslizante sy
    s_y= error_d_y + lambda_y*error_y;
    % función de saturación
    sat_y= s_y/(abs(s_y)+delta_y);
    Usw_y=k_y*sat_y;
    % Señal de control virtual Uy
    Uy=(mass/F)*(ydd_des + lambda_y*error_d_y+ Usw_y );
   
    % Calculo de ángulos deseados
    phi_des= -sin(psi)*Ux + cos(psi)*Uy;
    theta_des= -cos(psi)*Ux - sin(psi)*Uy;
    
    out=[F,phi_des,theta_des];
end