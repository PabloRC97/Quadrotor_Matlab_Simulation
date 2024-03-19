function out=Position_MST_SMC(des_states,x_states,x2_x,x2_y,x2_z,t,P)

    mass=P.mass;
    g=P.gravity;
    x_des= des_states(1);
    y_des= des_states(2);   %% Posiciones deseadas
    z_des= -des_states(3);
    
    xd_des= des_states(4);
    yd_des= des_states(5);  %% Velocidades deseadas
    zd_des= -des_states(6);
    
    xdd_des= des_states(7);
    ydd_des= des_states(8); %% Aceleraciones deseadas
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

    dist_x=0;
    lambda_z=4.9;
    k1_z=4.1;
    k2_z=1.99;          % Parámetros de control MST-SMC para F
    k3_z=0.2;
    k4_z=0.1;
    
    error_z= z_des-z_;
    error_d_z= zd_des- z_dot; 
    % superficie deslizante s_z
    s_z= error_d_z + lambda_z*error_z;
    u_z=-(k1_z*sqrt(abs(s_z))*sign(s_z) +k2_z*s_z)+ x2_z;
    x2_z_dot=-(k3_z*sign(s_z)+ k4_z*s_z );
    % Señal de control F
    F=(mass/cos(theta)*cos(phi))*(g-zdd_des-lambda_z*error_d_z+u_z);

    lambda_x=1.2;
    k1_x=0.95;
    k2_x=0.95;    %% Parámetros de control MST-SMC para control virtual Ux
    k3_x=0.03;
    k4_x=0.04;

    error_x= x_des-x_;
    error_d_x= xd_des- x_dot; 
    % superficie deslizante sx
    s_x= error_d_x + lambda_x*error_x;
    
    lambda_y=1.2;
    k1_y=0.95;
    k2_y=0.95;   %% Parámetros de control MST-SMC para control virtual Uy
    k3_y=0.03;
    k4_y=0.04;
     
    error_y= y_des-y_;
    error_d_y= yd_des- y_dot; 
    % superficie deslizante sy
    s_y= error_d_y + lambda_y*error_y;
  

    %% Controles virtuales Ux y Uy
    Vu_x= k1_x*sqrt(abs(s_x))*sign(s_x) +k2_x*s_x - x2_x;
    x2_x_dot= -(k3_x*sign(s_x)+ k4_x*s_x) + dist_x ;
    % Señal de control virtual Ux
    Ux=(mass/F)*(xdd_des+ lambda_x*error_d_x +Vu_x);
    
    Vu_y= k1_y*sqrt(abs(s_y))*sign(s_y) + k2_y*s_y - x2_y;
    x2_y_dot=-(k3_y*sign(s_y) + k4_y*s_y);
    % Señal de control virtual Ux
    Uy=(mass/F)*(ydd_des + lambda_y*error_d_y +Vu_y);
    
    % Calculo de ángulos deseados
    phi_des= -sin(psi)*Ux + cos(psi)*Uy;
    theta_des= -cos(psi)*Ux - sin(psi)*Uy;

    out=[F,phi_des,theta_des,x2_x_dot, x2_y_dot,x2_z_dot];
end

