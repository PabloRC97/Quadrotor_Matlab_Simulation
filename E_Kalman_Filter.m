function out=E_Kalman_Filter(sensors,input,t,P)
    persistent x_hat cont  u P_c 

    phi_hat_s= sensors(1);
    theta_hat_s= sensors(2);
    psi_hat_s= sensors(3);   % Lecturas del IMU
    p_hat_s= sensors(4);
    q_hat_s= sensors(5);
    r_hat_s= sensors(6);
    
    F=input(1);
    T_phi=input(2);
    T_theta=input(3);       % Lectura de las señales de control
    T_psi=input(4);
 
    mass= P.mass;   % masa
    l=P.l;          % longitud de brazo
    Ix=P.Ix;        % momento de inercia sobre el eje x
    Iy=P.Iy;        % momento de inercia sobre el eje y
    Iz=P.Iz;        % momento de inercia sobre el eje z
    g=P.gravity;    % gravedad
    T_est=P.Ts_est; % tiempo de muestreo del filtro 
    
    if t == 0
        % Inicialización de contador que indica cuando el GPS esta
        % disponible
        cont=0; 
        % Inicializacion del vector de estados con las condiciones iniciales
        x_hat=[...
            P.x0;
            P.y0;
            P.z0;
            P.x0_dot;
            P.y0_dot;   
            P.z0_dot;
            P.phi0;
            P.theta0;
            P.psi0;
            P.phi0_dot; 
            P.theta0_dot;
            P.psi0_dot];    
        % Inicialización de las señales de control
        u=[mass*g;0;0;0];
    else 
       u=[...
        F;
        T_phi;
        T_theta; % Señales de control
        T_psi];
    end
  
    Q=eye(12);
    R=eye(6); 
    
    % Matriz de covarianza del error de estimación
    P_c= diag([0.0003*1.5, 0.0003*1.5, 0.0005*1.5, 0.0095*1.5,...
        0.0095*1.5, 0.0099*1.5, (10*pi/180)/480, (10*pi/180)/480,...
        (5*pi/180)/5000,0.0000001,0.0000001,0.0000001]*0.2);
    
     % Matriz de covarianza del error del proceso wQw^T
    w=[0.0009*1.5, 0.0009*1.5, 0.0009*1.5, 0.0093*1.5, 0.0093*1.5, ...
        0.0099*1.5,(10*pi/180)/480,(10*pi/180)/480,(5*pi/180)/5000,...
        0.00000001,0.00000001,0.00000001]*0.3; 
    
    % Matriz de covarianza del error de las mediciones vRv^T
    v=[P.sigma_gps_n, P.sigma_gps_e, P.sigma_gps_d,...
    P.sigma_accel, P.sigma_accel, P.sigma_mag];

%% Etapa de predicción o de propagación
    N=10;
    for i=1:N
        x_dot=x_hat(4);
        y_dot=x_hat(5);
        z_dot=x_hat(6);
        phi_hat=x_hat(7);
        theta_hat=x_hat(8);
        psi_hat=x_hat(9);
        p_hat=x_hat(10);
        q_hat=x_hat(11);
        r_hat=x_hat(12);
        
        % Ecuaciones del modelo para las estimaciones
        fun=[...
            x_dot;
            y_dot;
            z_dot;
            -(sin(phi_hat)*sin(psi_hat) + ...
            cos(phi_hat)*sin(theta_hat)*cos(psi_hat))*(u(1)/mass);
            -(-sin(phi_hat)*cos(psi_hat) +...
            cos(phi_hat)*sin(theta_hat)*sin(psi_hat))*(u(1)/mass);
            g-(cos(theta_hat)*cos(phi_hat))*(u(1)/mass);
            p_hat;
            q_hat;
            r_hat;
            (l/Ix)*u(2);
            (l/Iy)*u(3);
            (1/Iz)*u(4)];
        
        % Calculo de la matriz de derivadas parciales A
        a1=(sin(phi_hat)*sin(theta_hat)*cos(psi_hat) - ...
            cos(phi_hat)*sin(psi_hat))*(u(1)/mass);
        a2=-(cos(phi_hat)*cos(theta_hat)*cos(psi_hat))*(u(1)/mass);
        a3= -(sin(phi_hat)*cos(psi_hat)- ...
            cos(phi_hat)*sin(theta_hat)*sin(psi_hat))*(u(1)/mass);
        a4=(cos(phi_hat)*cos(psi_hat) + ...
            sin(phi_hat)*sin(theta_hat)*sin(psi_hat))*(u(1)/mass);
        a5= -(cos(phi_hat)*cos(theta_hat)*sin(psi_hat))*(u(1)/mass);
        a6= -(sin(phi_hat)*sin(psi_hat) + ...
            cos(phi_hat)*sin(theta_hat)*cos(psi_hat))*(u(1)/mass);
        a7= (sin(phi_hat)*cos(theta_hat))*(u(1)/mass);
        a8= (cos(phi_hat)*sin(theta_hat))*(u(1)/mass);
% 
        A= [...
            0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, a1, a2, a3, 0, 0, 0;
            0, 0, 0, 0, 0, 0, a4, a5, a6, 0, 0, 0;
            0, 0, 0, 0, 0, 0, a7, a8, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        
        % Prediccion de los estados
        x_hat= x_hat + (T_est/N)*fun;
        % Prediccion de la matriz de covarianza de errores de estimación 
        P_c= P_c + (T_est/N)*(A*P_c + P_c*A' +  w.*Q.*w');
    end
    
    %% Etapa de actualización de las mediciones 
    if cont==20 % Si las lecturas del GPS estan disponibles
        x_gps=sensors(7);
        y_gps=sensors(8);  % Lecturas del GPS
        z_gps=sensors(9);
        
        H=[...
        1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;  % Matriz de predicción
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;  % de mediciones  H*x_hat
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0];
        
        % Vector de medición de los sensores
        zz=[x_gps,y_gps,z_gps,phi_hat_s,theta_hat_s,psi_hat_s]';
        
        % Calculo de la ganancia de Kalman
        Kk= (P_c*H')/(H*P_c*H' + v.*R.*v');
        % Actualización de la matriz de los errores de estimación
        P_c=(eye(12)- Kk*H)*P_c;
        % Actualización de los estados
        x_hat= x_hat+ Kk*(zz- H*x_hat);
        cont=0; 
    end
    cont=cont+1;
    out=x_hat; % Vector de estados como salida del filtro
end




