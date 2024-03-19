function out=GPS(u,t,P)
    x_= u(1);
    y_= u(2); % Posiciones reales del quadrotor
    z_= u(3);
  
    persistent nu_n;
    persistent nu_e;
    persistent nu_d;
    
    if t == 0
        nu_n = 0;
        nu_e = 0; % Inicialización del error
        nu_d = 0;
    end
    
    y_gps_n = x_ + nu_n;
    y_gps_e = y_ + nu_e;  %	Datos del GPS
    y_gps_h = z_ - nu_d;   
    
    nu_n=randn*P.sigma_gps_n;
    nu_e=randn*P.sigma_gps_e; % error en las mediciones
    nu_d=randn*P.sigma_gps_d;

    
    out=[y_gps_n, y_gps_e, y_gps_h];
end