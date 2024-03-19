function des_states=Trajectory(t,P)
    z_max=4;   % altura máxima
    r=5;       % radio de la trayectoria
    T =P.tsim; % tiempo de vuelo

    t0      = 0;
    tf      = T;
    M       = ... % Matriz M
       [1    t0  t0^2    t0^3    t0^4    t0^5;
        0    1   2*t0    3*t0^2  4*t0^3  5*t0^4;
        0    0   2       6*t0    12*t0^2 20*t0^3;
        1    tf  tf^2    tf^3    tf^4    tf^5;
        0    1   2*tf    3*tf^2  4*tf^3  5*tf^4;
        0    0   2       6*tf    12*tf^2 20*tf^3];
    
    b = ...      % Matriz b
       [0    0;
        0    0;
        0    0;
        4*pi z_max;
        0    0;
        0    0];
    
    a       = M\b; % Matriz a
    
    % Polinomios de posiciones
    u     = a(1,1) + a(2,1)*t + a(3,1)*t^2 + a(4,1)*t^3 ...
            + a(5,1)*t^4 + a(6,1)*t^5;
    u_z     = a(1,2) + a(2,2)*t + a(3,2)*t^2 + a(4,2)*t^3 ... 
            + a(5,2)*t^4 + a(6,2)*t^5;
    % Polinomios de velocidades
    u_der= a(2,1) + 2*a(3,1)*t + 3*a(4,1)*t^2 ...
            + 4*a(5,1)*t^3 + 5*a(6,1)*t^4;
    u_z_der= a(2,2) + 2*a(3,2)*t + 3*a(4,2)*t^2 ...
            + 4*a(5,2)*t^3 + 5*a(6,2)*t^4;
    % Polinomios de aceleraciones
    u_dder  = 2*a(3,1) + 6*a(4,1)*t + 12*a(5,1)*t^2 + 20*a(6,1)*t^3;
    u_z_dder= 2*a(3,2) + 6*a(4,2)*t + 12*a(5,2)*t^2 + 20*a(6,2)*t^3; 

    % Posiciones
    x       = cos(u)*r;
    y       = sin(u)*r;
    pos     = [x; y; u_z];
    % Velocidades
    xd      = -y*u_der;
    yd      =  x*u_der;
    vel     = [xd; yd; u_z_der];
    % Acceleraciones
    xdd     = -x*u_der^2 - y*u_dder;
    ydd     = -y*u_der^2 + x*u_dder;
    acc     = [xdd; ydd; u_z_dder];

    des_states= [pos', vel',acc'];
           
end

