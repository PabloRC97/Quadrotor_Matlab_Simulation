P.tsim=30;      % Tiempo de simulación
P.gravity=9.81; % gravedad[m/s^2]

%% Parámetros del quadrotor 
P.Ix=0.014002764; % Momento de inercia sobre el eje x [kg.m2]
P.Iy=0.014267729; % Momento de inercia sobre el eje y [kg.m2]
P.Iz=0.029487252; % Momento de inercia sobre el eje z [kg.m2]
P.l= 0.2475;      % longitud de los brazos [m]
P.mass=1.645;     % masa [kg]
P.Kf= 2.4495e-5;  % Constante de fuerza aerodinámica
P.Km= 7.8833e-7;  % Constante de momento aerodinámico
P.Jr=2.66838e-4;  % Momento de inercia del motor


%% Condiciones iniciales
P.x0=5;
P.y0=0;
P.z0=0;
P.x0_dot=0;
P.y0_dot=0;
P.z0_dot=0;
P.phi0=0;
P.theta0=0;
P.psi0=0;
P.phi0_dot=0;
P.theta0_dot=0;
P.psi0_dot=0;

%% Sensores
P.sigma_accel = 0.0025;% g  Desviación estandar del error de medición
                       %   del acelerómetro ADXL325.
                       
P.sigma_gyro = 0.13*(pi/180); % rad/s Desviación estandar del error 
                       % de medición del giroscopio ADXRS450.
                       
P.sigma_mag=0.3*(pi/180);% rad  Desviación estandar del error 
                       % de medición del maegnetómetro Honeywell HMR3300
                       
P.Ts_sensor=0.01; % s   tiempo de muestreo del IMU

P.sigma_gps_n = 0.10;  % m
P.sigma_gps_e = 0.10; % m   errores en la medición del GPS
P.sigma_gps_d = 0.10; % m
P.Ts_gps = 0.2;       % s    tiempo de muestreo del GPS

P.Ts_est=0.01;        % s   tiempo de muestreo del EKF



