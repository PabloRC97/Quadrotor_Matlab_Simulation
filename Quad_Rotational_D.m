function [sys,x0,str,ts,simStateCompliance] = Quad_Rotational_D(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 6;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 6;
sizes.NumInputs      = 7;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
        P.phi0;
        P.theta0;
        P.psi0;
        P.phi0_dot;
        P.theta0_dot;
        P.psi0_dot;
        ];

%
% str is always an empty matrix
%
str = [];
          
%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu,P)
    phi=        x(1);
    theta=      x(2);
    psi=        x(3); %% Estados de quadrotor
    phi_dot=    x(4); 
    theta_dot=  x(5);
    psi_dot=    x(6);

Ix=P.Ix;            % Momento de inercia sobre el eje x [kg.m2]
Iy=P.Iy;            % Momento de inercia sobre el eje y [kg.m2]
Iz=P.Iz;            % Momento de inercia sobre el eje z [kg.m2]
Jr=P.Jr;            % Momento de inercia del motor
g =P.gravity;       % gravedad
l=P.l;              % longitud de los brazos [m]

T_phi= uu(1); %U2
T_theta=uu(2);%U3    %% Señales de control (Torques)
T_psi=uu(3);  %U4

Omega_r= uu(4);  % velocidad relativa

 T_phi_dist=uu(5);
 T_theta_dist=uu(6); % Torques externos causados por el viento
 T_psi_dist=uu(7);
 
%% Ecuaciones de la dinámica rotacional del quadrotor
phi_ddot=  ((Iy-Iz)/Ix)*theta_dot*psi_dot  -(Jr/Ix)*theta_dot*Omega_r + ...
            (l/Ix)*( T_phi) + T_phi_dist;
        
theta_ddot=((Iz-Ix)/Iy)*phi_dot*psi_dot +(Jr/Iy)*phi_dot*Omega_r + ...
            (l/Iy)*(T_theta)+ T_theta_dist;
        
psi_ddot=((Ix-Iy)/Iz)*phi_dot*theta_dot+ (1/Iz)*(T_psi)+ T_psi_dist;

sys = [phi_dot;theta_dot;psi_dot;phi_ddot;theta_ddot;psi_ddot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate