function Quad_states_plot(uu)
%
% modified 12/11/2009 - RB

    % process inputs to function
    x =   uu(1);             % North position (meters)
    y =   uu(2);             % East position (meters)
    z = -uu(3);            % altitude (meters)
    u   = uu(4);             % body velocity along x-axis (meters/s)
    v   = uu(5);             % body velocity along y-axis (meters/s)
    w   = -uu(6);             % body velocity along z-axis (meters/s)
    phi   = uu(7)*(180/pi);      % roll angle (degrees)   
    theta = uu(8)*(180/pi);      % pitch angle (degrees)
    psi   = uu(9)*(180/pi);             % yaw angle (degrees)
    p     = uu(10)*(180/pi);     % body angular rate along x-axis (degrees/s)
    q     = uu(11)*(180/pi);     % body angular rate along y-axis (degrees/s)
    r     = uu(12)*(180/pi);     % body angular rate along z-axis (degrees/s)
    x_des = uu(13);            % commanded North position (meters)
    y_des = uu(14);            % commanded East position (meters)
    z_des = uu(15);            % commanded altitude (meters)
    u_des = uu(16);            
    v_des = uu(17);            
    w_des = uu(18);
    phi_des = uu(19)*(180/pi);
    theta_des = uu(20)*(180/pi);
    phi_hat = uu(21)*(180/pi);
    theta_hat = uu(22)*(180/pi);
    psi_hat = uu(23)*(180/pi);
    p_hat = uu(24);
    q_hat = uu(25);
    r_hat = uu(26);
    t      = uu(27);         
   
    psi_des=0;
    % define persistent variables 
    persistent x_handle
    persistent y_handle
    persistent z_handle
    persistent u_handle
    persistent v_handle
    persistent w_handle
    persistent phi_handle
    persistent theta_handle
    persistent psi_handle
    persistent p_handle
    persistent q_handle
    persistent r_handle
    
  % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(2), clf

        subplot(2,3,1)
        hold on
        grid on
        x_handle = graph_y_yd(t, x, x_des, 'x', []);
        
        subplot(2,3,2)
        hold on
        grid on
        y_handle = graph_y_yd(t, y, y_des, 'y', []);

        subplot(2,3,3)
        hold on
        grid on
        z_handle = graph_y_yd(t, z, z_des, 'z', []);

        subplot(2,3,4)
        hold on
        grid on
        u_handle = graph_y_yd(t, u, u_des, 'u', []);

        subplot(2,3,5)
        hold on
        grid on
        v_handle = graph_y_yd(t, v, v_des, 'v', []);

        subplot(2,3,6)
        hold on
        grid on
        w_handle = graph_y_yd(t, w, w_des, 'w', []);

        figure(3), clf
       
        subplot(2,3,1)
        hold on
        grid on
        phi_handle = graph_y_yhat_yd(t, phi, phi_hat, phi_des, '\phi', []);
        
        subplot(2,3,2)
        hold on
        grid on
        theta_handle = graph_y_yhat_yd(t, theta, theta_hat, theta_des, '\theta', []);
        
        subplot(2,3,3)
        hold on
        grid on
        psi_handle = graph_y_yhat_yd(t, psi, psi_hat, psi_des, '\psi', []);
        
        subplot(2,3,4)
        hold on
        grid on
        p_handle = graph_y_yd(t, p, p_hat, 'p', []);
        
        subplot(2,3,5)
        hold on
        grid on
        q_handle = graph_y_yd(t, q, q_hat, 'q', []);
        
        subplot(2,3,6)
        hold on
        grid on
        r_handle = graph_y_yd(t, r, r_hat, 'r', []);

        
    % at every other time step, redraw state variables
    else 
        graph_y_yd(t, x, x_des, 'x', x_handle);        
        graph_y_yd(t, y, y_des, 'y', y_handle);
        graph_y_yd(t, z, z_des, 'z', z_handle);
        graph_y_yd(t, u, u_des, 'u', u_handle);
        graph_y_yd(t, v, v_des, 'v', v_handle);
        graph_y_yd(t, w, w_des, 'w', w_handle);
        graph_y_yhat_yd(t, phi, phi_hat, phi_des, '\phi', phi_handle);
        graph_y_yhat_yd(t, theta, theta_hat, theta_des, '\theta', theta_handle);
        graph_y_yhat_yd(t, psi, psi_hat, psi_des, '\psi', psi_handle);
        graph_y_yd(t, p, p_hat, 'p', p_handle);
        graph_y_yd(t, q, q_hat, 'q', q_handle);
        graph_y_yd(t, r, r_hat, 'r', r_handle);

    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% graph y and yd with lable mylabel
function handle = graph_y_yd(t, y, yd, lab, handle)
  
  if isempty(handle),
    handle(1)    = plot(t,y,'b');          
    handle(2)    = plot(t,yd,'r--');
    ylabel(lab)
    set(get(gca, 'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yd]);
    %drawnow
  end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plot the variable y in blue, its estimated value yhat in green, and its 
% desired value yd in red, lab is the label on the graph
function handle = graph_y_yhat_yd(t, y, yhat, yd, lab, handle)
  if isempty(handle),
    handle(1)   = plot(t,y,'b');
    handle(2)   = plot(t,yhat,'g--');
    handle(3)   = plot(t,yd,'r-.');
    ylabel(lab)
    set(get(gca,'YLabel'),'Rotation',0.0);
  else
    set(handle(1),'Xdata',[get(handle(1),'Xdata'),t]);
    set(handle(1),'Ydata',[get(handle(1),'Ydata'),y]);
    set(handle(2),'Xdata',[get(handle(2),'Xdata'),t]);
    set(handle(2),'Ydata',[get(handle(2),'Ydata'),yhat]);   
    set(handle(3),'Xdata',[get(handle(3),'Xdata'),t]);
    set(handle(3),'Ydata',[get(handle(3),'Ydata'),yd]);     
    %drawnow
  end

