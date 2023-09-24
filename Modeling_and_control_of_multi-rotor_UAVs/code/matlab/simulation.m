%% Drone simulation wrapper
% We use the control function on the system dynamics defined in
% drone_system, and we plot the results


clc
clear all
close all

syms a 'real'

%% Hovering with non zero tilting
% s_goal      = [0;0;0.5;pi/8;pi/8;0];
% ds_goal     = zeros(6,1);
% dds_goal    = zeros(6,1);
% ddds_goal   = zeros(6,1);

%% Tornado reference
% x_r = cos(a);
% y_r = sin(a);
% z_r = 0.5 + a/10;
% 
% ref = [x_r;y_r;z_r;0;0;0];
% d_ref = jacobian(ref, a);
% dd_ref = jacobian(d_ref, a);
% ddd_ref = jacobian(dd_ref, a);

%% Moving in diagonal
% s_goal      = [1;1;1;0;0;0];
% ds_goal     = zeros(6,1);
% dds_goal    = zeros(6,1);
% ddds_goal   = zeros(6,1);

%% Moving in diagonal
s_ref       = [1,0,0;1,2,0;1,0.7,0.5;0,0,0;0,0,0;0,0,0];
ds_goal     = zeros(6,1);
dds_goal    = zeros(6,1);
ddds_goal   = zeros(6,1);

Ts = 8;
Tf = 16;
%% Linear fast reference
% a_max = 6;
% v_max = a_max*2.5;
% sigma = [a_max*a^2/2, -a_max*(a-5)^2/2+v_max*5-v_max^2/(a_max), v_max*5-v_max^2/(a_max)];
% 
% x_r = sigma;
% y_r = zeros(1,3);
% z_r = 0.5*ones(1,3);
% 
% ref = [x_r;y_r;z_r;zeros(3,3)];
% 
% 
% d_ref = diff(ref, a);
% dd_ref = diff(d_ref, a);
% ddd_ref = diff(dd_ref, a);

%% Quintic fast ref
% Ts = 10;
% Tf = 16;
% 
% %sigma = [3*a^5/16000 - 3*a^4/320 + a^3/8 , 100];
% sigma = [3*a^5/500 - a^4*3/20 + a^3, 100];
% x_r = sigma;
% y_r = zeros(1,2);
% z_r = 0.5*ones(1,2);
% 
% ref = [x_r;y_r;z_r;0,0;0,0;0,0];
% 
% 
% d_ref = diff(ref, a);
% dd_ref = diff(d_ref, a);
% ddd_ref = diff(dd_ref, a);
% dddd_ref = diff(ddd_ref, a);

%% Simulation

T = 25; % Simulation time lenght in seconds
dt = 0.01; % time step in seconds

% Initialize the drone parameters
kf = 0.016;
drone.w     = ones(4,1)*1*9.81/(kf*4); 
drone.s     = [0;0;0.5;0;0;0]; % starts at 50 cm in height 
drone.ds    = zeros(6,1); 
drone.dds   = zeros(6,1); 
drone.Ib    = diag([0.015, 0.015, 0.025]);
drone.theta = 0; 
drone.phi   = 0; 
drone.m     = 1;
drone.l     = 0.3;

vel_and_pos_input_old = [drone.w; drone.phi; drone.theta];

s_space_history = zeros(6,T/dt);
ds_space_history = zeros(6,T/dt);
dds_space_history = zeros(6,T/dt);

ref_history     = zeros(3,T/dt);
dref_history     = zeros(3,T/dt);
ddref_history     = zeros(3,T/dt);

tilting_history = zeros(2,T/dt);
prop_history    = zeros(4,T/dt);

sat = false;

for i = (0:dt:T)
    %Uncomment below if using symbolic functions
%     a = i;
%     s_goal = eval(ref);
%     ds_goal = eval(d_ref);
%     dds_goal = eval(dd_ref);
%     ddds_goal = eval(ddd_ref);

    % Uncomment below if using bang bang functions
    if i < Ts
        indx = 1; 
    elseif i < Tf
        indx = 2;
    else 
        indx = 3;
    end
%     a = i;
%     s_goal = eval(ref(:,indx));
%     ds_goal = eval(d_ref(:,indx));
%     if sat == true
%         ddds_goal = zeros(6,1);
%     else
%         dds_goal = eval(dd_ref(:,indx));
%         ddds_goal = eval(ddd_ref(:,indx));
%     end

    % uncomment if using the linear dyagonal ref with mutliple points
    s_goal = s_ref(:,indx);

    acc_and_vel_input = control_function(s_goal, ds_goal, dds_goal, ddds_goal, drone);
    vel_and_pos_input = acc_and_vel_input*dt + vel_and_pos_input_old;
    vel_and_pos_input_old = vel_and_pos_input;
    space_acc = drone_system([vel_and_pos_input(1:4);acc_and_vel_input(5:6)], drone);

    drone.w     = vel_and_pos_input(1:4); 
    drone.s     = space_acc*dt^2/2 + drone.ds*dt+ drone.s;
    drone.ds    = space_acc*dt + drone.ds; 
    drone.dds   = space_acc; 

    drone.theta = vel_and_pos_input(6); 
    drone.phi   = vel_and_pos_input(5); 
    if abs(vel_and_pos_input(6)) >= pi/6
        drone.theta = pi/6*vel_and_pos_input(6)/abs(vel_and_pos_input(6));
        sat = true;
    else 
        sat = false;
    end
    if abs(vel_and_pos_input(5)) >= pi/6
        drone.phi   = pi/6*vel_and_pos_input(5)/abs(vel_and_pos_input(5)); 
        sat = true;
    else
        sat = false;
    end


    % storing info for plot
    s_space_history(:,round(i/dt)+1)    = drone.s;
    ds_space_history(:,round(i/dt)+1)   = drone.ds;
    dds_space_history(:,round(i/dt)+1)  = drone.dds;

    ref_history(:,round(i/dt)+1)        = s_goal(1:3);
    dref_history(:,round(i/dt)+1)       = ds_goal(1:3);
    ddref_history(:,round(i/dt)+1)      = dds_goal(1:3);

    tilting_history(:,round(i/dt)+1) = [drone.phi, drone.theta];
    prop_history(:,round(i/dt)+1) = drone.w;

end


figure Name 'Cartesian Position'
plot(0:dt:T, s_space_history(1,:), 'LineWidth', 2, 'Color', 'c')
hold on
plot(0:dt:T, s_space_history(2,:), 'LineWidth', 2)
plot(0:dt:T, s_space_history(3,:), 'LineWidth', 2)
plot(0:dt:T, ref_history(1,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, ref_history(2,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, ref_history(3,:),'LineStyle', '-.', 'Color', 'k')
grid on
ax = gca;
% ax.XAxis.Limits = [0 5];
% ax.YAxis.Limits = [-100 100];
xlabel('time [s]');
ylabel('position [m]');
legend('X','Y','Z', 'interpreter','latex')


figure Name 'Cartesian Velocities'
plot(0:dt:T, ds_space_history(1,:), 'LineWidth', 2, 'Color', 'c')
hold on
plot(0:dt:T, ds_space_history(2,:), 'LineWidth', 2)
plot(0:dt:T, ds_space_history(3,:), 'LineWidth', 2)
plot(0:dt:T, dref_history(1,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, dref_history(2,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, dref_history(3,:),'LineStyle', '-.', 'Color', 'k')
grid on
% ax = gca;
% ax.XAxis.Limits = [0 5];
% ax.YAxis.Limits = [-100 100];
xlabel('time [s]');
ylabel('velocity [m/s]');
legend('dX','dY','dZ', 'interpreter','latex')

figure Name 'Cartesian Accelerations'
plot(0:dt:T, dds_space_history(1,:), 'LineWidth', 2, 'Color', 'c')
hold on
plot(0:dt:T, dds_space_history(2,:), 'LineWidth', 2)
plot(0:dt:T, dds_space_history(3,:), 'LineWidth', 2)
plot(0:dt:T, ddref_history(1,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, ddref_history(2,:),'LineStyle', '-.', 'Color', 'k')
plot(0:dt:T, ddref_history(3,:),'LineStyle', '-.', 'Color', 'k')
grid on
% ax = gca;
% ax.XAxis.Limits = [0 5];
% ax.YAxis.Limits = [-100 100];
xlabel('time [s]');
ylabel('acceleration [m/s^2]');
legend('ddX','ddY','ddZ', 'interpreter','latex')

figure Name 'Cartesian Orientation'
plot(0:dt:T, s_space_history(4,:), 'LineWidth', 2)
hold on
plot(0:dt:T, s_space_history(5,:), 'LineWidth', 2)
plot(0:dt:T, s_space_history(6,:), 'LineWidth', 2)
grid on
% ax = gca;
% ax.XAxis.Limits = [0 5];
% ax.YAxis.Limits = [-100 100];
xlabel('time [s]');
ylabel('orientation [rad]');
legend('$\phi$','$\theta$','$\psi$', 'interpreter','latex')


figure Name 'Propeller velocities'
plot(0:dt:T, prop_history(1,:), 'LineWidth', 2)
hold on
plot(0:dt:T, prop_history(2,:), 'LineWidth', 2)
plot(0:dt:T, prop_history(3,:), 'LineWidth', 2)
plot(0:dt:T, prop_history(4,:), 'LineWidth', 2)
grid on
% ax = gca;
% ax.XAxis.Limits = [0 5];
% ax.YAxis.Limits = [-100 100];
xlabel('time [s]');
ylabel('velocity [rad/s]');
legend('1','2','3','4', 'interpreter','latex')


figure Name 'Tiliting angles'
plot(0:dt:T, tilting_history(1,:), 'LineWidth', 2)
hold on
plot(0:dt:T, tilting_history(2,:), 'LineWidth', 2)
grid on
xlabel('time [s]');
ylabel('tilting [rad]');
legend('$\phi$','$\theta$', 'interpreter','latex')


figure Name '3D Path'
plot3(s_space_history(1,:), s_space_history(2,:), s_space_history(3,:), 'LineWidth', 2, 'Color', 'c')
hold on
plot3(ref_history(1,:), ref_history(2,:), ref_history(3,:), '-.', 'Color', 'k')
axis equal
ax = gca;
%axis([0 100 -10 10 -10 10])
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on



%% Auxiliary functions

function Rx=Rx(theta)
    Rx = [ 1,          0,          0;
           0, cos(theta),-sin(theta); 
           0, sin(theta), cos(theta)];
end
function Ry=Ry(theta)
    Ry = [ cos(theta), 0, sin(theta);
                    0, 1,          0;
          -sin(theta), 0, cos(theta)];
end

function Rz=Rz(theta)
    Rz = [ cos(theta), -sin(theta), 0;
           sin(theta),  cos(theta), 0;
                   0 ,           0, 1];
end

function S=Skew(p)
    S =[    0, -p(3),  p(2);
         p(3),     0, -p(1);
        -p(2),  p(1),    0];
end

function dS = Skew_dot(p)
    dt = [ 0,  0,  0;
           0,  0, -1;
           0,  1,  0;
           0,  0,  1;
           0,  0,  0;
          -1,  0,  0;
           0, -1,  0;
           1,  0,  0;
           0,  0,  0];
    dp= [p(1), p(1),p(1),p(2), p(2),p(2), p(3), p(3),p(3)   ;
         p(1), p(1),p(1),p(2), p(2),p(2), p(3), p(3),p(3)   ;
         p(1), p(1),p(1),p(2), p(2),p(2), p(3), p(3),p(3) ]';
    
    dS=-dt'*dp;
end

function angles = rpy(R)
    sy = np.sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0));
    singular = sy < 1e-6;

    if  ~singular 
        x = np.arctan2(R(2,1) , R(2,2));
        y = np.arctan2(-R(2,0), sy);
        z = np.arctan2(R(1,0), R(0,0));
    else
        x = np.arctan2(-R(1,2), R(1,1));
        y = np.arctan2(-R(2,0), sy);
        z = 0;
    end

    angles = [x;y;z];
end