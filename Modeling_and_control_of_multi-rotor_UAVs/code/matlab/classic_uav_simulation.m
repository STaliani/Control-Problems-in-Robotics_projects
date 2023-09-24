%% Drone simulation wrapper
% We use the control function on the system dynamics defined in
% drone_system, and we plot the results


clc
clear all
close all

syms a 'real'


%% Tornado reference
% x_r = cos(a);
% y_r = sin(a);
% z_r = 0.5 + a/10;
% 
% ref = [x_r;y_r;z_r;0;0;0];
% d_ref = jacobian(ref, a);
% dd_ref = jacobian(d_ref, a);
% ddd_ref = jacobian(dd_ref, a);
% dddd_ref = jacobian(ddd_ref, a);

%% Moving in diagonal
% s_goal      = [1;1;1;0;0;0];
% ds_goal     = zeros(6,1);
% dds_goal    = zeros(6,1);
% ddds_goal   = zeros(6,1);
% dddds_goal  = zeros(6,1);

%% Linear fast reference
% a_max = 8;
% v_max = a_max*2.5;
% sigma = [a_max*a^2/2, -a_max*(a-5)^2/2+v_max*5-v_max^2/(a_max)];
% 
% x_r = sigma;
% y_r = zeros(1,2);
% z_r = 0.5*ones(1,2);
% 
% ref = [x_r;y_r;z_r;0.1992,-0.1992;0.1992,-0.1992;zeros(1,2)];
% 
% 
% d_ref = diff(ref, a);
% dd_ref = diff(d_ref, a);
% ddd_ref = diff(dd_ref, a);
% dddd_ref = diff(ddd_ref, a);

%% Quintic fast ref
% a_max = 6;
Ts = 10;
Tf = 16;

%sigma = [3*a^5/16000 - 3*a^4/320 + a^3/8 , 100];
sigma = [3*a^5/500 - a^4*3/20 + a^3, 100];
x_r = sigma;
y_r = zeros(1,2);
z_r = 0.5*ones(1,2);

ref = [x_r;y_r;z_r;0,0;0,0;0,0];


d_ref = diff(ref, a);
dd_ref = diff(d_ref, a);
ddd_ref = diff(dd_ref, a);
dddd_ref = diff(ddd_ref, a);


%% Simulation

T = 15; % Simulation time lenght in seconds
dt = 0.01; % time step in seconds

% Initialize the drone parameters
kf = 0.016;
km = 0.0027;
drone.T     = 9.81;
drone.dT    = 0;
drone.w     = ones(4,1)*1*9.81/(kf*4);
drone.s     = [0;0;0.5;0;0;0]; % starts at 50 cm in height 
drone.ds    = zeros(6,1); 
drone.dds   = zeros(6,1); 
drone.ddds  = zeros(6,1); 
drone.Ib    = diag([0.015, 0.015, 0.025]);
drone.theta = 0; 
drone.phi   = 0; 
drone.m     = 1;
drone.l     = 0.3;


s_space_history = zeros(6,T/dt);
ds_space_history = zeros(6,T/dt);
dds_space_history = zeros(6,T/dt);

ref_history     = zeros(3,T/dt);
dref_history     = zeros(3,T/dt);
ddref_history     = zeros(3,T/dt);

prop_history    = zeros(4,T/dt);

for i = (0:dt:T)
    %Uncomment below if using symbolic functions
%     a = i;
%     s_goal = eval(ref);
%     ds_goal = eval(d_ref);
%     dds_goal = eval(dd_ref);
%     ddds_goal = eval(ddd_ref);
%     dddds_goal = eval(dddd_ref);

    % Uncomment below if using bang bang functions
    if i < Ts 
        indx = 1; 
    else 
        indx = 2;
    end
    a = i;
    s_goal = eval(ref(:,indx));
    ds_goal = eval(d_ref(:,indx));
    dds_goal = eval(dd_ref(:,indx));
    ddds_goal = eval(ddd_ref(:,indx));
    dddds_goal = eval(dddd_ref(:,indx));

    tacc_and_torque_input = classic_uav_control_function(s_goal, ds_goal, dds_goal, ddds_goal, dddds_goal, drone);
    thrust = tacc_and_torque_input(1)*dt^2/2 + drone.dT*dt + drone.T;
    thrust_and_torque_input = [thrust; tacc_and_torque_input(2:4)];
    space_acc = classic_drone_system(thrust_and_torque_input, drone);

    drone.T     = thrust;
    drone.dT    = tacc_and_torque_input(1)*dt + drone.dT;
    drone.w     = prop_vel_from_tt(thrust_and_torque_input, kf, km, drone.l, drone.m, drone.Ib);
    drone.s     = space_acc*dt^2/2 + drone.ds*dt+ drone.s;
    drone.ds    = space_acc*dt + drone.ds; 
    drone.ddds  = (space_acc - drone.dds)/dt;
    drone.dds   = space_acc;


    % storing info for plot
    s_space_history(:,round(i/dt)+1)    = drone.s;
    ds_space_history(:,round(i/dt)+1)   = drone.ds;
    dds_space_history(:,round(i/dt)+1)  = drone.dds;

    ref_history(:,round(i/dt)+1)        = s_goal(1:3);
    dref_history(:,round(i/dt)+1)       = ds_goal(1:3);
    ddref_history(:,round(i/dt)+1)      = dds_goal(1:3);

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


figure Name '3D Path'
plot3(s_space_history(1,:), s_space_history(2,:), s_space_history(3,:), 'LineWidth', 2, 'Color', 'c')
hold on
plot3(ref_history(1,:), ref_history(2,:), ref_history(3,:), '-.', 'Color', 'k')
axis equal
ax = gca;
axis([0 100 -10 10 -10 10])
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
grid on

function prop_vel = prop_vel_from_tt(tt, kf, km, l, m, I_B)
    tt = [m, zeros(1,3); zeros(3,1), I_B]*tt;
    prop_vel = zeros(4,1);
    prop_vel(1) = tt(1)/(4*kf) + tt(4)/(4*km) + tt(3)/(2*l*km);
    prop_vel(2) = tt(1)/(4*kf) - tt(4)/(4*km) + tt(2)/(2*l*km);
    prop_vel(3) = tt(1)/(4*kf) + tt(4)/(4*km) - tt(3)/(2*l*km);
    prop_vel(4) = tt(1)/(4*kf) - tt(4)/(4*km) - tt(2)/(2*l*km);
end