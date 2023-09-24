%% Drone simulation wrapper
% We use the control function on the system dynamics defined in
% drone_system, and we plot the results


clc
clear all
close all

syms a 'real'

%% Hovering with non zero tilting
% s_goal      = [0;0;0.5;pi/8;pi/7;pi/2];
% ds_goal     = zeros(6,1);
% dds_goal    = zeros(6,1);
% ddds_goal   = zeros(6,1);
% dddds_goal  = zeros(6,1);

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
% a_max = 6;
Ts = 10;
Tf = 16;
% v_max = a_max*Ts;
% a_min = -2;
% sigma = [a_max*a^2/2, a_min*(a-Ts)^2/2+v_max*(a-Ts)+a_max*Ts^2/2, a_min*(Tf-Ts)^2/2+v_max*(Tf-Ts)+a_max*4^2/2];
% 
%sigma = [3*a^5/16000 - 3*a^4/320 + a^3/8 , 100];
sigma = 1.05*[3*a^5/500 - a^4*3/20 + a^3, 100];
%sigma = [3*a^4/100 - a^3*4/5 + 6*a^2, 100];
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
eps = 1e-2; % tolerance for the control switch

% Initialize the drone parameters
kf = 0.016;
km = 0.0027;

drone.w     = ones(4,1)*1*9.81/(kf*4);
drone.T     = 1*9.81;
drone.dT    = 0;
drone.s     = [0;0;0.5;0;0;0]; % starts at 50 cm in height 
drone.ds    = zeros(6,1); 
drone.dds   = zeros(6,1); 
drone.ddds  = zeros(6,1); 
drone.Ib    = diag([0.015, 0.015, 0.025]);
drone.theta = 0; 
drone.phi   = 0; 
drone.dtheta = 0; 
drone.dphi   = 0;
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

classic = false;
start = 0;
count = 0;
back = 0;
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
    elseif i<Tf
        indx = 2;
%     else
%         indx = 3;
    end
    a = i;
    s_goal = eval(ref(:,indx));
    ds_goal = eval(d_ref(:,indx));
    dds_goal = eval(dd_ref(:,indx));
    ddds_goal = eval(ddd_ref(:,indx));
    dddds_goal = eval(dddd_ref(:,indx));


    %if (abs(dds_goal(1)) < 5 && abs(dds_goal(2)) < 5) && (~classic || (abs(drone.ddds(1)) < eps && abs(drone.ddds(2)) < eps)) % (abs(drone.dds(1)) < 5 && abs(drone.dds(2)) < 5)
    if  abs(drone.theta) <= pi/6 && abs(drone.phi) <= pi/6

        if back == 1
            disp('in back')
            back = 0;
            drone.s(4) = drone.s(4) - drone.phi;
            drone.s(5) = drone.s(5) - drone.theta;
        end

        count = 0;
        disp('1')
        classic = false;
        acc_and_vel_input = control_function(s_goal, ds_goal, dds_goal, ddds_goal, drone);
        vel_and_pos_input = acc_and_vel_input*dt + vel_and_pos_input_old;
        vel_and_pos_input_old = vel_and_pos_input;
        space_acc = drone_system(vel_and_pos_input, drone);
    
        drone.w     = vel_and_pos_input(1:4);
        drone.dT    = (sum(drone.w)*kf - drone.T)/dt;
        drone.T     = sum(drone.w)*kf;
        drone.s     = space_acc*dt^2/2 + drone.ds*dt+ drone.s;
        drone.ds    = space_acc*dt + drone.ds;
        drone.ddds  = (space_acc - drone.dds)/dt;
        drone.dds   = space_acc; 
    
        drone.theta = vel_and_pos_input(6); 
        drone.phi   = vel_and_pos_input(5); 
        drone.dtheta = acc_and_vel_input(6); 
        drone.dphi   = acc_and_vel_input(5);
%         if abs(vel_and_pos_input(6)) >= pi/6
%             drone.theta = pi/6*vel_and_pos_input(6)/abs(vel_and_pos_input(6));
%             drone.dtheta = 0;
%         end
%         if abs(vel_and_pos_input(5)) >= pi/6
%             drone.phi   = pi/6*vel_and_pos_input(5)/abs(vel_and_pos_input(5)); 
%             drone.dphi = 0;
%         end
        start = 1;


        % storing info for plot
        s_space_history(:,round(i/dt)+1)    = drone.s;
    
        tilting_history(:,round(i/dt)+1) = [drone.phi; drone.theta];

    else 
%         count = count + 1;
%         K0 = (1+ count*0.005)*2;
%         K1 = (1+ count*0.01)*4;
%         K2 = (1+ count*0.01)*8;
%         K3 = (1+ count*0.01)*4;

        disp('2')
        classic = true;

        % add theta and phi to the current orientation for the classical
        % controller setting
        if start == 1
            disp('in start')
            start = 0;
            %drone.T = 9.81;
            drone.dT = 0;
            drone.s(4:6) = round(drone.s(4:6), 4);
            drone.s(4) = drone.s(4) + drone.phi;
            drone.s(5) = drone.s(5) + drone.theta;
            drone.ds(4:6) = round(drone.ds(4:6), 4);
            drone.dds(4:6) = round(drone.dds(4:6), 4);
            drone.ddds = zeros(6,1);
            back = 1;
        end
        
        %tacc_and_torque_input = classic_uav_control_function_2(s_goal, ds_goal, dds_goal, ddds_goal, dddds_goal, drone, K0, K1, K2, K3);
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

        drone.phi   = drone.s(4);
        drone.theta = drone.s(5);
        vel_and_pos_input_old = [drone.w; drone.phi; drone.theta];

        % storing info for plot
        s_space_history(:,round(i/dt)+1)    = drone.s;
        if abs(vel_and_pos_input(5)) >= pi/6
            tilting_history(1,round(i/dt)+1) = pi/6*drone.phi/abs(drone.phi);
            s_space_history(4,round(i/dt)+1) = s_space_history(4,round(i/dt)+1) - pi/6*drone.phi/abs(drone.phi);
        end
        if abs(vel_and_pos_input(6)) >= pi/6
            tilting_history(2,round(i/dt)+1) = pi/6*drone.theta/abs(drone.theta);
            s_space_history(5,round(i/dt)+1) = s_space_history(5,round(i/dt)+1) - pi/6*drone.theta/abs(drone.theta);
        end

    end


    % storing info for plot
    %s_space_history(:,round(i/dt)+1)    = drone.s;
    ds_space_history(:,round(i/dt)+1)   = drone.ds;
    dds_space_history(:,round(i/dt)+1)  = drone.dds;

    ref_history(:,round(i/dt)+1)        = s_goal(1:3);
    dref_history(:,round(i/dt)+1)       = ds_goal(1:3);
    ddref_history(:,round(i/dt)+1)      = dds_goal(1:3);

    %tilting_history(:,round(i/dt)+1) = [drone.phi; drone.theta];
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
axis([0 100 -10 10 -10 10])
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

function prop_vel = prop_vel_from_tt(tt, kf, km, l, m, I_B)
    tt = [m, zeros(1,3); zeros(3,1), I_B]*tt;
    prop_vel = zeros(4,1);
    prop_vel(1) = tt(1)/(4*kf) + tt(4)/(4*km) + tt(3)/(2*l*km);
    prop_vel(2) = tt(1)/(4*kf) - tt(4)/(4*km) + tt(2)/(2*l*km);
    prop_vel(3) = tt(1)/(4*kf) + tt(4)/(4*km) - tt(3)/(2*l*km);
    prop_vel(4) = tt(1)/(4*kf) - tt(4)/(4*km) - tt(2)/(2*l*km);
end