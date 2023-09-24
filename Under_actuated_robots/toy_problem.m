clc
close all
clear all

addpath('C:/Users/saver/Documents/MATLAB/casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

%% Constants definition
m  =  0.20;
g  = -9.81;
dT =  0.01;
T  =  1.00;
%initial conditions
x0 =  0.1;%0.15; %0.05; 
v0 =  0.00;
f0 =  0.00;

cost_multiplier = 100;

M_fdot = 100;   % Max f_dot
eps_r  = 0.004; % rc
K_bs   = 20;    % 1/s dcc 
eps_d  = 0.05;  % dcc
K_h    = 250;   % hc
K_fz   = 500;   % hc control bounds


%% dynamics


X = casadi.MX.sym('X', 3);
U = casadi.MX.sym('U', 2);
x     = X(1);
v     = X(2);
f     = X(3);
p     = U(1);
f_dot = U(2);

X_dot =  [        v  ;
          g+(f+p)/m  ;
              f_dot ];

F = casadi.Function('continuous_dynamics', {X,U}, {X_dot});

% discretization   
X_k1 = casadi.MX.sym('X_k1', 3);
X_k2 = casadi.MX.sym('X_k2', 3);
U_k1 = casadi.MX.sym('U_k1', 2);
U_k2 = casadi.MX.sym('U_k2', 2);

update1 = X_k1 + 0.5 * dT *  (F(X_k1, U_k1) + F(X_k2, U_k2)); %Implicit trapezoidal
update2 = X_k1 + dT *  F(X_k1, U_k1); %explicit euler

F_k = casadi.Function('discrete_dynamics', {X_k1, U_k1, X_k2, U_k2}, {update1});
    
%% Complementary Conditions

rc = x*f - eps_r; %relaxed complementary condition

dc = v*f+x*f_dot+K_bs*x*f - eps_d; %dynamic complementary conditios

delta = 1/cosh(K_fz*x);
hc = f_dot-delta*M_fdot+ (1-delta)*K_h*f; %hyperbolic tangent

cc = casadi.Function('cc', {X,U},{rc}); %choose the complementary condition 
%% Optimization

opti = casadi.Opti();
N = round(T/dT);

Xo = opti.variable(3, N+1);
Uo = opti.variable(2, N+1);

% initial conditions
opti.subject_to(Xo(:,1) == [x0; f0; v0]);

t= 0 : dT : T;
freeFalling = x0 + v0*t + 0.5*g*t.*t;
flying = freeFalling > 0;
freeFalling = flying .* freeFalling;
free_falling_velocity = v0 + g * t;
free_falling_velocity = flying .* free_falling_velocity;
in_contact = ~freeFalling;
expectedForce = m*abs(g)*in_contact;

opti.set_initial(Xo, [freeFalling; expectedForce; free_falling_velocity]);

% constraints
for state = 1:N
    opti.subject_to(Xo(:,state+1) == F_k(Xo(:,state),Uo(:,state), Xo(:,state+1),Uo(:,state+1))); %dynamics
    opti.subject_to(Xo(1,state) >= 0);                                           %position positive
    opti.subject_to(Xo(3,state) >= 0);                                           %force positive
    opti.subject_to(-M_fdot <= Uo(2, state));                                    %max force derivative
    opti.subject_to(Uo(2, state) <= M_fdot);                                     %max force derivative
    opti.subject_to(cc(Xo(:,state),Uo(:,state))<= 0)                                     %complementarity 
end

opti.subject_to(Xo(1,N+1) >= 0); %position positive
opti.subject_to(Xo(2,N+1) >= 0); %force positive
opti.subject_to(-M_fdot <= Uo(2, N+1)); %max force derivative
opti.subject_to(Uo(2, N+1) <= M_fdot); %max force derivative
opti.subject_to(cc(Xo(:,N+1),Uo(:,N+1)) <= 0) %complementarity

cost = cost_multiplier*sumsqr(Uo(1,:));

opti.minimize(cost);

opti.solver('ipopt');

%%Uncomment to repeat the experiment multiple times
% time=zeros(1,30);
% for i = 1:30
    sol = opti.solve();
    %time(i)= sol.stats.t_wall_total;
% end
% 
% mean_time=mean(time)

Xi=sol.value(Xo);

figure
subplot(2,2,1)
plot(t,Xi(1,:),'k');
hold on
plot(t, freeFalling, '--')
title("x")
%ylim([-0.01, 1.1 *Xi(1:N+1)])

subplot(2,2,2)
plot(t,Xi(3,:),'c');
hold on
plot(t, expectedForce, '--')
title("f")

subplot(2,2,3)
plot(t,Xi(1,:).*Xi(3,:),'g');
title('  complemetary');



subplot(2,2,4)
propeller=sol.value(Uo);
plot(t, propeller(1,:))
title('p')

figure 
plot(t,Xi(1,:),'b');
hold on
plot(t, freeFalling, '--')
plot(t,Xi(3,:)/10,'r');
plot(t, expectedForce/10, '--')
title("pos/force")

figure 
plot(t,Xi(1,:),'b');
hold on
plot(t, freeFalling, '--')
plot(t,Xi(2,:)/10,'r');
plot(t, free_falling_velocity/10, '--')
title("pos/vel")