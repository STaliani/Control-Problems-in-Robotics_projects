%% This function implement the dynamical step of a fully actuated drone
% it takes the input generated by the controller and implements its 
% dynamic behaviour.
% drone.w, drone.s, drone.ds, drone.dds, drone.Ib, drone.theta, drone.phi, drone.m 



function out = drone_system(vel_input, drone)
    
    w     = drone.w; 
    s     = drone.s; 
    ds    = drone.ds; 
    dds   = drone.dds; 
    I_B   = drone.Ib;
    theta = drone.theta; 
    phi   = drone.phi; 
    m     = drone.m;
    l     = drone.l;

    kf = 0.016;
    km = 0.0027;

    R_y = Ry(s(5));
    R_x = Rx(s(4));
    R_z = Rz(s(6));
    %rpy rotation matrix
    R_B = R_z*R_y*R_x;

    c = -inv(I_B)*(cross(ds(4:6),I_B*ds(4:6)));
    f = [0;0;-9.81; c];


    ct = cos(theta);
    st = sin(theta);
    cp = cos(phi);
    sp = sin(phi);

    Fm = [kf*cp*st, kf*cp*st, kf*cp*st, kf*cp*st ; 
            -kf*sp,   -kf*sp,   -kf*sp,   -kf*sp ; 
          kf*cp*ct, kf*cp*ct, kf*cp*ct, kf*cp*ct];

        
    taum = [        -km*ct*sp, l*kf*ct*cp+km*ct*sp,        -km*ct*sp, -l*kf*ct*cp+km*ct*sp ;
            -l*kf*cp*ct+km*sp,              -km*sp, l*kf*cp*ct+km*sp,               -km*sp ;
            -l*kf*sp-km*cp*ct,-l*kf*cp*st+km*cp*ct, l*kf*sp-km*cp*ct,  l*kf*cp*st+km*cp*ct];
        
        
    zer = zeros(3,3);
    J_R = [1/m*R_B,      zer ; 
               zer, inv(I_B)];

    J = J_R*[Fm, zeros(3,2); taum, zeros(3,2)];

    out = f + J*vel_input;
end


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

    