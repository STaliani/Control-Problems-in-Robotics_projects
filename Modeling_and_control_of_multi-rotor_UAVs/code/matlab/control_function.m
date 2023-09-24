%% This function implement control of fully actuated drone
% it takes references for position velocity and acceleration of the drone
% and uses as input the current status of the drone specified by the
% structure:
% drone.w, drone.s, drone.ds, drone.dds, drone.Ib, drone.theta, drone.phi, drone.m 



function input = control_function(s_goal, ds_goal, dds_goal, ddds_goal, drone)
    
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
    
    K_p1 = 24*eye(3);
    K_p2 = 32*eye(3);
    K_p3 = 16*eye(3);
    K_om1 = 24*eye(3);
    K_om2 = 32*eye(3);
    K_om3 = 16*eye(3);

    ct = cos(theta);
    st = sin(theta);
    cp = cos(phi);
    sp = sin(phi);
        
    R_y = Ry(s(5));
    R_x = Rx(s(4));
    R_z = Rz(s(6));
    %rpy rotation matrix
    R_B = R_z*R_y*R_x;
        
    Fm = [kf*cp*st, kf*cp*st, kf*cp*st, kf*cp*st ; 
            -kf*sp,   -kf*sp,   -kf*sp,   -kf*sp ; 
          kf*cp*ct, kf*cp*ct, kf*cp*ct, kf*cp*ct];
        
    taum = [        -km*ct*sp, l*kf*ct*cp+km*ct*sp,        -km*ct*sp, -l*kf*ct*cp+km*ct*sp ;
            -l*kf*cp*ct+km*sp,              -km*sp, l*kf*cp*ct+km*sp,               -km*sp ;
            -l*kf*sp-km*cp*ct,-l*kf*cp*st+km*cp*ct, l*kf*sp-km*cp*ct,  l*kf*cp*st+km*cp*ct];
        
        
    zer = zeros(3,3);
    J_R = [1/m*R_B,      zer ; 
               zer, inv(I_B)];
     
    dFm_dphim = [-kf*sp*st, -kf*sp*st, -kf*sp*st, -kf*sp*st; 
                    -kf*cp,    -kf*cp,    -kf*cp,    -kf*cp;
                 -kf*sp*ct, -kf*sp*ct, -kf*sp*ct, -kf*sp*ct];
    
    dFm_dthetam = [ kf*cp*ct,  kf*cp*ct,  kf*cp*ct,  kf*cp*ct;
                           0,         0,         0,         0; 
                   -kf*cp*st, -kf*cp*st, -kf*cp*st, -kf*cp*st];
                       
    dtaum_dphim = [        -km*ct*cp, -l*kf*ct*sp+km*ct*cp,        -km*ct*cp,  l*kf*ct*sp+km*ct*cp;
                    l*kf*sp*ct+km*cp,               -km*cp,-l*kf*sp*ct+km*cp,               -km*cp;
                   -l*kf*cp+km*sp*ct,  l*kf*sp*st-km*sp*ct, l*kf*cp+km*sp*ct, -l*kf*sp*st-km*sp*ct];
    
    dtaum_dthetam = [   km*st*sp, -l*kf*st*cp-km*st*sp,    km*st*sp,  l*kf*st*cp-km*st*sp;
                      l*kf*cp*st,                    0, -l*kf*cp*st,                    0;
                        km*cp*st, -l*kf*cp*ct-km*cp*st,    km*cp*st,  l*kf*cp*ct-km*cp*st];
    

    second_mat = [  Fm,   dFm_dphim*w,   dFm_dthetam*w ;
                  taum, dtaum_dphim*w, dtaum_dthetam*w];
    
    skew_omega = Skew(ds(4:6));
    dskew_omega = Skew(dds(4:6));

    dc = -inv(I_B)*(dskew_omega*I_B*ds(4:6)+skew_omega*I_B*dds(4:6));

    J_star = J_R*second_mat;
    R_B_dot = skew_omega*R_B;

    final_sub = [1/m*R_B_dot*Fm*w; dc];

    pr_ddd = ddds_goal(1:3) + K_p1*(dds_goal(1:3)-dds(1:3)) + K_p2*(ds_goal(1:3)-ds(1:3)) + K_p3*(s_goal(1:3) - s(1:3));

    e_R = s_goal(4:6) - s(4:6);
    omegar_dd = -K_om1*dds(4:6) - K_om2*ds(4:6) + K_om3*e_R;

    snap = [pr_ddd; omegar_dd];

    input = inv(J_star)*(snap-final_sub);

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