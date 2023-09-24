%% This function implement control of fully actuated drone
% it takes references for position velocity and acceleration of the drone
% and uses as input the current status of the drone specified by the
% structure:
% drone.w, drone.s, drone.ds, drone.dds, drone.Ib, drone.theta, drone.phi, drone.m 



function input = classic_uav_control_function(s_goal, ds_goal, dds_goal, ddds_goal, dddds_goal, drone)

%     syms psi_s theta phi d_psi d_theta d_phi csi1 csi2 'real'

    T     = drone.T;
    dT    = drone.dT;
    s     = drone.s; 
    ds    = drone.ds; 
    dds   = drone.dds;
    ddds  = drone.ddds;
    
    
    K_p0 = 8*eye(3);
    K_p1 = 24*eye(3);
    K_p2 = 32*eye(3);
    K_p3 = 16*eye(3);
    K_om1 = 24;
    K_om2 = 32;
    K_om3 = 16;

   
        
%     A = [-cos(psi_s)*sin(theta)*cos(phi)-sin(psi_s)*sin(phi), 0, 0, 0;
%          -sin(psi_s)*sin(theta)*cos(phi)+cos(psi_s)*sin(phi), 0, 0, 0;
%                                         -cos(phi)*cos(theta), 0, 0, 0;
%                                                            0, 0, 0, 1];
%     
%     ang_dot = [d_phi; d_theta; d_psi];
%     G = A(1:3,1);
%     G_prime = jacobian(G, [phi, theta, psi_s]);
%     G_prime_prime = jacobian(G_prime*ang_dot, [phi, theta, psi_s]);
%     
%     
%     J_star_sym = [ G,  G_prime*csi1;
%                    0,   0,   0,   1];
%     
%     final_sub_sym = [G_prime_prime*ang_dot*csi1 + 2*G_prime*ang_dot*csi2;
%                                                                        0];


    theta = s(5);
    phi = s(4);
    psi = s(6);

    d_theta = ds(5);
    d_phi = ds(4);
    d_psi = ds(6);

    csi1 = T;
    csi2 = dT;

    %J_star = eval(J_star_sym);
    %final_sub = eval(final_sub_sym);

    J_star = -[- sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta), -csi1*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), -csi1*cos(phi)*cos(psi)*cos(theta), -csi1*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta));
                 cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta),  csi1*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)), -csi1*cos(phi)*cos(theta)*sin(psi), -csi1*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta));
                                             -cos(phi)*cos(theta),                                 csi1*cos(theta)*sin(phi),           csi1*cos(phi)*sin(theta),                                                        0;
                                                                0,                                                        0,                                  0,                                                        1];
     
         
    final_sub =-[  csi1*(d_theta*(d_phi*cos(psi)*cos(theta)*sin(phi) + d_psi*cos(phi)*cos(theta)*sin(psi) + d_theta*cos(phi)*cos(psi)*sin(theta)) + d_phi*(d_phi*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - d_psi*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) + d_theta*cos(psi)*cos(theta)*sin(phi)) + d_psi*(d_psi*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - d_phi*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) + d_theta*cos(phi)*cos(theta)*sin(psi))) - csi2*(d_phi*(2*cos(phi)*sin(psi) - 2*cos(psi)*sin(phi)*sin(theta)) + d_psi*(2*cos(psi)*sin(phi) - 2*cos(phi)*sin(psi)*sin(theta)) + 2*d_theta*cos(phi)*cos(psi)*cos(theta));
                 - csi1*(d_psi*(d_phi*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + d_psi*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + d_theta*cos(phi)*cos(psi)*cos(theta)) - d_theta*(d_phi*cos(theta)*sin(phi)*sin(psi) - d_psi*cos(phi)*cos(psi)*cos(theta) + d_theta*cos(phi)*sin(psi)*sin(theta)) + d_phi*(d_phi*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + d_psi*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - d_theta*cos(theta)*sin(phi)*sin(psi))) - csi2*(d_psi*(2*sin(phi)*sin(psi) + 2*cos(phi)*cos(psi)*sin(theta)) - d_phi*(2*cos(phi)*cos(psi) + 2*sin(phi)*sin(psi)*sin(theta)) + 2*d_theta*cos(phi)*cos(theta)*sin(psi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                           csi2*(2*d_phi*cos(theta)*sin(phi) + 2*d_theta*cos(phi)*sin(theta)) + csi1*(d_phi*(d_phi*cos(phi)*cos(theta) - d_theta*sin(phi)*sin(theta)) + d_theta*(d_theta*cos(phi)*cos(theta) - d_phi*sin(phi)*sin(theta)));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0];


    pr_dddd = dddds_goal(1:3) + K_p0*(ddds_goal(1:3)-ddds(1:3)) + K_p1*(dds_goal(1:3)-dds(1:3)) + K_p2*(ds_goal(1:3)-ds(1:3)) + K_p3*(s_goal(1:3) - s(1:3));

    e_R = s_goal(6) - s(6);
    omegar_dd = -K_om1*dds(6) - K_om2*ds(6) + K_om3*e_R;

    snap = [pr_dddd; omegar_dd];

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