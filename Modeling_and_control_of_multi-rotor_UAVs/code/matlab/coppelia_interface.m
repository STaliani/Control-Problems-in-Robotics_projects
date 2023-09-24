function coppelia_interface(pos, tilt, ref, N, dt)
    disp('Program started');
    % sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

    if (clientID>-1)
        disp('Connected to remote API server');
            
        % Now try to retrieve data in a blocking fashion (i.e. a service call):
        [res,objs]=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);
        
        if (res==sim.simx_return_ok)
            fprintf('Number of objects in the scene: %d\n',length(objs));
        else
            fprintf('Remote API function call returned with error code: %d\n',res);
        end
            
        pause(2);
    
        % Now retrieve streaming data (i.e. in a non-blocking fashion):
        t=clock;
        startTime=t(6);
        currentTime=t(6);
        sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming); % Initialize streaming
        
        propellerHandles=[0,0,0,0];
        jointHandles=[0,0,0,0];
        thetaJointHandles=[0,0,0,0];
        phiJointHandles=[0,0,0,0];

        init_or = [-pi/2, -pi/4, -pi/2];    
            

        [ret, propellerHandles(1)]  = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[0]/respondable', sim.simx_opmode_blocking);
        [ret, jointHandles(1)]      = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[0]/joint', sim.simx_opmode_blocking);
        [ret, thetaJointHandles(1)] = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[0]/Revolute_joint', sim.simx_opmode_blocking);
        [ret, phiJointHandles(1)]   = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[0]/Revolute_joint/Revolute_joint', sim.simx_opmode_blocking);
        [ret, propellerHandles(2)]  = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[1]/respondable', sim.simx_opmode_blocking);
        [ret, jointHandles(2)]      = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[1]/joint', sim.simx_opmode_blocking);
        [ret, thetaJointHandles(2)] = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[1]/Revolute_joint', sim.simx_opmode_blocking);
        [ret, phiJointHandles(2)]   = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[1]/Revolute_joint/Revolute_joint', sim.simx_opmode_blocking);
        [ret, propellerHandles(3)]  = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[2]/respondable', sim.simx_opmode_blocking);
        [ret, jointHandles(3)]      = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[2]/joint', sim.simx_opmode_blocking);
        [ret, thetaJointHandles(3)] = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[2]/Revolute_joint', sim.simx_opmode_blocking);
        [ret, phiJointHandles(3)]   = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[2]/Revolute_joint/Revolute_joint', sim.simx_opmode_blocking);
        [ret, propellerHandles(4)]  = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[3]/respondable', sim.simx_opmode_blocking);
        [ret, jointHandles(4)]      = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[3]/joint', sim.simx_opmode_blocking);
        [ret, thetaJointHandles(4)] = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[3]/Revolute_joint', sim.simx_opmode_blocking);
        [ret, phiJointHandles(4)]   = sim.simxGetObjectHandle(clientID, './Cuboid/propeller[3]/Revolute_joint/Revolute_joint', sim.simx_opmode_blocking);
    
        thetaJointHandles

        [ret, hely]  = sim.simxGetObjectHandle(clientID, './Cuboid', sim.simx_opmode_blocking);
        [ret, dummy] = sim.simxGetObjectHandle(clientID, './Dummy', sim.simx_opmode_blocking);

        [ret, pos_pres]  = sim.simxGetObjectPosition(clientID, hely, -1, sim.simx_opmode_blocking);

        for i = 1:N
            sim.simxSetObjectPosition(clientID, hely, -1, pos(1:3,i), sim.simx_opmode_oneshot);
            sim.simxSetObjectOrientation(clientID, hely, -1, [pos(4,1), pos(5,i),pos(6,i)]+init_or, sim.simx_opmode_oneshot);
            phi = tilt(1,i);
            theta = tilt(2,i);

            for j = 1:4
                sim.simxSetJointPosition(clientID, phiJointHandles(j), -phi, sim.simx_opmode_oneshot); 
                sim.simxSetJointPosition(clientID, thetaJointHandles(j), theta, sim.simx_opmode_oneshot);
                sim.simxSetJointPosition(clientID, jointHandles(j), i, sim.simx_opmode_oneshot);
            end

            sim.simxSetObjectPosition(clientID, dummy, -1, ref(1:3,i), sim.simx_opmode_oneshot);            
            
            pause(dt)
            
        end
        
        % Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID);

        % Now close the connection to CoppeliaSim:    
        sim.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    sim.delete(); % call the destructor!
    
    disp('Program ended');
end