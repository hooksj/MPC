clear all; clc; close all;
% ========================================================================
% This is a high level planner that will pass footstep locations and a
% desired CM velocity to a QP position controller. 
% 
% - End conditions and foot a footstep plan is created based off of forward
%   integration of the user input commands.
%
% - These conditions are passed to a QP solved by gurobi to come up with a
%   feasible CM trajectory for the given conditions.
% 
% USER INPUTS
% w - Positive Y
% s - Negative Y
% d - Positive X
% a - Negative X
% j - Positive yaw
% d - Negative yaw
% q - Quit
%
% ========================================================================
% Function to create yaw rotation matrix
yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];

% Gait Parameters for a single leg
swing_time = 0.2;
stance_time = 0.2;

% Timing parameters
T_final = swing_time + stance_time + 0.1; % final time of the entire plan
dt = 0.05; % time step of the QP
np = int64(T_final/dt)+1;

% State vector in the inertial frame
x = zeros(6,np); % x = [x, x_dot, y, y_dot, yaw, yaw_dot]
h = 0.5; % initial height

% Feet position vectors
p1 = zeros(2,np);
p2 = zeros(2,np);
p3 = zeros(2,np);
p4 = zeros(2,np);

% feet contact indicators
C1 = zeros(1,np);
C2 = zeros(1,np);
C3 = zeros(1,np);
C4 = zeros(1,np);

% velocities set by the user to determine the trajectory
x_dot = 0.0;
y_dot = 0.0;
yaw_dot = 0.0;

% Gait timing 
t1 = [0.0; swing_time];
t2 = [stance_time-(stance_time-swing_time)*0.5; swing_time+stance_time-(stance_time-swing_time)*0.5];
t3 = [0.0; swing_time];
t4 = [stance_time-(stance_time-swing_time)*0.5; swing_time+stance_time-(stance_time-swing_time)*0.5];
T1 = zeros(2,np);
T2 = zeros(2,np);
T3 = zeros(2,np);
T4 = zeros(2,np);

p1_nom = [0.25;0.15];
p2_nom = [-0.25;0.15];
p3_nom = [-0.25;-0.15];
p4_nom = [0.25;-0.15];

% initial parameters
p1(:,1) = p1_nom;
p2(:,1) = p2_nom;
p3(:,1) = p3_nom;
p4(:,1) = p4_nom;
C1(1,1) = 1.0;
C2(1,1) = 1.0;
C3(1,1) = 1.0;
C4(1,1) = 1.0;
T1(:,1) = t1;
T2(:,1) = t2;
T3(:,1) = t3;
T4(:,1) = t4;

controller = MPC_Position_Controller_class(np-1, dt, h);
animator = Animation_class(x(1:2:3,1), p1(:,1), p2(:,1), p3(:,1), p4(:,1));

while 1
    
    x_accel = 0.3;
    y_accel = 0.2;
    yaw_accel = 2.0;
    
    [x_dot, y_dot, yaw_dot, quit] = animator.GetUserInputs();
    
    if quit == 1
       break; 
    end
    
    for i = 2:np
        % update yaw velocity
        if x(6,i-1) < yaw_dot
            if abs(yaw_dot - x(6,i-1)) < yaw_accel*dt
                x(6,i) = yaw_dot;
            else
                x(6,i) = x(6,i-1) + yaw_accel*dt;
            end
        elseif  x(6,i-1) > yaw_dot
            if abs(yaw_dot - x(6,i-1)) < yaw_accel*dt
                x(6,i) = yaw_dot;
            else
                x(6,i) = x(6,i-1) - yaw_accel*dt;
            end
        end
        
        % update yaw position
        x(5,i) = x(5,i-1) + x(6,i)*dt;
        
        R = yaw_rot(x(5,i));
        
        % rotate user input commands into the inertial frame
        rotated = R*[x_dot, x_accel;y_dot, y_accel];
        x_dot = rotated(1,1);
        y_dot = rotated(2,1);
        %x_accel = rotated(1,2);
        %y_accel = rotated(2,2);
        
        % update linear velocity
        if x(2,i-1) < x_dot
            if abs(x_dot - x(2,i-1)) < x_accel*dt
                x(2,i) = x_dot;
            else
                x(2,i) = x(2,i-1) + x_accel*dt;
            end
        elseif  x(2,i-1) > x_dot
            if abs(x_dot - x(2,i-1)) < x_accel*dt
                x(2,i) = x_dot;
            else
                x(2,i) = x(2,i-1) - x_accel*dt;
            end
        end
        
        if x(4,i-1) < y_dot
            if abs(y_dot - x(4,i-1)) < y_accel*dt
                x(4,i) = y_dot;
            else
                x(4,i) = x(4,i-1) + y_accel*dt;
            end
        elseif  x(4,i-1) > y_dot
            if abs(y_dot - x(4,i-1)) < y_accel*dt
                x(4,i) = y_dot;
            else
                x(4,i) = x(4,i-1) - y_accel*dt;
            end
        end
        
        % update linear position
        x(1,i) = x(1,i-1) + x(2,i)*dt;
        x(3,i) = x(3,i-1) + x(4,i)*dt;
        
        % Determine the footstep locations
        T1(:,i) = T1(:,i-1) - [dt; dt]; 
        if T1(1,i) < 0.0
           if C1(1,i-1) == 0
              C1(1,i) = 1; 
              p1(:,i) = x(1:2:3,i)+R*(p1_nom) + [x_dot;y_dot]*stance_time*0.5;
           else
              C1(1,i) = 0;
           end
           T1(1,i) = T1(2,i);
           T1(2,i) = swing_time+stance_time - dt;
        else
            p1(:,i) = p1(:,i-1);
            C1(:,i) = C1(:,i-1);
        end

        T2(:,i) = T2(:,i-1) - [dt; dt]; 
        if T2(1,i) < 0.0
           if C2(1,i-1) == 0
              C2(1,i) = 1; 
              p2(:,i) = x(1:2:3,i)+R*(p2_nom) + [x_dot;y_dot]*stance_time*0.5;
           else
              C2(1,i) = 0;
           end
           T2(1,i) = T2(2,i);
           T2(2,i) = swing_time+stance_time - dt;
        else
            p2(:,i) = p2(:,i-1);
            C2(:,i) = C2(:,i-1);
        end
        
        T3(:,i) = T3(:,i-1) - [dt; dt]; 
        if T3(1,i) < 0.0
           if C3(1,i-1) == 0
              C3(1,i) = 1; 
              p3(:,i) = x(1:2:3,i)+R*(p3_nom) + [x_dot;y_dot]*stance_time*0.5;
           else
              C3(1,i) = 0;
           end
           T3(1,i) = T3(2,i);
           T3(2,i) = swing_time+stance_time - dt;
        else
            p3(:,i) = p3(:,i-1);
            C3(:,i) = C3(:,i-1);
        end
        
        T4(:,i) = T4(:,i-1) - [dt; dt]; 
        if T4(1,i) < 0.0
           if C4(1,i-1) == 0
              C4(1,i) = 1; 
              p4(:,i) = x(1:2:3,i)+R*(p4_nom) + [x_dot;y_dot]*stance_time*0.5;
           else
              C4(1,i) = 0;
           end
           T4(1,i) = T4(2,i);
           T4(2,i) = swing_time+stance_time - dt;
        else
            p4(:,i) = p4(:,i-1);
            C4(:,i) = C4(:,i-1);
        end
    end  
    
    x(3:4,:)
    X = controller.update(x(1:4,:), p1, p2, p3, p4, C1, C2, C3, C4); 
    
    x_pos = [1.0, dt, dt^2, dt^3, dt^4]*X(1:5,1);
    x_vel = [0.0, 1.0, 2.0*dt, 3.0*dt^2, 4.0*dt^3]*X(1:5,1);
    y_pos = [1.0, dt, dt^2, dt^3, dt^4]*X(6:10,1)
    y_vel = [0.0, 1.0, 2.0*dt, 3.0*dt^2, 4.0*dt^3]*X(6:10,1)
    
%     if abs(y_vel) > 0.001
%         y = 1
%     end
    
    animator.update([x_pos;y_pos], p1(:,2), p2(:,2), p3(:,2), p4(:,2), ...
                    [C1(1,2), C2(1,2), C3(1,2), C4(1,2)])
    
    % Set initial position
    x(1:4,1) = [x_pos; x_vel; y_pos; y_vel];
    x(5:6,1) = x(5:6,2);
    T1(:,1) = T1(:,2);
    T2(:,1) = T2(:,2);
    T3(:,1) = T3(:,2);
    T4(:,1) = T4(:,2);
    C1(:,1) = C1(:,2);
    C2(:,1) = C2(:,2);
    C3(:,1) = C3(:,2);
    C4(:,1) = C4(:,2);
    p1(:,1) = p1(:,2);
    p2(:,1) = p2(:,2);
    p3(:,1) = p3(:,2);
    p4(:,1) = p4(:,2);
    pause(0.01);
    
end

close all




