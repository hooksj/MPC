clear all; clc; close all;

% Function to create yaw rotation matrix
yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];

% Gait Parameters for a single leg
swing_time = 0.2;
stance_time = 0.2;

% Timing parameters
T_final = swing_time + stance_time; % final time of the entire plan
dt = 0.03; % time step of the QP
np = int64(T_final/dt);
t = linspace(0.0, T_final, np);

% State vector in the inertial frame
x = zeros(3,1);

% velocities set by the user to determine the trajectory
x_dot = 0.5;
y_dot = 0.0;
yaw_dot = 1.0;

% Gait timing 
t1 = [0.0, swing_time];
t2 = [stance_time-(stance_time-swing_time)*0.5, swing_time+stance_time-(stance_time-swing_time)*0.5];
t3 = [0.0, swing_time];
t4 = [stance_time-(stance_time-swing_time)*0.5, swing_time+stance_time-(stance_time-swing_time)*0.5];

%     t1 = [0.0, step_time];
%     t2 = [2.0*step_time, 3.0*step_time];
%     t3 = [step_time, 2.0*step_time];
%     t4 = [3.0*step_time, 4.0*step_time];

%     t1 = [step_time, 2.0*step_time];
%     t2 = [3.0*step_time, 4.0*step_time];
%     t3 = [0.0, step_time];
%     t4 = [2.0*step_time, 3.0*step_time];

c = [1, 1, 1, 1]; % indicates which foot is on the ground 1:on 0:off

% Nominal foot positions
p1_nom = [0.3; 0.2];
p2_nom = [-0.3; 0.2];
p3_nom = [-0.3; -0.2];
p4_nom = [0.3; -0.2];

% Current foot positions
p1 = p1_nom;
p2 = p2_nom;
p3 = p3_nom;
p4 = p4_nom;

% Matrices for storing the entire trajectory
x_ref = zeros(length(x),np);
P1 = zeros(2,np);
P2 = zeros(2,np);
P3 = zeros(2,np);
P4 = zeros(2,np);
T1 = zeros(np,2);
T2 = zeros(np,2);
T3 = zeros(np,2);
T4 = zeros(np,2);
C = zeros(np,4);


C(1,:) = ones(1,4);
P1(:,1) = p1_nom;
P2(:,1) = p2_nom;
P3(:,1) = p3_nom;
P4(:,1) = p4_nom;
T1(1,:) = t1;
T2(1,:) = t2;
T3(1,:) = t3;
T4(1,:) = t4;


% Build the desired trajectory for 2 steps
for i = 2:np  

    % Linear
    vel = yaw_rot(x_ref(3,i-1))*[x_dot; y_dot];
    x_ref(1:2,i) = x_ref(1:2,i-1) + vel*dt;

    x_ref(3,i) = x_ref(3,i-1) + yaw_dot*dt;

    % Determine the footstep locations
    t1 = t1 - [dt, dt]; 
    if t1(1) < 0.0
       if c(1) == 0
          c(1) = 1; 
          p1 = x_ref(1:2,i)+yaw_rot(x_ref(3,i))*(p1_nom) + ...
              yaw_rot(x_ref(3,i))*[x_dot;y_dot]*stance_time*0.5;
       else
          c(1) = 0;
       end
       t1(1) = t1(2);
       t1(2) = swing_time+stance_time - dt;
    end

    t2 = t2 - [dt, dt];
    if t2(1) < 0.0
       if c(2) == 0
          c(2) = 1; 
          p2 = x_ref(1:2,i)+yaw_rot(x_ref(3,i))*(p2_nom) + ...
              yaw_rot(x_ref(3,i))*[x_dot;y_dot]*stance_time*0.5;
       else
          c(2) = 0;
       end
       t2(1) = t2(2);
       t2(2) = swing_time+stance_time - dt;
    end

    t3 = t3 - [dt, dt];
    if t3(1) < 0.0
       if c(3) == 0
          c(3) = 1; 
          p3 = x_ref(1:2,i)+yaw_rot(x_ref(3,i))*(p3_nom) + ...
              yaw_rot(x_ref(3,i))*[x_dot;y_dot]*stance_time*0.5;
       else
          c(3) = 0;
       end
       t3(1) = t3(2);
       t3(2) = swing_time+stance_time - dt;
    end

    t4 = t4 - [dt, dt];
    if t4(1) < 0.0
       if c(4) == 0
          c(4) = 1; 
          p4 = x_ref(1:2,i)+yaw_rot(x_ref(3,i))*(p4_nom) + ...
              yaw_rot(x_ref(3,i))*[x_dot;y_dot]*stance_time*0.5;
       else
          c(4) = 0;
       end
       t4(1) = t4(2);
       t4(2) = swing_time+stance_time - dt;
    end

    % Store trajectory
    P1(:,i) = p1;
    P2(:,i) = p2;
    P3(:,i) = p3;
    P4(:,i) = p4;
    T1(i,:) = t1;
    T2(i,:) = t2;
    T3(i,:) = t3;
    T4(i,:) = t4;
    C(i,:) = c;
end

fig = figure 
for j = 1:np
   plot(x_ref(1,j), x_ref(2,j),'o')
   hold on
   if C(j,1) == 1.0
       plot(P1(1,j), P1(2,j),'x')
   end
   if C(j,2) == 1.0
       plot(P2(1,j), P2(2,j),'x')
   end
   if C(j,3) == 1.0
       plot(P3(1,j), P3(2,j),'x')
   end
   if C(j,4) == 1.0
       plot(P4(1,j), P4(2,j),'x')
   end
   
   axis([-0.5, 0.5 , -0.5, 0.5])
   clf(fig)
end

