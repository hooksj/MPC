function [x_ref, p1, p2, p3, p4, C1, C2, C3, C4] = ...
foot_step_planner(vel, rot_vel, x0, C_init, p1_init, p2_init, p3_init, p4_init, ...
                  t1, t2, t3, t4, dt)
              
    yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];
              
    accel = 0.1;
    rot_accel = 0.1;
    
    max_vel = 1.5;
    max_rot_vel = 1.0;

    x_ref = zeros(4,11);
    p1 = zeros(2,11);
    p2 = zeros(2,11);
    p3 = zeros(2,11);
    p4 = zeros(2,11);
    C1 = zeros(1,11);
    C2 = zeros(1,11);
    C3 = zeros(1,11);
    C4 = zeros(1,11);
    
    x_ref(:,1) = x0(1:4,1);
    p1(:,1) = p1_init;
    p2(:,1) = p2_init;
    p3(:,1) = p3_init;
    p4(:,1) = p4_init;
    C1(1,1) = C_init(1,1);
    C2(1,1) = C_init(1,2);
    C3(1,1) = C_init(1,3);
    C4(1,1) = C_init(1,4);
    
    for i = 2:11
        
    end
end