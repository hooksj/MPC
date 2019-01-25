% Create test data for the MPC

% Function to create yaw rotation matrix
yaw_rot = @(x) [cos(x), -sin(x), 0; sin(x) cos(x), 0; 0, 0, 1];

mass = 20;

yaw = 0.0;

I_inv = eye(3);

g = 9.81;

t = linspace(0.0, 0.5, 11);

cm_pos = zeros(9,3);
cm_pos(1,:) = [0.0, 0.0, 0.5];
% CM trajectory
for i = 2:11
   cm_pos(i,:) = cm_pos(i-1,:) + [0.05, 0.0, 0.0]; 
end

t1 = [0.2, 0.4, 0.5];
t2 = [0.0, 0.2, 0.4, 0.5];
t3 = [0.2, 0.4, 0.5];
t4 = [0.0, 0.2, 0.4, 0.5];

p1 = zeros(2,3);
p2 = zeros(2,3);
p3 = zeros(2,3);
p4 = zeros(2,3);

p1(1,:) = [0.4, -0.2, -0.0];
p2(1,:) = [0.4, 0.2, -0.0];
p3(1,:) = [-0.4, 0.2, -0.0];
p4(1,:) = [-0.4, -0.2, -0.0];

p1(2,:) = p1(1,:) + cm_pos(11,:);
p3(2,:) = p3(1,:) + cm_pos(11,:);
p2(2,:) = p2(1,:) + cm_pos(7,:);
p4(2,:) = p4(1,:) + cm_pos(7,:);

r1 = zeros(9,3);
r2 = zeros(9,3);
r3 = zeros(9,3);
r4 = zeros(9,3);

for i = 1:9
   c1 = find(t1>t(i),1); 
   c2 = find(t2>t(i),1); 
   c3 = find(t3>t(i),1); 
   c4 = find(t4>t(i),1); 
   
   if(rem(c1,2))
       r1(i,:) = cm_pos(i,:) - p1(int32(c1/2),:);
   end
   
   if(rem(c2,2))
       r2(i,:) = cm_pos(i,:) - p2(int32(c2/2),:);
   end
   
   if(rem(c3,2))
       r3(i,:) = cm_pos(i,:) - p3(int32(c3/2),:);
   end
   
   if(rem(c4,2))
       r4(i,:) = cm_pos(i,:) - p4(int32(c4/2),:);
   end
end

save test_data t yaw r1 r2 r3 r4 mass I_inv