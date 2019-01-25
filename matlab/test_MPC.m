clear all; close all; clc;

% Function to create yaw rotation matrix
R = @(x) [cos(x), -sin(x) 0.0; sin(x) cos(x) 0.0; 0.0 0.0 1];

% Function to skew semetric cross product matrix
skew = @(x) [0.0 -x(3) x(2); x(3) 0.0 -x(1); -x(2) x(1) 0.0];

load test_data

controller = MPCControllerGurobi_class(3,0.03);
x0 = zeros(13,1);
x0(13,1) = -9.81;
x_ref = zeros(39,1);
x_ref(13,1) = -9.81;
x_ref(26,1) = -9.81;
x_ref(39,1) = -9.81;
x_ref(14:26,1) = zeros(13,1);
yaw = pi/4;
v1 = ones(36,1);
v1(2:3:end,1) = zeros(12,1);
v2 = ones(36,1);
v2(1:3:end,1) = zeros(12,1);
v3 = ones(36,1);
v3(1:3:end,1) = -1.0*ones(12,1);
v3(2:3:end,1) = zeros(12,1);
v4 = ones(36,1);
v4(1:3:end,1) = zeros(12,1);
v4(2:3:end,1) = -1.0*ones(12,1);

C = ones(3,4);
C(3,3) = 0.0;

[F, current_state] = controller.update(x0, x_ref, yaw, v1, v2, v3, v4, C);
F

yaw = pi/4;

dt = t(2)-t(1);
A = zeros(13,13);
B = zeros(13,12);
Bd = zeros(13,12,length(t)-2);
M = zeros(25,25);
Ident = eye(size(M,1));

x0 = ones(13,1);
x_ref = ones(13*(length(t)-2),1);

L = eye(117);
K = 0.001*eye(108);
G = zeros(109,109);
C = zeros(109,1);

tic

A(1:3,7:9) = R(yaw);
A(4:6,10:12) = eye(3);
A(12,13) = 1.0;

B(7:9,:) = [I_inv*skew(r1(1,:)), I_inv*skew(r2(1,:)), ...
            I_inv*skew(r3(1,:)), I_inv*skew(r4(1,:))];
        
B(10:12,:)= [eye(3)/mass, eye(3)/mass, eye(3)/mass, eye(3)/mass];

M(1:13,1:13) = A;
M(1:13,14:end) = B;

eM = Ident + M*dt +0.5*M*M*dt*dt;

Ad = eM(1:13,1:13);

Bd(:,:,1) = eM(1:13,14:end);

for i = 2:length(t)-2
    M(7:9,14:end) = [I_inv*skew(r1(i,:)), I_inv*skew(r2(i,:)), ...
                      I_inv*skew(r3(i,:)), I_inv*skew(r4(i,:))];
                  
    eM = Ident + M*dt +0.5*M*M*dt*dt;
    Bd(:,:,i) = eM(1:13,14:end);
end

Aq = zeros(13*(length(t)-2),13);
Aq(1:13,:) = Ad;
for n = 2:length(t)-2
   Aq((1+13*(n-1)):(13*n),:) =  Ad*Aq((1+13*(n-2)):(13*(n-1)),:);
end

Bq = zeros(13*(length(t)-2),12*(length(t)-2));

n = 1;

for i = 1:9
   Bq((1+13*(n-1)):13*n, (1+12*(n-1)):12*n) = Bd(:,:,i);
   for j = 1:9
       if j > n
           Bq((1+13*(j-1)):13*j, (1+12*(n-1)):12*n) = Aq((1+13*(j-1)):13*j,:)*Bd(:,:,i);          
       end 
   end
   n = n + 1;
end

G(1:end-1,1:end-1) = 2.0*(Bq'*L*Bq + K);
C(1:end-1,1) = 2*Bq'*L*(Aq*x0 - L*x_ref);
C(end,1) = x0'*Aq'*L*Aq*x0 + x_ref'*L*x_ref - x0'*Aq'*L*x_ref;
toc