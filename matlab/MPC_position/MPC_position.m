clear all; clc; close all;
tic
dt = 0.05;
g = 9.81;
h = 0.5;

p1_init = [0.25; 0.25];
p2_init = [-0.25; 0.25];
p3_init = [-0.25; -0.25];
p4_init = [0.25; -0.25];

t1 = [0.01, 0.21];
t2 = [0.21, 0.41];
t3 = [0.01, 0.21];
t4 = [0.21, 0.41];

x0 = [0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

C_init = [1.0, 1.0, 1.0, 1.0];

vel = 0.5;
rot_vel = 0.0;

p1 = [0.25, 0.25, 0.25, 0.25, 0.25, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35;
      0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25];
C1 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0];

p2 = [-0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.05, -0.05;
      0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25];
C2 = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0];

p3 = [-0.25, -0.25, -0.25, -0.25, -0.25, -0.15, -0.15, -0.15, -0.15, -0.15, -0.15;
      -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25];
C3 = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0];

p4 = [0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.45, 0.45;
      -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25, -0.25];
C4 = [1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0];

x_ref = [0.0, 0.0125, 0.025, 0.0325, 0.05, 0.075, 0.1, 0.125, 0.15, 0.175, 0.2;
         0.25, 0.25, 0.25, 0.25, 0.25, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
     
T = [1, dt, dt^2, dt^3, dt^4];
dT = [0, 1, 2.0*dt, 3*dt^2, 4*dt^3];
ddT = [0, 0, 2.0, 6.0*dt, 12.0*dt^2];

Q = zeros(150,150);
H = zeros(150,1);

for i = 2:11
    % Acceleration cost
    Q1 = 0.00*[ddT, zeros(1,10); zeros(1,5), ddT, zeros(1,5)]'*[ddT, zeros(1,10); zeros(1,5), ddT, zeros(1,5)];

    % Loading cost
    lambda_star = 1.0/(C1(i)+C2(i)+C3(i)+C4(i));
    Q2 = zeros(4,15);
    Q2(:,11:14) = eye(4);
    Q2(:,end) = -lambda_star*[C1(i);C2(i);C3(i);C4(i)];
    Q2 = Q2'*Q2;

    % Dyanamics cost
    Q3 = [(ddT - (g/h)*T), zeros(1,5), (g/h)*[p1(1,i),p2(1,i),p3(1,i),p4(1,i)], 0;...
          zeros(1,5), (ddT - (g/h)*T),  (g/h)*[p1(2,i),p2(2,i),p3(2,i),p4(2,i)], 0];
    Q3 = Q3'*Q3;

    % Path Regularization
    Q4 = [T, zeros(1,10); zeros(1,5), T, zeros(1,5)];
    Q4 = Q4'*Q4;
    
    Q5 = [dT, zeros(1,10); zeros(1,5), dT, zeros(1,5)];
    Q5 = Q5'*Q5;

    Q(1+(i-2)*15:15+(i-2)*15,1+(i-2)*15:15+(i-2)*15) = Q1+Q2+Q3+Q4+Q5;
    
    % Linear cost
    H(1+(i-2)*15:15+(i-2)*15,1) = [-2.0*x_ref(1,(i-1))*T-2.0*x_ref(2,(i-1))*dT, ...
                                   -2.0*x_ref(3,(i-1))*T-2.0*x_ref(4,(i-1))*dT, ...
                                   zeros(1,4), x_ref(:,(i-1))'*x_ref(:,(i-1))];
end


first = 1.0;

% Constraints
C = zeros(134,150);

for i = 1:10
    if first == 1.0
        C(1,15) = x_ref(1,1);
        C(2,15) = x_ref(2,1);
        C(3,15) = x_ref(3,1);
        C(4,15) = x_ref(4,1);
        first = 0.0;
    end
    
    C(1+13*(i-1),1+15*(i-1)) = -1.0;
    C(2+13*(i-1),2+15*(i-1)) = -1.0;

    C(3+13*(i-1),6+15*(i-1)) = -1.0;
    C(4+13*(i-1),7+15*(i-1)) = -1.0;

    C(5+13*(i-1),11+15*(i-1)) = -1.0;
    C(6+13*(i-1),11+15*(i-1)) = 1.0;
    C(6+13*(i-1),15+15*(i-1)) = -C1(i);

    C(7+13*(i-1),12+15*(i-1)) = -1.0;
    C(8+13*(i-1),12+15*(i-1)) = 1.0;
    C(8+13*(i-1),15+15*(i-1)) = -C2(i);

    C(9+13*(i-1),13+15*(i-1)) = -1.0;
    C(10+13*(i-1),13+15*(i-1)) = 1.0;
    C(10+13*(i-1),15+15*(i-1)) = -C3(i);

    C(11+13*(i-1),14+15*(i-1)) = -1.0;
    C(12+13*(i-1),14+15*(i-1)) = 1.0;
    C(12+13*(i-1),15+15*(i-1)) = -C4(i);

    C(13+13*(i-1),11+15*(i-1):14+15*(i-1)) = ones(1,4);
    C(13+13*(i-1),15+15*(i-1)) = -1;

    C(14+13*(i-1),1+15*(i-1):5+15*(i-1)) = T;
    C(15+13*(i-1),1+15*(i-1):5+15*(i-1)) = dT;
    
    C(16+13*(i-1),6+15*(i-1):10+15*(i-1)) = T;
    C(17+13*(i-1),6+15*(i-1):10+15*(i-1)) = dT;
end

C(131,end) = -x_ref(1,end);
C(132,end) = -x_ref(2,end);
C(133,end) = -x_ref(3,end);
C(134,end) = -x_ref(4,end);

signs = ['='; '='; '='; '='; '<'; '<'; '<'; '<'; '<'; '<'; '<'; '<'; '=';];
sense = [signs; signs; signs; signs; signs; signs; signs; signs; signs; signs; '='; '=';'=';'='];
rhs = zeros(132,1);

lb = [-5000.0*ones(10,1); 0.0; 0.0; 0.0; 0.0; 1.0];
ub = [5000.0*ones(10,1); 1.0; 1.0; 1.0; 1.0; 1.0];

LB = [lb; lb; lb; lb; lb; lb; lb; lb; lb; lb];
UB = [ub; ub; ub; ub; ub; ub; ub; ub; ub; ub];

model.Q = sparse(Q);
model.A = sparse(C);
model.obj = H;
model.lb = LB;
model.ub = UB;
model.sense = sense;


% Solve the MPC
params.outputflag = 0;

res = gurobi(model, params);
toc

controller = MPC_Position_Controller_class(10, dt, h);
X = controller.update(x_ref, p1, p2, p3, p4, C1, C2, C3, C4);

P = [p1; p2; p3; p4];
Contact = [C1; C2; C3; C4];

plot_solution(X, x_ref, P, Contact, dt, 10)



