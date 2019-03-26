classdef MPC_Position_Controller_class < handle
   properties
       % ===================================== 
       % Constants
       % =====================================
       g = 9.81; % gravity 
       h % height of the robot
        
      % =======================================  
      % Gurobi Variables
      % =======================================
     
      sense; % Inequality signs
      rhs;   % right hand side of linear inequalities
      C;     % Linear inequality constraints on decision variables 
      Q;   % Quadratic cost matrix
      H;   % Linear cost matrix
      lb;  % lower bounds for decision variables
      ub;  % upper bounds for decision variables

       
      % =================================  
      % MPC Variables
      % =================================
      
      dt
      np
      
      T
      dT
      ddT
      
      Q_reg
      
      
   end
   
   methods
       function obj = MPC_Position_Controller_class(NP, DT, height)
           obj.dt = DT;
           obj.np = NP;
           obj.h = height;
           
           obj.T = [1, obj.dt, obj.dt^2, obj.dt^3, obj.dt^4];
           obj.dT = [0, 1, 2.0*obj.dt, 3*obj.dt^2, 4*obj.dt^3];
           obj.ddT = [0, 0, 2.0, 6.0*obj.dt, 12.0*obj.dt^2];
           
           % Create linear constraint matrix
           obj.C = zeros(13*obj.np+4,15*obj.np);
           for i = 1:obj.np
               obj.C(1+13*(i-1),1+15*(i-1)) = -1.0;
               obj.C(2+13*(i-1),2+15*(i-1)) = -1.0;

               obj.C(3+13*(i-1),6+15*(i-1)) = -1.0;
               obj.C(4+13*(i-1),7+15*(i-1)) = -1.0;

               obj.C(5+13*(i-1),11+15*(i-1)) = -1.0;
               obj.C(6+13*(i-1),11+15*(i-1)) = 1.0;

               obj.C(7+13*(i-1),12+15*(i-1)) = -1.0;
               obj.C(8+13*(i-1),12+15*(i-1)) = 1.0;

               obj.C(9+13*(i-1),13+15*(i-1)) = -1.0;
               obj.C(10+13*(i-1),13+15*(i-1)) = 1.0;

               obj.C(11+13*(i-1),14+15*(i-1)) = -1.0;
               obj.C(12+13*(i-1),14+15*(i-1)) = 1.0;

               obj.C(13+13*(i-1),11+15*(i-1):14+15*(i-1)) = ones(1,4);
               obj.C(13+13*(i-1),end) = -1;

               obj.C(14+13*(i-1),1+15*(i-1):5+15*(i-1)) = obj.T;
               obj.C(15+13*(i-1),1+15*(i-1):5+15*(i-1)) = obj.dT;

               obj.C(16+13*(i-1),6+15*(i-1):10+15*(i-1)) = obj.T;
               obj.C(17+13*(i-1),6+15*(i-1):10+15*(i-1)) = obj.dT;
           end
           
           % Bounds on the decision variables
           obj.lb = zeros(15*obj.np,1);
           LB = [-5000.0*ones(10,1); 0.0; 0.0; 0.0; 0.0; 1.0];
           
           obj.ub = zeros(15*obj.np,1);
           UB = [5000.0*ones(10,1); 1.0; 1.0; 1.0; 1.0; 1.0];
           
           for i = 1:obj.np
              obj.lb(1+15*(i-1):15+15*(i-1),1) = LB;
              obj.ub(1+15*(i-1):15+15*(i-1),1) = UB;
           end
           
           % vector of signs of the inequalities
           signs = zeros(13*obj.np+4,1);
           sign = ['='; '='; '='; '='; '<'; '<'; '<'; '<'; '<'; '<'; '<'; '<'; '=';];
           for j = 1:obj.np
              signs(1+13*(j-1):13+13*(j-1)) = sign; 
           end
           signs(end-3:end,1) = ['='; '=';'=';'='];
           obj.sense = char(signs);
           obj.rhs = zeros(13*obj.np+4,1);
           
           % Quadratic cost
           obj.Q = zeros(15*obj.np,15*obj.np);
           
           % Linear Cost Matrix
           obj.H = zeros(15*obj.np,1);
           
           % Quadratic Regulizer Cost
           q_pos = [obj.T, zeros(1,10); zeros(1,5), obj.T, zeros(1,5)];
           q_pos = q_pos'*q_pos;
    
           q_vel = [obj.dT, zeros(1,10); zeros(1,5), obj.dT, zeros(1,5)];
           q_vel = q_vel'*q_vel;
           
           obj.Q_reg = q_pos + q_vel;
           
       end
       
       function [X] = update(obj, x_ref, p1, p2, p3, p4, C1, C2, C3, C4) 
           tic
           % initial condition
           obj.C(1,end) = x_ref(1,1);
           obj.C(2,end) = x_ref(2,1);
           obj.C(3,end) = x_ref(3,1);
           obj.C(4,end) = x_ref(4,1);
           
           for i = 1:obj.np
               obj.C(6+13*(i-1),15+15*(i-1)) = -C1(i+1);
               obj.C(8+13*(i-1),15+15*(i-1)) = -C2(i+1);
               obj.C(10+13*(i-1),15+15*(i-1)) = -C3(i+1);
               obj.C(12+13*(i-1),15+15*(i-1)) = -C4(i+1);

               % Loading cost
               lambda_star = 1.0/(C1(i+1)+C2(i+1)+C3(i+1)+C4(i+1));
               Q1 = zeros(4,15);
               Q1(:,11:14) = eye(4);
               Q1(:,end) = -lambda_star*[C1(i+1);C2(i+1);C3(i+1);C4(i+1)];
               Q1 = Q1'*Q1;

               % Dyanamics cost
               Q2 = [(obj.ddT - (obj.g/obj.h)*obj.T), zeros(1,5), (obj.g/obj.h)*[p1(1,i+1),p2(1,i+1),p3(1,i+1),p4(1,i+1)], 0;...
                     zeros(1,5), (obj.ddT - (obj.g/obj.h)*obj.T),  (obj.g/obj.h)*[p1(2,i+1),p2(2,i+1),p3(2,i+1),p4(2,i+1)], 0];
               Q2 = Q2'*Q2;

               obj.Q(1+(i-1)*15:15+(i-1)*15,1+(i-1)*15:15+(i-1)*15) = Q1+Q2+obj.Q_reg;

               % Linear cost
               obj.H(1+(i-1)*15:15+(i-1)*15,1) = [-2.0*x_ref(1,(i+1))*obj.T-2.0*x_ref(2,(i+1))*obj.dT, ...
                                                  -2.0*x_ref(3,(i+1))*obj.T-2.0*x_ref(4,(i+1))*obj.dT, ...
                                                  zeros(1,4), x_ref(:,(i+1))'*x_ref(:,(i+1))];
               
           end
           
           % final condition
           obj.C(end-3,end) = -x_ref(1,end);
           obj.C(end-2,end) = -x_ref(2,end);
           obj.C(end-1,end) = -x_ref(3,end);
           obj.C(end,end) = -x_ref(4,end);
           
           %%%%% Updating current model 
           model.Q = sparse(obj.Q);
           model.A = sparse(obj.C);
           model.obj = obj.H;
           model.lb = obj.lb;
           model.ub = obj.ub;
           model.sense = obj.sense;

           % Solve the MPC
           params.outputflag = 0;
           res = gurobi(model, params);
           
           X = res.x;
           toc
       end
       
   end
    
end