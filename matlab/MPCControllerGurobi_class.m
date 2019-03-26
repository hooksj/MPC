classdef MPCControllerGurobi_class < handle
   properties
       % ===================================== 
       % Constants
       % =====================================
       g = [0.0; 0.0; 9.81]; % gravity vector   
      
       mass = 43.0;   % Mass of the robot
      
       I = [0.5 0.0 0.0;...   % Inertia matrix
            0.0  2.1 0.0;...
            0.0  0.0 2.1];
       
       I_inv = [1.0/0.5 0.0 0.0;...   % Inertia matrix
                0.0  1.0/2.1 0.0;...
                0.0  0.0 1.0/2.1];
       
       mu = 0.6;
       
       lf = [-666.0; -666.0; 10.0]; % lower bound for ground reaction force
       uf = [666.0; 666.0; 666.0];  % upper bound for ground reaction force
       
        
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
      
      alpha = 0.0001 % gain on total ground reaction force
      
      
      np;  % Number of subdivisions of the time horizon
      dt;  % Discritized time step
      
      L;   % Weighting matrix for tracking error
      K;   % Weighting matrix for total control effort
      
      A;   % Continous time state matrix
      Ad;  % Discrete time state matrix
      
      B;   % Continous time control input matrix
      Bd;  % Discrete time control input matrix
      
      M = zeros(25,25);   % State Transition Matrix
      Ident = eye(25);
      
      % Condensed formulation
      Aqp;
      Bqp;
      
      
   end
   
   methods
       function obj = MPCControllerGurobi_class(NP, DT)
           obj.dt = DT;
           obj.np = NP;
           
           % Create linear constraint matrix
           obj.C = zeros(16*NP,12*NP+1);
           obj.C(1:4,1:3) = [-1.0, 0.0, obj.mu; 1.0, 0.0, obj.mu;...
                             0.0, -1.0, obj.mu; 0.0, 1.0, obj.mu];
                        
           for i = 1:(NP*4-1)
              obj.C(4*i+1:4*i+4,3*i+1:3*i+3) = [-1.0, 0.0, obj.mu; 1.0, 0.0, obj.mu;...
                                                 0.0, -1.0, obj.mu; 0.0, 1.0, obj.mu];
           end
           
           
           obj.lb = zeros(12*NP+1,1);
           obj.lb(end,1) = 1.0;
           
           obj.ub = zeros(12*NP+1,1);
           obj.ub(end,1) = 1.0;
           
           % vector of signs of the inequalities
           signs = zeros(16*NP,1);
           for i = 1:16*NP
              signs(i,1) = '>'; 
           end
           obj.sense = char(signs);
           
           % Weighting matrix for tracking errors
           obj.L = eye(13*NP);
           
           % z weight
           for i = 1:NP
              obj.L(13*(i-1)+3,13*(i-1)+3) = 50.0; 
           end
           
           % roll weight
           for i = 1:NP
              obj.L(13*(i-1)+4,13*(i-1)+4) = 1.0; 
           end
           
           % pitch weight
           for i = 1:NP
              obj.L(13*(i-1)+5,13*(i-1)+5) = 1.0; 
           end
           
           % x vel weight
           for i = 1:NP
              obj.L(13*(i-1)+7,13*(i-1)+7) = 1.0; 
           end
           
           % y vel weight
           for i = 1:NP
              obj.L(13*(i-1)+8,13*(i-1)+8) = 20.0; 
           end
           
           % Weighting matrix for total ground reaction forces
           obj.K = obj.alpha*eye(12*NP);
           
           % State matrix
           obj.A = zeros(13,13);
           obj.A(1:3,7:9) = eye(3);
           obj.A(9,13) = 1.0;
           
           obj.Ad = zeros(13,13);
           
           % Control input matrix
           obj.B = zeros(13,12);
           obj.B(7:9,:)= [eye(3)/obj.mass, eye(3)/obj.mass, eye(3)/obj.mass, eye(3)/obj.mass];
           
           obj.Bd = zeros(13,12,NP);
           
           % Condensed Formulation Matrices
           obj.Aqp = zeros(13*NP,13);
           obj.Bqp = zeros(13*NP,12*NP);
           
           % Quadratic Cost Matrix
           obj.Q = zeros(12*NP+1, 12*NP+1);
           
           % Linear Cost Matrix
           obj.H = eye(12*NP+1,1);
           
           
       end
       
       function S = skew(obj, x)
          S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
       end
       
       function w = inv_skew(obj, R)
           w = [-R(2,3); R(1,3); -R(1,2)];
       end
       
       function R = rot(obj, x)
           R = [cos(x), -sin(x) 0.0; sin(x) cos(x) 0.0; 0.0 0.0 1];
       end
       
       function setBounds(obj,C)
          for i = 1:obj.np
             if C(i,1) == 1.0
                obj.lb(12*(i-1)+1:12*(i-1)+3,1) = obj.lf; 
                obj.ub(12*(i-1)+1:12*(i-1)+3,1) = obj.uf; 
             else
                obj.lb(12*(i-1)+1:12*(i-1)+3,1) = [0.0;0.0;0.0];
                obj.ub(12*(i-1)+1:12*(i-1)+3,1) = [0.0;0.0;0.0];
             end
             
             if C(i,2) == 1.0
                obj.lb(12*(i-1)+4:12*(i-1)+6,1) = obj.lf; 
                obj.ub(12*(i-1)+4:12*(i-1)+6,1) = obj.uf; 
             else
                obj.lb(12*(i-1)+4:12*(i-1)+6,1) = [0.0;0.0;0.0];
                obj.ub(12*(i-1)+4:12*(i-1)+6,1) = [0.0;0.0;0.0];
             end
             
             if C(i,3) == 1.0
                obj.lb(12*(i-1)+7:12*(i-1)+9,1) = obj.lf; 
                obj.ub(12*(i-1)+7:12*(i-1)+9,1) = obj.uf; 
             else
                obj.lb(12*(i-1)+7:12*(i-1)+9,1) = [0.0;0.0;0.0];
                obj.ub(12*(i-1)+7:12*(i-1)+9,1) = [0.0;0.0;0.0];
             end
             
             if C(i,4) == 1.0
                obj.lb(12*(i-1)+10:12*(i-1)+12,1) = obj.lf; 
                obj.ub(12*(i-1)+10:12*(i-1)+12,1) = obj.uf; 
             else
                obj.lb(12*(i-1)+10:12*(i-1)+12,1) = [0.0;0.0;0.0];
                obj.ub(12*(i-1)+10:12*(i-1)+12,1) = [0.0;0.0;0.0];
             end
          end
       end
       
       function [F, current_state] = update(obj, x0, x_ref, yaw, v1, v2, v3, v4, C) 
           
           tic
           obj.setBounds(C);
           
           obj.A(4:6,10:12) = obj.rot(yaw);
           
           obj.B(10:12,:) = [obj.I_inv*obj.skew(v1(1:3,1)), obj.I_inv*obj.skew(v2(1:3,1)), ...
                             obj.I_inv*obj.skew(v3(1:3,1)), obj.I_inv*obj.skew(v4(1:3,1))];
                       
           obj.M(1:13,1:13) = obj.A;
           obj.M(1:13,14:end) = obj.B;

           % Discretize the state space by taking the matrix exponential
           % of the condenced state space matrix
           eM = obj.Ident + obj.M*obj.dt +0.5*obj.M*obj.M*obj.dt*obj.dt;

           obj.Ad = eM(1:13,1:13);

           obj.Bd(:,:,1) = eM(1:13,14:end);

           for i = 2:obj.np
               obj.M(10:12,14:end) = [obj.I_inv*obj.skew(v1(i*3-2:i*3,1)),...
                                      obj.I_inv*obj.skew(v2(i*3-2:i*3,1)), ...
                                      obj.I_inv*obj.skew(v3(i*3-2:i*3,1)),...
                                      obj.I_inv*obj.skew(v4(i*3-2:i*3,1))];

               eM = obj.Ident + obj.M*obj.dt +0.5*obj.M*obj.M*obj.dt*obj.dt;
               obj.Bd(:,:,i) = eM(1:13,14:end);
           end
           
           obj.Aqp(1:13,:) = obj.Ad;
           for i = 2:obj.np
              obj.Aqp((1+13*(i-1)):(13*i),:) =  obj.Ad*obj.Aqp((1+13*(i-2)):(13*(i-1)),:);
           end

           
           n = 1;

           for i = 1:obj.np
              obj.Bqp((1+13*(n-1)):13*n, (1+12*(n-1)):12*n) = obj.Bd(:,:,i);
              for j = n+1:obj.np
                  obj.Bqp((1+13*(j-1)):13*j, (1+12*(n-1)):12*n) = obj.Aqp((1+13*(j-1)):13*j,:)*obj.Bd(:,:,i);          
              end
              n = n + 1;
           end
           
           obj.Q(1:end-1,1:end-1) = (obj.Bqp'*obj.L*obj.Bqp + obj.K);
           obj.H(1:end-1,1) = 2.0*obj.Bqp'*obj.L*(obj.Aqp*x0 - x_ref);
           obj.H(end,1) = 1.0*(x0'*obj.Aqp'*obj.L*obj.Aqp*x0 + x_ref'*obj.L*x_ref - 2.0*x0'*obj.Aqp'*obj.L*x_ref);
           
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
           
           F = res.x(1:12); % Control effort
           
           toc
           
           % Simulate dynamics
           [t,state] = ode45(@(t,state) simulateDynamics_MPC(t,state,F, ...
               v1(1:3,1),v2(1:3,1),v3(1:3,1),v4(1:3,1),obj.I_inv, obj.mass), ...
                             [0 0.02], x0);
           current_state = state(end,:)';
           
       end
       
   end
    
end