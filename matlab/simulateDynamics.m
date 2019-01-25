function dx = simulateDynamics(t, x, F, v1, v2, v3, v4, inv_I, mass)

    % Statespace:
    % x = [position, euler angles, velocity, angular rate]

    A = zeros(13,13);
    A(1:3,7:9) = eye(3);
    A(4:6,10:12) = rot(x(4:6));
    A(9,13) = 1.0;
    
    B = zeros(13,12);
    B(7:9,:) = [eye(3)*1/mass eye(3)*1/mass eye(3)*1/mass eye(3)*1/mass];
    B(10:12,:) = [inv_I*skew(v1) inv_I*skew(v2)... 
                  inv_I*skew(v3) inv_I*skew(v4)];
              
    g = zeros(12,1);
    g(9,1) = 9.81;
      
    dx = A*x + B*F;
      
    function S = skew(x)
        S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    end
  
    function R = rot(x)
        R = [cos(x(3))/cos(x(2))  sin(x(3))/cos(x(2))  0;
                 -sin(x(3))        cos(x(3))     0;
             cos(x(3))*tan(x(2))  sin(x(3))*tan(x(2))  1];
    end
end