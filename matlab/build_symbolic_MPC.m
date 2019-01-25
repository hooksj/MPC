clear all; close all; clc;

r11 = sym('r11');
r12 = sym('r12');
r21 = sym('r21');
r22 = sym('r22');

p11 = sym('p11');
p12 = sym('p12');
p13 = sym('p13');

p21 = sym('p21');
p22 = sym('p22');
p23 = sym('p23');

p31 = sym('p31');
p32 = sym('p32');
p33 = sym('p33');

p41 = sym('p41');
p42 = sym('p42');
p43 = sym('p43');

p51 = sym('p51');
p52 = sym('p52');
p53 = sym('p53');

p61 = sym('p61');
p62 = sym('p62');
p63 = sym('p63');

p71 = sym('p71');
p72 = sym('p72');
p73 = sym('p73');

p81 = sym('p81');
p82 = sym('p82');
p83 = sym('p83');

p91 = sym('p91');
p92 = sym('p92');
p93 = sym('p93');

p101 = sym('p101');
p102 = sym('p102');
p103 = sym('p103');




% Function to skew semetric cross product matrix
skew = @(x) [0.0 -x(3) x(2); x(3) 0.0 -x(1); -x(2) x(1) 0.0];

R = [r11 r12 0.0; r21 r22 0.0; 0.0 0.0 1.0];

dt = 0.05;

mass = 25.0;
I_inv = eye(3)*0.5;

A = [zeros(3,6) R, zeros(3,4);
     zeros(3,9) eye(3), zeros(3,1);
     zeros(6,13);
     zeros(1,12), 1.0];


B = [zeros(6,12);
     I_inv*skew([p11,p12,p13]), I_inv*skew([p21,p22,p23]),...
     I_inv*skew([p31,p32,p33]), I_inv*skew([p41,p42,p43]);
     eye(3)/mass, eye(3)/mass, eye(3)/mass, eye(3)/mass;
     zeros(1,12)];

 
M = [A, B; zeros(12,13), zeros(12,12)];
 
eM = eye(size(M,1)) + M*dt + 0.5*M*M*dt*dt;

Ad = eM(1:13,1:13);


Bd = eM(1:13,14:end);

Aq = sym(zeros(13*10,13));

Aq(1:13,:) = Ad;

for i = 1:9
   Aq((13*i+1):13*(i+1),:) =  Aq((13*(i-1)+1):13*(i),:)*Ad;
end

Bq = sym(zeros(13*9,12*9));

n = 1;

for i = 1:9
   Bq((1+13*(n-1)):13*n, (1+12*(n-1)):12*n) = Bd;
   for j = 1:9
       if j > n
           Bq((1+13*(j-1)):13*j, (1+12*(n-1)):12*n) = Aq((1+13*(j-1)):13*j,:)*Bd;
           
       end 
   end
   n = n + 1;
end

rotation = [r11, r12, r21, r22];
P = [p11, p12, p13, p21, p22, p23, p31, p32, p33, p41, p42, p43];

matlabFunction(Aq,'file','A_matrix','vars',{rotation});
matlabFunction(Bq,'file','B_matrix','vars',{rotation,P});
