function [T06, Tglobal] = cinematicaDirecta(q, robot)
DH = robot.DH;
Tglobal = zeros(4,4,6);
T06 = eye(4);

for i = 1:6
    theta = q(i) + DH(i,4);
    Ti = T_DH(DH(i,1), DH(i,2), DH(i,3), theta);
    T06 = T06*Ti;
    Tglobal(:,:,i) = T06;
end
end