function [q06, S6_dir, a67_dir] = crearTrayectoria(n, robot)
% Esta trayectoria articular solo genera una trayectoria cartesiana alcanzable.
t = linspace(0, 2*pi, n).';
qRef = zeros(n,6);

qRef(:,1) = deg2rad(15*sin(t));
qRef(:,2) = deg2rad(30 + 4*sin(t));
qRef(:,3) = deg2rad(-30 + 8*cos(t));
qRef(:,4) = deg2rad(10*sin(t));
qRef(:,5) = deg2rad(30 + 5*cos(t));
qRef(:,6) = deg2rad(20*sin(2*t));

q06 = zeros(n,3);
S6_dir = zeros(n,3);
a67_dir = zeros(n,3);

for i = 1:n
    [T, ~] = cinematicaDirecta(qRef(i,:).', robot);
    q06(i,:) = T(1:3,4).';
    a67_dir(i,:) = T(1:3,1).';
    S6_dir(i,:) = T(1:3,3).';
end
end