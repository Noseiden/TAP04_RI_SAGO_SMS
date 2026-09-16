function T = T_DH(a, alpha, S, theta)
T = [
    cos(theta),              -sin(theta),             0,             a;
    cos(alpha)*sin(theta),    cos(alpha)*cos(theta),  -sin(alpha),   -S*sin(alpha);
    sin(alpha)*sin(theta),    sin(alpha)*cos(theta),   cos(alpha),    S*cos(alpha);
    0,                        0,                       0,             1
];
end