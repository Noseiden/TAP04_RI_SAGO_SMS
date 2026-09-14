function KUKA_KR10_IK_Taller
clc;
close all;

%% PARAMETROS DH APROXIMADOS
% Cambie solamente estos valores cuando tenga la tabla DH correcta.
% Unidades: mm y radianes.

a01 = 0;
a12 = 25;
a23 = 560;
a34 = 35;
a45 = 0;
a56 = 0;

S1 = 400;
S2 = 0;
S3 = 0;
S4 = 515;
S5 = 0;
S6 = 80;

alpha01 = deg2rad(0);
alpha12 = deg2rad(-90);
alpha23 = deg2rad(0);
alpha34 = deg2rad(90);
alpha45 = deg2rad(90);
alpha56 = deg2rad(90);

offset1 = deg2rad(0);
offset2 = deg2rad(-90);
offset3 = deg2rad(180);
offset4 = deg2rad(180);
offset5 = deg2rad(180);
offset6 = deg2rad(180);

% Columnas: [a_ij, alpha_ij, S_i, offset_theta_i]
DH_const = [
    a01, alpha01, S1, offset1;
    a12, alpha12, S2, offset2;
    a23, alpha23, S3, offset3;
    a34, alpha34, S4, offset4;
    a45, alpha45, S5, offset5;
    a56, alpha56, S6, offset6
];

% Limites del datasheet para A1 hasta A6.
qLim = deg2rad([
    -170,  170;
    -190,   45;
    -120,  156;
    -185,  185;
    -120,  120;
    -350,  350
]);

robot.DH = DH_const;
robot.qLim = qLim;

%% TRAYECTORIA CARTESIANA
n = 40;
[q06, S6_dir, a67_dir] = crearTrayectoria(n, robot);
verificarVolumen(q06);

%% CINEMATICA INVERSA
SlnSet = nan(n, 6, 8);

for i = 1:n
    Q = cinematicaInversa(q06(i,:).', S6_dir(i,:).', ...
        a67_dir(i,:).', robot);

    for k = 1:8
        SlnSet(i,:,k) = Q(:,k).';
    end
end

%% FILTRADO POR LIMITES
SlnSetLim = SlnSet;

for k = 1:8
    for i = 1:n
        q = SlnSet(i,:,k).';

        if ~solucionValida(q, robot.qLim)
            SlnSetLim(i,:,k) = nan(1,6);
        end
    end
end

%% FILTRADO POR CAPAS
capasValidas = false(1,8);

for k = 1:8
    capa = SlnSetLim(:,:,k);
    capasValidas(k) = isreal(capa) && all(isfinite(capa(:)));
end

SlnSetVer = SlnSetLim(:,:,capasValidas);
indicesValidos = find(capasValidas);

fprintf('Tamaño de SlnSet: %d x %d x %d\n', size(SlnSet));
fprintf('Soluciones del primer punto en radianes:\n');
disp(squeeze(SlnSet(1,:,:)));
fprintf('Capas validas para toda la trayectoria: ');
fprintf('%d ', indicesValidos);
fprintf('\n');

if isempty(indicesValidos)
    error('No existe una configuracion valida para toda la trayectoria.');
end

%% SELECCION DE CONFIGURACION
costos = zeros(1, size(SlnSetVer,3));

for k = 1:size(SlnSetVer,3)
    Qk = unwrap(SlnSetVer(:,:,k), [], 1);
    costos(k) = sum(sum(abs(diff(Qk,1,1))));
end

[~, indiceLocal] = min(costos);
configuracionElegida = indicesValidos(indiceLocal);
Qtray = SlnSetVer(:,:,indiceLocal);

fprintf('Configuracion seleccionada: capa %d\n', configuracionElegida);

%% VERIFICACION CON CINEMATICA DIRECTA
errorPos = zeros(n,1);
errorOri = zeros(n,1);

for i = 1:n
    [Tcalc, ~] = cinematicaDirecta(Qtray(i,:).', robot);
    Rdes = construirOrientacion(S6_dir(i,:).', a67_dir(i,:).');
    Rerr = Tcalc(1:3,1:3).' * Rdes;

    errorPos(i) = norm(Tcalc(1:3,4) - q06(i,:).');
    valor = (trace(Rerr) - 1) / 2;
    errorOri(i) = acos(max(-1, min(1, valor)));
end

fprintf('Error maximo de posicion: %.6e mm\n', max(errorPos));
fprintf('Error maximo de orientacion: %.6e rad\n', max(errorOri));

assignin('base', 'SlnSet', SlnSet);
assignin('base', 'SlnSetLim', SlnSetLim);
assignin('base', 'SlnSetVer', SlnSetVer);
assignin('base', 'Qtray', Qtray);
assignin('base', 'q06', q06);
assignin('base', 'S6_dir', S6_dir);
assignin('base', 'a67_dir', a67_dir);

%% ANIMACION
animarRobot(Qtray, q06, robot);
end


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


function verificarVolumen(q06)
dentro = q06(:,1) >= 800 & q06(:,1) <= 1300 & ...
         q06(:,2) >= -500 & q06(:,2) <= 500 & ...
         q06(:,3) >= 0 & q06(:,3) <= 1300;

if ~all(dentro)
    error('La trayectoria tiene puntos fuera del volumen indicado.');
end
end


function Q = cinematicaInversa(pTool, S6_dir, a67_dir, robot)
DH = robot.DH;
Q = nan(6,8);
tol = 1e-9;

R06 = construirOrientacion(S6_dir, a67_dir);
pC = pTool - abs(DH(6,3))*R06(:,3);

px = pC(1);
py = pC(2);
pz = pC(3);
rho = hypot(px, py);

if rho < tol
    return;
end

theta1a = atan2(py, px);
theta1b = wrapToPiLocal(theta1a + pi);
theta1Set = [theta1a, theta1b];

a12 = DH(2,1);
a23 = DH(3,1);
x3 = DH(4,1);
y3 = -DH(4,3);
S1 = DH(1,3);

columna = 0;

for i1 = 1:2
    theta1 = theta1Set(i1);

    Bpos = [
        px*cos(theta1) + py*sin(theta1) - a12;
        pz - S1
    ];

    A = 2*a23*x3;
    B = -2*a23*y3;
    D = a23^2 + x3^2 + y3^2 - dot(Bpos,Bpos);
    amplitud = hypot(A,B);

    if amplitud < tol
        columna = columna + 4;
        continue;
    end

    argumento = -D/amplitud;

    if argumento < -1-tol || argumento > 1+tol
        columna = columna + 4;
        continue;
    end

    argumento = max(-1, min(1, argumento));
    fase = atan2(B,A);
    delta = acos(argumento);
    theta3Set = [fase + delta, fase - delta];

    for i3 = 1:2
        theta3 = wrapToPiLocal(theta3Set(i3));

        m11 = a23 + x3*cos(theta3) - y3*sin(theta3);
        m12 = -x3*sin(theta3) - y3*cos(theta3);
        M = [m11, m12; m12, -m11];

        if abs(det(M)) < tol
            columna = columna + 2;
            continue;
        end

        cs2 = M\Bpos;
        cs2 = cs2/norm(cs2);
        theta2 = atan2(cs2(2), cs2(1));

        R03 = eye(3);
        theta123 = [theta1, theta2, theta3];

        for j = 1:3
            Tj = T_DH(DH(j,1), DH(j,2), DH(j,3), theta123(j));
            R03 = R03*Tj(1:3,1:3);
        end

        R36 = R03.'*R06;
        c5 = max(-1, min(1, R36(2,3)));
        theta5Set = [acos(c5), -acos(c5)];

        for i5 = 1:2
            columna = columna + 1;
            theta5 = theta5Set(i5);
            s5 = sin(theta5);

            if abs(s5) < 1e-7
                continue;
            end

            theta6 = atan2(R36(2,2)/s5, -R36(2,1)/s5);
            theta4 = atan2(R36(3,3)/s5, R36(1,3)/s5);
            thetaDH = [theta1; theta2; theta3; theta4; theta5; theta6];
            q = thetaDH - DH(:,4);

            for j = 1:6
                q(j) = representacionArticular(q(j), robot.qLim(j,:));
            end

            [Tver, ~] = cinematicaDirecta(q, robot);
            ePos = norm(Tver(1:3,4) - pTool);
            eOri = norm(Tver(1:3,1:3) - R06, 'fro');

            if isreal(q) && all(isfinite(q)) && ePos < 1e-5 && eOri < 1e-7
                Q(:,columna) = q;
            end
        end
    end
end
end


function R = construirOrientacion(S6_dir, a67_dir)
z = S6_dir/norm(S6_dir);
x = a67_dir - z*dot(z,a67_dir);

if norm(x) < 1e-10
    error('Los vectores de orientacion son paralelos.');
end

x = x/norm(x);
y = cross(z,x);
y = y/norm(y);
R = [x, y, z];
end


function valido = solucionValida(q, qLim)
if ~isreal(q) || any(~isfinite(q))
    valido = false;
    return;
end

tol = 1e-9;
valido = all(q >= qLim(:,1)-tol & q <= qLim(:,2)+tol);
end


function q = representacionArticular(q, limite)
qBase = wrapToPiLocal(q);
candidatos = qBase + 2*pi*(-2:2);
dentro = candidatos >= limite(1) & candidatos <= limite(2);

if any(dentro)
    validos = candidatos(dentro);
    [~, indice] = min(abs(validos));
    q = validos(indice);
else
    q = qBase;
end
end


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


function T = T_DH(a, alpha, S, theta)
T = [
    cos(theta),              -sin(theta),             0,             a;
    cos(alpha)*sin(theta),    cos(alpha)*cos(theta),  -sin(alpha),   -S*sin(alpha);
    sin(alpha)*sin(theta),    sin(alpha)*cos(theta),   cos(alpha),    S*cos(alpha);
    0,                        0,                       0,             1
];
end


function animarRobot(Qtray, q06, robot)
n = size(Qtray,1);
fig = figure('Color','w', 'Name','KUKA KR10 R1100-2');
ax = axes(fig);
hold(ax,'on');
grid(ax,'on');
axis(ax,'equal');
view(ax,35,25);

plot3(ax, q06(:,1), q06(:,2), q06(:,3), '--', ...
    'Color',[0.65 0.65 0.65], 'LineWidth',1);

hRobot = plot3(ax, nan, nan, nan, '-o', ...
    'Color',[0.95 0.35 0.05], 'MarkerFaceColor',[0.15 0.15 0.15], ...
    'LineWidth',4, 'MarkerSize',6);
hTray = animatedline(ax, 'Color',[0 0.35 0.85], 'LineWidth',2);
hX = plot3(ax,nan,nan,nan,'r','LineWidth',2);
hY = plot3(ax,nan,nan,nan,'g','LineWidth',2);
hZ = plot3(ax,nan,nan,nan,'b','LineWidth',2);

xlabel(ax,'X [mm]');
ylabel(ax,'Y [mm]');
zlabel(ax,'Z [mm]');
title(ax,'Trayectoria del KUKA KR10 R1100-2');
xlim(ax,[-1250 1350]);
ylim(ax,[-1250 1250]);
zlim(ax,[-200 1600]);

for i = 1:n
    [T06, Tglobal] = cinematicaDirecta(Qtray(i,:).', robot);
    puntos = zeros(3,7);

    for j = 1:6
        puntos(:,j+1) = Tglobal(1:3,4,j);
    end

    set(hRobot, 'XData',puntos(1,:), 'YData',puntos(2,:), ...
        'ZData',puntos(3,:));
    addpoints(hTray, T06(1,4), T06(2,4), T06(3,4));

    p = T06(1:3,4);
    R = T06(1:3,1:3);
    escala = 100;
    set(hX,'XData',[p(1),p(1)+escala*R(1,1)], ...
        'YData',[p(2),p(2)+escala*R(2,1)], ...
        'ZData',[p(3),p(3)+escala*R(3,1)]);
    set(hY,'XData',[p(1),p(1)+escala*R(1,2)], ...
        'YData',[p(2),p(2)+escala*R(2,2)], ...
        'ZData',[p(3),p(3)+escala*R(3,2)]);
    set(hZ,'XData',[p(1),p(1)+escala*R(1,3)], ...
        'YData',[p(2),p(2)+escala*R(2,3)], ...
        'ZData',[p(3),p(3)+escala*R(3,3)]);

    drawnow;
    pause(0.04);
end
end


function angulo = wrapToPiLocal(angulo)
angulo = mod(angulo + pi, 2*pi) - pi;
end
