clc;
close all;

%% PARAMETROS DH 
% Recodatorio: revisar matriz DH
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
n = 40; % puntos de la trayectoria
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