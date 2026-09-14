clear all
clc
close all
%% KUKA KR10 R1100-2 - Parte 5: Jacobiano analitico vs MoveIt2 (RobotState::getJacobian)
% DH modificado (Craig), misma estructura que Simulacion_dh.m
% Archivos que usa (en resultados/):
%   q_eval.csv, q_eval_etiquetas.csv  200 configuraciones aleatorias + todos los puntos de 4B y 4D (scripts/configuraciones_5.py)
%   J_moveit.csv                      Jacobiano 6x6 de MoveIt en cada configuracion (kr10_jacobiano_moveit)
%   4_ciclo.json                      trayectorias planeadas y medidas del ciclo (scripts/ciclo_4.py)
aqui = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'resultados');
a3 = 0.025; d6 = 0.090;

%% 1) Modelo DH simbolico
syms q1 q2 q3 q4 q5 q6 real
q = [q1 q2 q3 q4 q5 q6];
s = [-1 1 1 -1 1 -1];            % theta_i = s_i*q_i + offset_i (A1, A4, A6 giran al reves en el URDF de KUKA)
% DH Matrix - DH=[a_ij alpha_ij s_j theta_j]
DH = [ 0      0         0.400  -q1;
       0.025  -sym(pi)/2 0       q2;
       0.560  0         0       q3-sym(pi)/2;
       a3     -sym(pi)/2 0.515  -q4;
       0      sym(pi)/2  0       q5;
       0      -sym(pi)/2 d6      sym(pi)-q6];
frame = size(DH,1);
T = sym(zeros(4,4,frame));
Ti = sym(eye(4));
for i = 1:frame
    Ti = Ti*T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4));
    T(:,:,i) = Ti;                % T_0^i (MARCO GLOBAL)
end
p = T(1:3,4,frame);               % posicion de tool0 en base_link

%% 2) Jacobiano analitico
% Parte lineal: derivada de la posicion respecto a cada articulacion
Jv = jacobian(p, q);
% Parte angular: en DH modificado el eje z del marco i ES el eje de la articulacion i.
% omega = sum( s_i * z_i * qd_i )
Jw = sym(zeros(3,6));
for i = 1:6
    Jw(:,i) = s(i)*T(1:3,3,i);
end
J = [Jv; Jw];

% Comprobacion con el metodo geometrico: Jv_i = s_i * z_i x (p - o_i)
Jg = sym(zeros(3,6));
for i = 1:6
    Jg(:,i) = s(i)*cross(T(1:3,3,i), p - T(1:3,4,i));
end

disp('Columna 1 de Jv (derivada de p respecto a q1), simplificada:')
disp(simplify(Jv(:,1)))

J_fun  = matlabFunction(J,  'Vars', {q});
Jg_fun = matlabFunction(Jg, 'Vars', {q});

home = [0 -pi/2 pi/2 0 0 0];
disp('J analitico en HOME [0 -90 90 0 0 0] deg (filas vx vy vz wx wy wz):')
disp(round(J_fun(home), 4))

%% 3) Comparacion contra MoveIt2 (RobotState::getJacobian, punto de referencia = origen de tool0)
Qe  = readmatrix(fullfile(aqui,'q_eval.csv'));
JM  = readmatrix(fullfile(aqui,'J_moveit.csv'));
etq = readcell(fullfile(aqui,'q_eval_etiquetas.csv'));
n = size(Qe,1);
errV = zeros(n,1); errW = zeros(n,1); errG = zeros(n,1);
for k = 1:n
    Ja = J_fun(Qe(k,:));
    Jm = reshape(JM(k,:), 6, 6)';     % el CSV esta fila por fila
    errV(k) = max(abs(Ja(1:3,:) - Jm(1:3,:)), [], 'all');
    errW(k) = max(abs(Ja(4:6,:) - Jm(4:6,:)), [], 'all');
    errG(k) = max(abs(Ja(1:3,:) - Jg_fun(Qe(k,:))), [], 'all');
end
grupos = unique(etq(:,1), 'stable');
fprintf('\n=========== J analitico vs MoveIt2 (max |diferencia| por grupo) ===========\n');
fprintf('%-12s %6s %16s %16s %22s\n', 'grupo', 'n', 'lineal [m/rad]', 'angular [-]', 'derivada vs geometrico');
for g = 1:numel(grupos)
    idx = strcmp(etq(:,1), grupos{g});
    fprintf('%-12s %6d %16.2e %16.2e %22.2e\n', grupos{g}, nnz(idx), max(errV(idx)), max(errW(idx)), max(errG(idx)));
end
k = 1;
fprintf('\nEjemplo, configuracion aleatoria 1: q = [%s] deg\n', num2str(rad2deg(Qe(k,:)), '%8.2f'));
disp('J analitico:'); disp(round(J_fun(Qe(k,:)), 4));
disp('J MoveIt2:');   disp(round(reshape(JM(k,:),6,6)', 4));

%% 4) Verificacion de velocidades en 4B y 4D: v = Jv * qd debe dar el perfil impuesto
d = jsondecode(fileread(fullfile(aqui,'4_ciclo.json')));
nombres = fieldnames(d.rectas);
tlog = d.t; Qlog = d.q;
% limites de tiempo de los tramos rectos (mismo orden que d.rectas: 4B ida, 4B retorno, 4D ida, 4D retorno)
tramos = d.tramos;
lim_rectas = [];
for i = 1:numel(tramos)
    if contains(tramos{i}{1}, 'ida') || contains(tramos{i}{1}, 'retorno')
        lim_rectas(end+1,:) = [tramos{i}{2} tramos{i}{3}]; %#ok<SAGROW>
    end
end
figure(1); set(gcf, 'position', [100 100 1100 700])
fprintf('\n=========== Velocidad de tool0 en los tramos rectos ===========\n');
fprintf('%-12s %8s %12s %12s %14s %12s %12s %16s %14s\n', 'tramo', 'v_lim', 'v_perfil', 'max|J*qd|', 'err plan [m/s]', 'max|w| plan', 'v ejecutada', 'rms ejec [m/s]', 'cumple limite');
for r = 1:numel(nombres)
    R = d.rectas.(nombres{r});
    nombre = strrep(nombres{r}, 'x', '');           % jsondecode antepone 'x' a campos que empiezan con numero
    u = (R.pf - R.p0)/norm(R.pf - R.p0);
    if contains(nombre, 'ida'), v_lim = 0.2; else, v_lim = 0.1; end

    % plan: q y qd re-parametrizados que se enviaron al controlador
    vp = zeros(numel(R.t),3); wp = zeros(numel(R.t),1);
    for k = 1:numel(R.t)
        Jk = J_fun(R.q(k,:));
        vp(k,:) = (Jk(1:3,:)*R.qd(k,:)')';
        wp(k)   = norm(Jk(4:6,:)*R.qd(k,:)');
    end
    v_esp = R.v_perfil(:)*u(:)';                   % velocidad impuesta: perfil a lo largo de la recta
    err_plan = max(vecnorm(vp - v_esp, 2, 2));

    % ejecucion: joint_states (100 Hz); qd por diferencias finitas (mock_components publica velocidad 0)
    idx = find(tlog >= lim_rectas(r,1) & tlog <= lim_rectas(r,2));
    idx = idx([true; diff(tlog(idx)) > 1e-6]);     % descarta marcas de tiempo repetidas
    % joint_states llega con intervalos irregulares: se remuestrea a 10 ms antes de derivar
    te = (tlog(idx(1)):0.01:tlog(idx(end)))';
    Qe_ = interp1(tlog(idx), Qlog(idx,:), te);
    qde = gradient(Qe_')' ./ gradient(te);
    ve = zeros(numel(te),1);
    for k = 1:numel(te)
        Jk = J_fun(Qe_(k,:));
        ve(k) = norm(Jk(1:3,:)*qde(k,:)');
    end
    % alineacion temporal: MoveIt tarda unas decimas en arrancar la ejecucion (no es error de seguimiento)
    desfases = -1:0.002:3;
    rms = arrayfun(@(dt) sqrt(mean((interp1(te - te(1) - dt, ve, R.t, 'linear', 0) - abs(R.v_perfil)).^2)), desfases);
    [err_ejec, kmin] = min(rms);
    te = te - te(1) - desfases(kmin);

    fprintf('%-12s %8.3f %12.4f %12.4f %14.2e %12.2e %12.4f %16.2e %14s\n', nombre, v_lim, max(abs(R.v_perfil)), ...
            max(vecnorm(vp,2,2)), err_plan, max(wp), max(ve), err_ejec, string(max(ve) <= v_lim));

    subplot(2,2,r)
    plot(R.t, abs(R.v_perfil), 'k-', 'linewidth', 2); hold on
    kk = 1:8:numel(R.t);
    plot(R.t(kk), vecnorm(vp(kk,:),2,2), 'ro', 'markersize', 5)
    plot(te, ve, 'b.', 'markersize', 6)
    yline(v_lim, 'k:', 'v_{max}')
    xlim([-0.1 R.t(end)*1.1]); ylim([0 v_lim*1.15]); grid on
    xlabel('t [s]'); ylabel('|v tool0| [m/s]')
    title(sprintf('%s  (error plan %.1e, rms ejecutado %.1e m/s)', strrep(nombre,'_',' '), err_plan, err_ejec))
    if r == 1, legend('perfil impuesto', 'J_{analitico}\cdot qd (plan)', 'J_{analitico}\cdot qd (ejecutado)', 'location', 'northeast'); end
end
sgtitle('Parte 5: velocidad de tool0 = J(q) qd en los acercamientos finos')

figure(2); set(gcf, 'position', [150 150 900 380])
semilogy(errV, '.'); hold on; semilogy(errW, '.'); grid on
xline(200.5, 'k--', 'aleatorias | 4B y 4D')
xlabel('configuracion'); ylabel('max |J_{analitico} - J_{MoveIt}|')
legend('parte lineal', 'parte angular', 'location', 'best')
title('Parte 5: Jacobiano analitico vs RobotState::getJacobian() de MoveIt2')

%% Transform tipo Denavit-Hartenberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)
T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end
