function [q06, S6_dir, a67_dir] = crearTrayectoria(n, robot)

% Parámetro utilizado para generar los puntos de la trayectoria.
t = linspace(0,2*pi,n).';

% Matriz con los ángulos articulares de referencia.
qRef = zeros(n,6);

% Trayectoria articular utilizada para generar poses alcanzables.
qRef(:,1) = deg2rad(15*sin(t));
qRef(:,2) = deg2rad(30 + 4*sin(t));
qRef(:,3) = deg2rad(20 + 8*cos(t));
qRef(:,4) = deg2rad(10*sin(t));
qRef(:,5) = deg2rad(30 + 5*cos(t));
qRef(:,6) = deg2rad(20*sin(2*t));

% Matrices para guardar la posición y orientación.
q06 = zeros(n,3);
S6_dir = zeros(n,3);
a67_dir = zeros(n,3);

% Convierte la trayectoria articular en una trayectoria cartesiana.
for i = 1:n

    [T, ~] = cinematicaDirecta(qRef(i,:).',robot);

    % Posición cartesiana [X Y Z].
    q06(i,:) = T(1:3,4).';

    % Dirección del eje X del efector.
    a67_dir(i,:) = T(1:3,1).';

    % Dirección del eje Z del efector.
    S6_dir(i,:) = T(1:3,3).';

end

end