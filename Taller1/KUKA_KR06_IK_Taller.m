clc;
close all;

%% PARAMETROS DH

% Recordatorio: estos valores son aproximados y deben actualizarse
% cuando se obtenga la matriz DH correcta.
% Las distancias se expresan en mm y los ángulos en radianes.


% Longitudes de los eslabones medidas sobre los ejes X.
a01 = 0;
a12 = 260;
a23 = 680;
a34 = 35;
a45 = 0;
a56 = 0;


% Desplazamientos entre sistemas coordenados.
S1 = 675;
S2 = 0;
S3 = 0;
S4 = 670;
S5 = 0;
S6 = 115;


% Ángulos de torsión entre los ejes de las articulaciones.
% Se convierten de grados a radianes para realizar los cálculos.

alpha01 = deg2rad(0);
alpha12 = deg2rad(-90);
alpha23 = deg2rad(0);
alpha34 = deg2rad(90);
alpha45 = deg2rad(90);
alpha56 = deg2rad(90);


% Desfase angular de cada articulación según la convención DH utilizada.
% El ángulo utilizado en cada transformación será:
% theta_DH = q + offset
offset1 = deg2rad(0);
offset2 = deg2rad(-90);
offset3 = deg2rad(180);
offset4 = deg2rad(180);
offset5 = deg2rad(180);
offset6 = deg2rad(180);


% Matriz de parámetros DH constantes.
% Columnas:
% 1. Longitud a_ij
% 2. Ángulo de torsión alpha_ij
% 3. Desplazamiento S_i
% 4. Desfase del ángulo theta_i
DH_const = [
    a01, alpha01, S1, offset1;
    a12, alpha12, S2, offset2;
    a23, alpha23, S3, offset3;
    a34, alpha34, S4, offset4;
    a45, alpha45, S5, offset5;
    a56, alpha56, S6, offset6
];


% Límites articulares obtenidos del datasheet.
% límite mínimo y límite máximo
qLim = deg2rad([
    -170,  170;    % Límites de A1
    -190,   45;    % Límites de A2
    -120,  156;    % Límites de A3
    -185,  185;    % Límites de A4
    -120,  120;    % Límites de A5
    -350,  350     % Límites de A6
]);


% Se agrupan los parámetros DH y los límites dentro de una estructura.
% Esto permite enviar toda la información del robot a las funciones
% utilizando una sola variable.
robot.DH = DH_const;
robot.qLim = qLim;


%% TRAYECTORIA CARTESIANA

n = 40;  % Cantidad de puntos de la trayectoria.

[q06, S6_dir, a67_dir] = crearTrayectoria(n, robot);	% Se generan las posiciones y orientaciones deseadas.

% q06: Matriz n x 3 con las posiciones [X Y Z] del efector.
% S6_dir: Matriz n x 3 con la dirección del eje Z del efector.
% a67_dir:Matriz n x 3 con la dirección del eje X del efector.


verificarVolumen(q06);	% Comprueba que todos los puntos estén dentro del volumen de trabajo


%% CINEMATICA INVERSA

% Inicializa la matriz de soluciones con valores NaN.
%
% Dimensiones:
% n filas: puntos de la trayectoria.
% 6 columnas: articulaciones del robot.
% 8 capas: posibles configuraciones.
%
% Cada capa SlnSet(:,:,k) contiene una configuración completa.
SlnSet = nan(n, 6, 8);


% Recorre todos los puntos de la trayectoria.
for i = 1:n

    % Calcula hasta ocho soluciones articulares para el punto i.
    % .' es para convertir cada fila en un vector columna.
    % cada fila representa una articulación y cada columna una solución.
    Q = cinematicaInversa( ...
        q06(i,:).', ...
        S6_dir(i,:).', ...
        a67_dir(i,:).', ...
        robot);

    % Guarda las ocho soluciones del punto i dentro de SlnSet.
    for k = 1:8
    
        % Se transpone para guardarla como una fila en SlnSet.
        
        SlnSet(i,:,k) = Q(:,k).'; % Q(:,k) contiene los seis ángulos de la configuración k.

    end
end


%% FILTRADO POR LIMITES

% Se crea una copia de las soluciones originales.
% SlnSet conserva todas las soluciones calculadas.
% SlnSetLim contendrá las soluciones después de revisar los límites.
SlnSetLim = SlnSet;


% Recorre cada una de las ocho configuraciones.
for k = 1:8

    % Recorre todos los puntos de la trayectoria.
    for i = 1:n

        % Extrae los seis ángulos del punto i y la configuración k.
        % El resultado se convierte en un vector columna de 6 x 1.
        q = SlnSet(i,:,k).';


        % Comprueba que los seis ángulos sean reales, finitos y estén dentro de los límites articulares.
        if ~solucionValida(q, robot.qLim)

            % Si la solución no es válida, se reemplazan sus ángulos por NaN.
            SlnSetLim(i,:,k) = nan(1,6);

        end
    end
end


%% FILTRADO POR CAPAS

% Vector lógico que indica cuáles de las ocho capas son válidas.
% Inicialmente se considera que ninguna capa es válida.
capasValidas = false(1,8);


% Revisa cada configuración de manera independiente.
for k = 1:8

    % Extrae todos los puntos y articulaciones de la configuración k.
    % La variable capa tiene dimensiones n x 6.
    capa = SlnSetLim(:,:,k);


    % Una capa es válida si:
    % 1. Todos sus valores son reales.
    % 2. Todos sus valores son finitos.
    % 3. No contiene ningún NaN.
    %
    % capa(:) coloca todos los elementos de la capa en un solo vector.
    capasValidas(k) = isreal(capa) && all(isfinite(capa(:)));

end


% Conserva solamente las capas válidas.
SlnSetVer = SlnSetLim(:,:,capasValidas);


% Obtiene los números originales de las capas válidas.
% Por ejemplo, indicesValidos = [1 2 5].
indicesValidos = find(capasValidas);


% Muestra las dimensiones de SlnSet.
fprintf('Tamaño de SlnSet: %d x %d x %d\n', size(SlnSet));


% Muestra las ocho soluciones correspondientes al primer punto.
% squeeze elimina la dimensión de tamaño 1 y deja una matriz 6 x 8.
fprintf('Soluciones del primer punto en radianes:\n');
disp(squeeze(SlnSet(1,:,:)));


% Muestra cuáles capas son válidas para todos los puntos.
fprintf('Capas válidas para toda la trayectoria: ');
fprintf('%d ', indicesValidos);
fprintf('\n');


% Detiene el programa si ninguna configuración puede completar
% toda la trayectoria respetando los límites.
if isempty(indicesValidos)

    error('No existe una configuración válida para toda la trayectoria.');

end


%% SELECCION DE CONFIGURACION

% Crea un vector para guardar el movimiento articular total
% requerido por cada configuración válida.
costos = zeros(1, size(SlnSetVer,3));


% Calcula el costo de movimiento de cada configuración.
for k = 1:size(SlnSetVer,3)

    % Extrae la configuración k.
    %
    % unwrap corrige saltos artificiales de 2*pi.
    % ejemplo, evita interpretar el cambio de 179° a -179° como un movimiento de 358°.
    Qk = unwrap(SlnSetVer(:,:,k), [], 1);


    % diff calcula el cambio de cada articulación entre
    % dos puntos consecutivos.
    %
    % abs obtiene la magnitud de cada cambio.
    %
    % sum(sum(...)) suma todos los movimientos articulares.
    costos(k) = sum(sum(abs(diff(Qk,1,1))));

end


% Encuentra la configuración con el menor movimiento total.
%
% El símbolo ~ indica que no se necesita guardar el valor mínimo;
% solamente se necesita conocer su posición.
[~, indiceLocal] = min(costos);


% Convierte el índice dentro de SlnSetVer en el número de la capa
% correspondiente dentro de SlnSet.
configuracionElegida = indicesValidos(indiceLocal);


% Extrae la trayectoria articular definitiva.
% Qtray tiene dimensiones n x 6.
Qtray = SlnSetVer(:,:,indiceLocal);


% Muestra la capa seleccionada.
fprintf('Configuración seleccionada: capa %d\n', ...
    configuracionElegida);


%% VERIFICACION CON CINEMATICA DIRECTA

% Vectores para guardar el error de cada punto.
errorPos = zeros(n,1);
errorOri = zeros(n,1);


% Recorre todos los puntos de la configuración seleccionada.
for i = 1:n

    % Calcula la posición y orientación obtenidas mediante
    % cinemática directa usando los ángulos de Qtray.
    [Tcalc, ~] = cinematicaDirecta(Qtray(i,:).', robot);


    % Reconstruye la matriz de orientación deseada a partir
    % de los dos vectores definidos para el efector.
    Rdes = construirOrientacion( ...
        S6_dir(i,:).', ...
        a67_dir(i,:).');


    % Calcula la rotación necesaria para pasar desde la
    % orientación calculada hasta la orientación deseada.
    Rerr = Tcalc(1:3,1:3).' * Rdes;


    % Calcula la distancia entre la posición obtenida mediante
    % cinemática directa y la posición deseada.
    errorPos(i) = norm( ...
        Tcalc(1:3,4) - q06(i,:).');


    % Obtiene el coseno del ángulo del error de orientación
    % a partir de la traza de la matriz de rotación relativa.
    valor = (trace(Rerr) - 1) / 2;


    % Limita el valor al intervalo [-1,1] para evitar errores
    % numéricos al utilizar acos.
    errorOri(i) = acos( ...
        max(-1, min(1, valor)));

end


% Muestra los errores máximos encontrados en toda la trayectoria.
fprintf('Error máximo de posición: %.6e mm\n', max(errorPos));
fprintf('Error máximo de orientación: %.6e rad\n', max(errorOri));

%% ANIMACION

% Anima el robot utilizando la configuración seleccionada.
%
% Qtray contiene los ángulos articulares.
% q06 contiene la trayectoria cartesiana que debe dibujarse.
animarRobot(Qtray, q06, robot);