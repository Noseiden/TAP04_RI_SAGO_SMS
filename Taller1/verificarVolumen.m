function verificarVolumen(q06)
dentro = q06(:,1) >= 800 & q06(:,1) <= 1300 & ...
         q06(:,2) >= -500 & q06(:,2) <= 500 & ...
         q06(:,3) >= 0 & q06(:,3) <= 1300;

if ~all(dentro)
    error('La trayectoria tiene puntos fuera del volumen indicado.');
end
end