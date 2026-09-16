function angulo = wrapToPiLocal(angulo)
angulo = mod(angulo + pi, 2*pi) - pi;
end