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