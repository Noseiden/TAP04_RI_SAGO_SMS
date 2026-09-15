function valido = solucionValida(q, qLim)
if ~isreal(q) || any(~isfinite(q))
    valido = false;
    return;
end

tol = 1e-9;
valido = all(q >= qLim(:,1)-tol & q <= qLim(:,2)+tol);
end