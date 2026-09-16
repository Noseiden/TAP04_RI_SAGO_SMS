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