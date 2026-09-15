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