function R_b2w = body2worldRotationMatrix(g_world,g_body)

% Normalizzazione dei vettori
v1 = g_body / norm(g_body);   % direzione gravity in camera
v2 = g_world / norm(g_world);  % direzione gravity nel world (Z down)

% Calcolo della rotazione tra i due vettori
v = cross(v1, v2);        % asse di rotazione
s = norm(v);              % seno dell'angolo
c = dot(v1, v2);          % coseno dell'angolo

% Matrice antisimmetrica
Vx = [  0   -v(3)  v(2);
       v(3)   0   -v(1);
      -v(2) v(1)    0 ];

% Formula di Rodrigues
if s < 1e-8
    R_b2w = eye(3);  % i vettori sono già allineati
else
    R_b2w = eye(3) + Vx + Vx^2 * ((1 - c)/(s^2));
end

end