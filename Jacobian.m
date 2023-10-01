function J = Jacobian(q1, q2, q3, q4, q5, l1, l2, l3, l4, l5)
    % Calcula el Jacobiano en coordenadas 3D (X, Y, Z) para un robot de 5 grados de libertad
    
    % Derivada respecto a q1
    J1 = [-sin(q1) * (l1 * cos(q1) + l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3) + l4 * cos(q1 + q2 + q3 + q4) + l5 * cos(q1 + q2 + q3 + q4 + q5));
          cos(q1) * (l1 * sin(q1) + l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3) + l4 * sin(q1 + q2 + q3 + q4) + l5 * sin(q1 + q2 + q3 + q4 + q5));
          0];
    
    % Derivada respecto a q2
    J2 = [-sin(q2) * (l2 * cos(q1 + q2) + l3 * cos(q1 + q2 + q3) + l4 * cos(q1 + q2 + q3 + q4) + l5 * cos(q1 + q2 + q3 + q4 + q5));
          cos(q2) * (l2 * sin(q1 + q2) + l3 * sin(q1 + q2 + q3) + l4 * sin(q1 + q2 + q3 + q4) + l5 * sin(q1 + q2 + q3 + q4 + q5));
          0];
    
    % Derivada respecto a q3
    J3 = [-sin(q3) * (l3 * cos(q1 + q2 + q3) + l4 * cos(q1 + q2 + q3 + q4) + l5 * cos(q1 + q2 + q3 + q4 + q5));
          cos(q3) * (l3 * sin(q1 + q2 + q3) + l4 * sin(q1 + q2 + q3 + q4) + l5 * sin(q1 + q2 + q3 + q4 + q5));
          0];
    
    % Derivada respecto a q4
    J4 = [-sin(q4) * (l4 * cos(q1 + q2 + q3 + q4) + l5 * cos(q1 + q2 + q3 + q4 + q5));
          cos(q4) * (l4 * sin(q1 + q2 + q3 + q4) + l5 * sin(q1 + q2 + q3 + q4 + q5));
          0];
    
    % Derivada respecto a q5
    J5 = [-sin(q5) * l5 * cos(q1 + q2 + q3 + q4 + q5);
          cos(q5) * l5 * sin(q1 + q2 + q3 + q4 + q5);
          0];
    
    % Matriz Jacobiana
    J = [J1, J2, J3, J4, J5];
end
