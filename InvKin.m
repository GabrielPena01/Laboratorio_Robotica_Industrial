clear;
clc;
format short;

% With this code we're going to work with inverse kinematics for the robot
% mitsubishi MV-R1

% COMMENT: The following are the different ranges of values that each 
% angle should take in order

% q1: [0° / 360°]
% q2: [-20° / 100°]
% q3: [0° / -110°]
% q4: [-90° / 90°]
% q5: [0° / 360°]

% First we define our desired position
Desired = [400; 150; 175];
% Then we define our starting values
q = [200; 50; 270; 83; 50];

% Define the constant of our work, getting epsilon, alpha
% and the max interations of our program
epsilon = 1e-3;
max_iter = 1000;
alpha = 0.3;

% Links longitude
L1 = 300; L2 = 250; L3 = 160; L4 = 147;


% Starting the operation
for i = 1:max_iter
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    
    % Transform matrix for every link
    LH1 = DHTransform(q1, L1, 0, 90);
    LH2 = DHTransform(q2, 0, L2, 0);
    LH3 = DHTransform(q3, 0, L3, 0);
    LH4 = DHTransform(q4, 0, 0, 90);
    LH5 = DHTransform(q5, L4, 0, 0);

    % We calculate the total Matrix
    T = LH1*LH2*LH3*LH4*LH5;
    
    % Get the final position
    f = [T(1,4); T(2,4); T(3,4)];
    
    % We calculate the error
    e = Desired - f;

    % Calculate the 3D Jacobian (Jacobian Matrix)
    J = Jacobian(q1, q2, q3, q4, q5, L1, L2, L3, L4, 0);

    % Usar la pseudo-inversa de la Jacobiana
    % Use the pseudo-inverse of the Jacobian
    q = q + alpha * pinv(J) * e;
    
    % Break condition
    if norm(e) < epsilon
        break;
    end
end


% We show the total transform
disp(T);

fprintf('and the position of the arm is:\n %d, %d, %d', T(1,4), T(2,4), T(3,4));
Axes = [T(1,4);T(2,4);T(3,4)];
x = Axes(1);
y = Axes(2);
z = Axes(3);

% Show the final configurations
fprintf('Ángulos:\n');
disp(q);

% Try (f(q))
fprintf('Comprobación (f(q)):\n');
disp(f);

O = [0,0,0];
A = [LH1(1,4), LH1(2,4), LH1(3,4)];
BM = LH1*LH2;
B = [BM(1,4), BM(2,4), BM(3,4)];
CM = BM*LH3;
C = [CM(1,4), CM(2,4), CM(3,4)];
D = [T(1,4);T(2,4);T(3,4)];

hold on
plot3(x, y, z, 'ro', 'MarkerSize', 10);
plot3([0;A(1) ]  ,   [0; A(2)]  ,   [0;A(3) ],'r', 'LineWidth', 2);
plot3([A(1);B(1) ]  ,   [A(2); B(2)]  ,   [A(3);B(3) ],'b', 'LineWidth', 2);
plot3([B(1);C(1) ]  ,   [B(2); C(2)]  ,   [B(3);C(3) ],'m', 'LineWidth', 2);
plot3([C(1);D(1) ]  ,   [C(2); D(2)]  ,   [C(3);D(3) ],'k', 'LineWidth', 2);

grid on;
title ('Robot Movement');

