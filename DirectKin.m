clear;
clc;
format short;
% With this program, we want to get the direct kinematics of a Mitsubishi robot

% First, we're going to define the constants
L1 = 300; L2 = 250; L3 = 160; L4 = 147;

% Before starting to calculate the matrices, we need to ask for input data
Angle1 = input('Please, tell us the first angle: ');
Angle2 = input('We need the second angle too: ');
Angle3 = input('The third angle is?: ');
Angle4 = input('And the fourth angle is: ');
Angle5 = input('The last angle is: ');

% Then we can start to calculate the Denavit-Hartenberg Matrices
% For Art 1
ML1 = DHTransform(Angle1, L1, 0, 90);
fprintf('The first matrix is: \n');
disp(ML1);

% For Art 2
ML2 = DHTransform(Angle2, 0, L2, 0);
fprintf('The second matrix is: \n');
disp(ML2);

%For Art 3
ML3 = DHTransform(Angle3, 0, L3, 0);
fprintf('The third matrix is: \n');
disp(ML3);

% For Art 4
ML4 = DHTransform(Angle4, 0, 0, 90);
fprintf('The fourth matrix is: \n');
disp(ML4);

%For Art 5
ML5 = DHTransform(Angle5, L4, 0, 0);
fprintf('The fifth matrix is: \n');
disp(ML5);

% Calculate the total transformation from base to end-effector
Transform = ML1 * ML2 * ML3 * ML4 * ML5;
fprintf('The total matrix is: \n');
disp(Transform);
fprintf('and the position of the arm is:\n %d, %d, %d', Transform(1,4), Transform(2,4), Transform(3,4));
Axes = [Transform(1,4);Transform(2,4);Transform(3,4)];
x = Axes(1);
y = Axes(2);
z = Axes(3);

O = [0,0,0];
A = [ML1(1,4), ML1(2,4), ML1(3,4)];
BM = ML1*ML2;
B = [BM(1,4), BM(2,4), BM(3,4)];
CM = BM*ML3;
C = [CM(1,4), CM(2,4), CM(3,4)];
D = [Transform(1,4);Transform(2,4);Transform(3,4)];

hold on
plot3(x, y, z, 'ro', 'MarkerSize', 10);
plot3([0;A(1) ]  ,   [0; A(2)]  ,   [0;A(3) ],'r', 'LineWidth', 2);
plot3([A(1);B(1) ]  ,   [A(2); B(2)]  ,   [A(3);B(3) ],'b', 'LineWidth', 2);
plot3([B(1);C(1) ]  ,   [B(2); C(2)]  ,   [B(3);C(3) ],'m', 'LineWidth', 2);
plot3([C(1);D(1) ]  ,   [C(2); D(2)]  ,   [C(3);D(3) ],'k', 'LineWidth', 2);

grid on;
title ('Robot Movement');
