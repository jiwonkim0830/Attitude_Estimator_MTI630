%% Jacobian test

clear
clc

%% Variables

syms q0 q1 q2 q3 b_a1 b_a2 b_a3 b_m1 b_m2 b_m3 g1 g2 g3 h1 h2 h3

%% Function

R = [2 * (q0 * q0 + q1 * q1) - 1, 2 * (q1 * q2 - q0 * q3),     2 * (q1 * q3 + q0 * q2); ...
     2 * (q1 * q2 + q0 * q3),     2 * (q0 * q0 + q2 * q2) - 1, 2 * (q2 * q3 - q0 * q1); ...
     2 * (q1 * q3 - q0 * q2),     2 * (q2 * q3 + q0 * q1),     2 * (q0 * q0 + q3 * q3) - 1];
R_t = R.'

g = [g1; g2; g3];
h = [h1; h2; h3];
b_a = [b_a1; b_a2; b_a3];
b_m = [b_m1; b_m2; b_m3];

measurement_model = [(R.')*g + b_a ; (R.')*h + b_m];

F1 = measurement_model(1)
F2 = measurement_model(2)
F3 = measurement_model(3)
F4 = measurement_model(4)
F5 = measurement_model(5)
F6 = measurement_model(6)

%% Jacobian

x = [q0 q1 q2 q3 b_a1 b_a2 b_a3 b_m1 b_m2 b_m3];
Jacob = jacobian([F1, F2, F3, F3, F4, F5, F6], x)



