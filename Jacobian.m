%% Jacobian test

clear
clc

%% Variables

syms q0 q1 q2 q3 q4 b_a1 b_a2 b_a3 b_m1 b_m2 b_m3 g0 g1 g2 h0 h1 h2 g

%% Function

R_from_n_to_b = [q1^2 - q2^2 - q3^2 + q4^2,   2 * (q1 * q2 + q3 * q4),      2 * (q1 * q3 - q2 * q4); ...
                 2 * (q1 * q2 - q3 * q4),     -q1^2 + q2^2 - q3^2 + q4^2,   2 * (q2 * q3 + q4 * q1); ...
                 2 * (q1 * q3 + q2 * q4),     2 * (q2 * q3 - q4 * q1),      -q1^2 - q2^2 + q3^2 + q4^2];
g = [g0; g1; g2];
h = [h0; h1; h2];
b_a = [b_a1; b_a2; b_a3];
b_m = [b_m1; b_m2; b_m3];

measurement_model = [R_from_n_to_b*g + b_a ; R_from_n_to_b*h + b_m];

F1 = measurement_model(1);
F2 = measurement_model(2);
F3 = measurement_model(3);
F4 = measurement_model(4);
F5 = measurement_model(5);
F6 = measurement_model(6);

%% Jacobian

x = [q1 q2 q3 q4 b_a1 b_a2 b_a3 b_m1 b_m2 b_m3];
Jacob = jacobian(measurement_model, x)

J0 = Jacob(6,1)
J1 = Jacob(6,2)
J2 = Jacob(6,3)
J3 = Jacob(6,4)
J4 = Jacob(6,5)
J5 = Jacob(6,6)
J6 = Jacob(6,7)
J7 = Jacob(6,8)
J8 = Jacob(6,9)
J9 = Jacob(6,10)



