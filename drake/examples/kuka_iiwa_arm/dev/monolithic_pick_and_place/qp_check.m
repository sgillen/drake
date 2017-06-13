% Simple check for unconstrained qp

% q = [x; y; z];

J_meas_W = zeros(3);
meas_W = [1; 2; 3];
J_mod_W = eye(3);
mod_W = [0; 0; 0];

e = meas_W - mod_W;
Je = J_meas_W - J_mod_W;

Q = 2 * Je' * Je;
f = 2 * e' * Je;

x = Q \ -f'
