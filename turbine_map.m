function [mdot, pi_t, eta, T_out, P, Tq] = turbine_map( ...
    omega, ...
    p_in, ...
    T_in, ...
    p_out, ...
    gamma, ...
    cp, ...
    p_ref, ...
    T_ref, ...
    omega_ref, ...
    t_eta_max, ...
    t_kN, ...
    t_Kflow ...
    )
%#codegen
%
% compressor_map  Quasi-steady turbine performance map.
%
% Inputs
%   omega      - shaft angular velocity (rad/s)
%   p_in       - turbine inlet total pressure (Pa)
%   T_in       - turbine inlet total temperature (K)
%   p_out      - turbine outlet total pressure (Pa)
%   gamma      - specific heat ratio (-)
%   cp         - specific heat at constant pressure (J/(kg*K))
%   p_ref      - reference pressure for corrected variables (Pa)
%   T_ref      - reference temperature for corrected variables (K)
%   omega_ref  - reference shaft speed (rad/s)
%   t_eta_max  - peak isentropic efficiency (-)
%   t_kN       - efficiency curvature vs corrected speed (-)
%   t_Kflow    - flow coefficient (kg*sqrt(K)/(s*Pa))
%
% Outputs
%   mdot  - mass flow rate (kg/s)
%   pi_t  - expansion pressure ratio p_in/p_out (-)
%   eta   - isentropic efficiency (-)
%   T_out - real outlet total temperature (K)
%   P     - shaft power produced (W, positive)
%   Tq    - shaft torque produced (Nm, positive)

epsw = 1e-6;

% ===== Pressure ratio (expansion: p_in > p_out) =====
pi_t = max(p_in / (p_out + epsw), 1.01);

% ===== Corrected speed (dimensionless) =====
N = (omega / (omega_ref + epsw)) * sqrt(T_ref / (T_in + epsw));

% ===== Isentropic efficiency (parabolic vs corrected speed) =====
eta = t_eta_max * (1 - t_kN * (N - 1)^2);
eta = max(min(eta, t_eta_max), 0.4);

% ===== Compressible flow function =====
flow_func = sqrt(max(1 - (1/pi_t)^((gamma-1)/gamma), 0));

% ===== Mass flow =====
mdot = t_Kflow * p_in * sqrt(1 / (T_in + epsw)) * flow_func;
mdot = max(mdot, 1e-4);

% ===== Isentropic outlet temperature =====
% T_is_out is the actual isentropic outlet temperature (K), not the drop.
T_is_out = T_in * (1/pi_t)^((gamma-1)/gamma);

% ===== Real outlet temperature =====
T_out = T_in - eta * (T_in - T_is_out);

% ===== Shaft power and torque (positive convention: turbine drives shaft) =====
P  = mdot * cp * (T_in - T_out);
Tq = P / (omega + 5);

end
