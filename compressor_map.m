function [mdot, pi_c, eta, T_out, P, Tq, SM] = compressor_map( ...
    omega, ...
    p_in, ...
    T_in, ...
    p_out, ...
    gamma, ...
    cp, ...
    p_ref, ...
    T_ref, ...
    omega_ref, ...
    c_a_s, ...
    c_b_s, ...
    c_a_c, ...
    c_b_c, ...
    c_a_o, ...
    c_b_o, ...
    c_d1, ...
    c_d2, ...
    c_kpi, ...
    c_keta, ...
    c_e0, ...
    c_e1)
%#codegen
%
% compressor_map  Quasi-steady compressor performance map.
%
% Inputs
%   omega      - shaft angular velocity (rad/s)
%   p_in       - compressor inlet total pressure (Pa)
%   T_in       - compressor inlet total temperature (K)
%   p_out      - compressor outlet total pressure (Pa)
%   gamma      - specific heat ratio (-)
%   cp         - specific heat at constant pressure (J/(kg*K))
%   p_ref      - reference pressure for corrected variables (Pa)
%   T_ref      - reference temperature for corrected variables (K)
%   omega_ref  - reference shaft speed (rad/s)
%   c_a_s      - surge line intercept (corrected flow)
%   c_b_s      - surge line slope vs corrected speed
%   c_a_c      - choke line intercept (corrected flow)
%   c_b_c      - choke line slope vs corrected speed
%   c_a_o      - optimal flow line intercept
%   c_b_o      - optimal flow line slope
%   c_d1       - pressure ratio capability magnitude
%   c_d2       - pressure ratio capability saturation factor
%   c_kpi      - PR map curvature factor
%   c_keta     - efficiency map curvature factor
%   c_e0       - peak efficiency at design speed (-)
%   c_e1       - efficiency drop away from design speed (-)
%
% Outputs
%   mdot  - mass flow rate (kg/s)
%   pi_c  - compression pressure ratio p_out/p_in (-)
%   eta   - isentropic efficiency (-)
%   T_out - real outlet total temperature (K)
%   P     - shaft power consumed (W, positive)
%   Tq    - shaft torque consumed (Nm, positive)
%   SM    - surge margin based on corrected flow (-)

epsw = 1e-6;

% ===== Corrected speed (dimensionless) =====
N = (omega / (omega_ref + epsw)) * sqrt(T_ref / (T_in + epsw));

% ===== Surge, choke, and optimal corrected flow limits =====
W_surge = c_a_s + c_b_s * N;
W_choke = c_a_c + c_b_c * N;
W_span  = max((W_choke - W_surge) / 2, 5e-3);
W_opt   = c_a_o + c_b_o * N;

% ===== Peak pressure ratio available at this speed =====
dPi_max = (c_d1 * N^2) / (1 + c_d2 * N^2);

% ===== Invert parabolic PR map to find corrected flow W =====
% PR(W) = 1 + dPi_max * (1 - c_kpi * ((W-W_opt)/W_span)^2)
% Given commanded PR = p_out/p_in, solve for W (higher-flow branch).
PR_cmd = max(p_out / (p_in + epsw), 1.01);

if dPi_max < 1e-3
    xi2 = 1;
else
    xi2 = (1 - (PR_cmd - 1) / dPi_max) / max(c_kpi, 1e-3);
end
xi2 = min(max(xi2, 0), 1.5);
W = W_opt + W_span * sqrt(xi2);

% ===== Clamp within surge/choke limits =====
W = min(max(W, W_surge + 2e-3), W_choke - 2e-3);

% ===== Pressure ratio and efficiency from map =====
pi_c = 1 + dPi_max * (1 - c_kpi * ((W - W_opt) / W_span)^2);
pi_c = max(pi_c, 1.01);

eta_max = max(min(c_e0 - c_e1 * (N - 1)^2, 0.86), 0.55);
eta     = eta_max * (1 - c_keta * ((W - W_opt) / W_span)^2);
eta     = max(min(eta, 0.86), 0.4);

% ===== Convert corrected flow to actual mass flow =====
mdot = W * (p_in / p_ref) / sqrt((T_in + epsw) / T_ref);
mdot = max(mdot, 1e-4);

% ===== Outlet temperature (real, accounting for efficiency) =====
T_out = T_in * (1 + (pi_c^((gamma-1)/gamma) - 1) / max(eta, 0.05));

% ===== Shaft power and torque (positive convention: compressor loads shaft) =====
P  = mdot * cp * (T_out - T_in);
Tq = P / (omega + 5);

% ===== Surge margin (corrected flow distance from surge line) =====
SM = (W - W_surge) / max(W_surge, 1e-3);

end
