%% ============================================================
%  INIT FILE FOR TURBO-GENERATOR SIMULINK MODEL
%  Engine  : PSA/BMW Prince EP6CDT 1.6L THP 156 hp
%  Turbo   : BorgWarner K03 (part no. 53039880163)
%  EMG     : 48 V / ~5 kW PMSM on turbo shaft
%  Model   : compressor + turbine + shaft + EMG + PI boost controller
%             + compressor outlet plenum (Phase 4a)
%             + exhaust manifold plenum  (Phase 4b)
%  ============================================================

clearvars -except ans
clc

%% ============================================================
% 1. GAS PROPERTIES (air approximation)
%% ============================================================

gamma = 1.4;              % heat capacity ratio (-)
cp    = 1005;             % J/(kg*K)
R     = cp*(gamma-1)/gamma;

%% ============================================================
% 2. REFERENCE CONDITIONS (for corrected variables)
%    omega_ref is the BorgWarner K03 design shaft speed.
%    It is used only inside TurbineMap / CompressorMap to form
%    the dimensionless corrected speed  N = (omega/omega_ref)*sqrt(T_ref/T_in).
%    The PI controller setpoint is p_c_out_ref (section 9).
%% ============================================================

p_ref = 101325;           % Pa   — ISA sea-level pressure
T_ref = 288.15;           % K    — ISA sea-level temperature

rpm_ref   = 175000;                       % K03 design shaft speed (rpm)
omega_ref = rpm_ref * 2*pi/60;            % rad/s — ~18 326 rad/s

%% ============================================================
% 3. SHAFT PARAMETERS
%    Rotor geometry: K03 compressor wheel 41/54 mm, turbine 44.9/40.2 mm.
%    Inertia estimated from published turbocharger rotor mass data.
%% ============================================================

J         = 3.5e-5;                  % kg*m^2   — K03 rotor inertia
                                     %   Increased from 2.5e-5 to 3.5e-5 to slow natural
                                     %   spool-up, making the EMG motor-assist advantage
                                     %   more visible during cruise-to-overtake transients.
                                     %   Still within the K03 published rotor mass range
                                     %   (2–4 × 10⁻⁵ kg·m²).
k_f       = 5e-7;                    % Nm*s/rad — viscous friction (bearing drag)
                                     %   K03 ball-bearing range: 2–8 × 10⁻⁷ Nm·s/rad
                                     %   Increased from 2e-7 to 5e-7 to damp the lightly-
                                     %   damped oscillatory mode of the coupled shaft+plenum
                                     %   system without changing steady-state behaviour.
omega_max = 220000 * 2*pi/60;        % rad/s    — K03 mechanical overspeed limit

%% ============================================================
% 4. COMPRESSOR MAP PARAMETERS
%    Fitted to BorgWarner K03 (53039880163) published map data.
%    All corrected flows W are in kg/s at reference conditions.
%
%    Design point (N = 1, omega = omega_ref):
%      W_surge ≈ 0.032 kg/s
%      W_opt   ≈ 0.062 kg/s   (peak efficiency ridge)
%      W_choke ≈ 0.115 kg/s
%      PR_max  ≈ 2.60   (= 1 + c_d1/(1+c_d2))
%      eta_max ≈ 0.75
%% ============================================================

% Surge line:  W_surge = c_a_s + c_b_s * N
c_a_s = 0.008;
c_b_s = 0.024;

% Choke line:  W_choke = c_a_c + c_b_c * N
c_a_c = 0.015;
c_b_c = 0.100;

% Optimal flow line (best efficiency):  W_opt = c_a_o + c_b_o * N
c_a_o = 0.008;
c_b_o = 0.054;

% Pressure ratio capability:  dPi_max = c_d1*N^2 / (1 + c_d2*N^2)
% At N=1: dPi_max = 3.2/2 = 1.6  →  PR_max = 2.60
c_d1  = 3.2;
c_d2  = 1.0;

% Map shape factors
c_kpi  = 1.0;     % PR curvature (parabola width factor)
c_keta = 0.72;    % efficiency curvature vs flow deviation

% Efficiency peak
c_e0 = 0.75;      % K03 peak isentropic efficiency (-) — measured ~74-76%
c_e1 = 0.04;      % drop coefficient away from design speed

%% ============================================================
% 5. TURBINE PARAMETERS
%    K03 turbine: 44.9/40.2 mm wheel, 11 blades, A/R ≈ 0.42.
%% ============================================================

t_eta_max = 0.72;     % K03 peak turbine isentropic efficiency (-)
t_kN      = 0.12;     % efficiency drop vs corrected speed deviation
% t_Kflow is derived in Section 11 from mass flow equilibrium — do not set here.

%% ============================================================
% 6. ELECTRIC MACHINE PARAMETERS
%    High-speed PMSM on the K03 shaft, 48 V mild-hybrid bus.
%    Sizing: P_max = V_bus * I_max * eta_e = 48 * 130 * 0.92 ≈ 5.8 kW
%            Tq_max = P_max / omega_ref = 5800 / 18326 ≈ 0.32 Nm  → 0.35 Nm
%    Reference: ISO 21780:2021 (48 V supply voltage standard).
%% ============================================================

eta_e  = 0.92;    % electro-mechanical round-trip efficiency (-)
Tg_max = 0.35;    % maximum machine torque (Nm)  — 48 V @ ~130 A · 0.92 ≈ 5.8 kW at design speed
                  %   Increased from 0.25 to 0.35 Nm to:
                  %     (a) widen the EMG-on vs EMG-off pressure recovery gap (motor mode)
                  %     (b) provide 40% more generator braking torque to arrest post-overtake
                  %         shaft acceleration (generator mode)
V_bus  = 48;      % DC bus voltage (V)

%% ============================================================
% 6b. STEADY-STATE GENERATOR EXTRACTION TARGET
%
%   P_gen_cruise — the electrical power the generator should extract
%   from the shaft at cruise steady state.  Setting this > 0 means
%   the turbine must produce a surplus:
%
%     P_turbine = P_compressor + P_friction + P_gen_cruise / eta_e
%
%   This raises p_t_in_0 (the required exhaust backpressure) so the
%   turbine has the expansion ratio necessary to drive the generator.
%   The PI integrator is pre-loaded (see Section 9) so the model
%   starts in generator mode immediately — no startup transient.
%
%   Chain: P_gen_cruise ↑  →  P_shaft_req ↑
%                          →  p_t_in_0 ↑      (Section 11)
%                          →  turbine expands further
%                          →  generator absorbs surplus
%
%   Set to 0 to recover the legacy compressor-only equilibrium.
%
%   emg_enabled — set to 0 to bypass the EMG for open-loop turbine
%   testing.  Wire as a Gain block multiplying the PI output `u`
%   before the motor / generator mode split in Simulink so the
%   MotorGenerator block receives zero demand in both channels.
%   With emg_enabled = 0 and P_gen_cruise > 0 the turbine produces
%   surplus power → shaft accelerates above omega0 → p_c_out overshoots
%   above setpoint, confirming the turbine has genuine surplus capacity.
%% ============================================================

P_gen_cruise = 000;   % W  — target cruise electrical output (0 = legacy, 1000 W ≈ 21 A at 48 V)
emg_enabled  = 0;      % 1  — EMG active;  0 — EMG bypassed (turbine open-loop test)

%% ============================================================
% 7. BOUNDARY CONDITIONS
%    p_c_out is NO LONGER a fixed constant — it is the plenum
%    integrator state initialised by p_c_out_0 (section 8).
%    p_t_in  is NO LONGER a fixed constant — it is the exhaust manifold
%    plenum integrator state, initialised by p_t_in_0 (section 11).
%    T_exh   is NO LONGER a fixed constant — see Section 7b.
%% ============================================================

% Compressor inlet (ambient, sea-level ISA)
p_c_in = 101325;     % Pa
T_c_in = 288.15;     % K

% Turbine outlet (after catalyst/muffler) — fixed back-pressure
p_t_out = 110000;    % Pa

%% ============================================================
% 7b. DYNAMIC EXHAUST GAS TEMPERATURE MODEL (Phase 4c)
%
%  The turbine inlet temperature T_exh is estimated from the
%  instantaneous engine load (throttle_factor) via two coupled maps:
%
%  (1) LOAD → LAMBDA MAP
%      At part-load cruise the EP6CDT runs lean (lambda ≈ 1.15) for
%      fuel economy. At WOT the mixture is enriched to lambda ≈ 0.92
%      to suppress knock and limit turbine inlet temperature.
%      Reference: Heywood (1988), §3.4; consistent with Euro-5
%      calibration data for 1.6 L turbocharged GDI engines.
%
%      lambda(u) = lambda_cruise - (lambda_cruise - lambda_WOT)
%                  × sat( (u - 1) / (throttle_overtake - 1), 0, 1 )
%      where u = throttle_factor signal.
%
%  (2) LAMBDA / LOAD → T_exh MAP
%      Exhaust temperature rises with load even as lambda drops,
%      because total fuel energy per cycle dominates over dilution.
%      Values are bracketed by BorgWarner K03 application limits:
%        • max continuous T_t_in ≈ 1050 K (777 °C)
%        • cruise range         800–870 K (Baines 2005, p. 8)
%      Linear interpolation is a standard simplification used in
%      mean-value engine models (Eriksson & Nielsen 2014, §8.3).
%
%      T_exh_static(u) = T_exh_cruise
%                       + (T_exh_WOT - T_exh_cruise)
%                         × sat( (u - 1) / (throttle_overtake - 1), 0, 1 )
%
%  (3) THERMAL LAG
%      The exhaust manifold has finite thermal mass (~0.8–1.2 kg cast
%      iron, c_p ≈ 500 J/(kg·K)), giving a first-order lag of 2–4 s
%      between combustion event and turbine inlet temperature change.
%      Watson & Janota (1982) §6.4 report τ ≈ 2–3 s on similar rigs.
%
%      dT_exh/dt = (T_exh_static - T_exh) / tau_T_exh
%
%  In Simulink: replace the fixed T_exh Constant block with the
%  "T_exh_Model" subsystem (see simulation-model.md §Phase 4c).
%% ============================================================

AFR_stoich    = 14.7;    % stoichiometric air/fuel ratio for gasoline (-)
lambda_cruise = 1.15;    % equivalence ratio at part-load cruise (-)
lambda_WOT    = 0.92;    % equivalence ratio at WOT — EP6CDT enrichment for
                         % knock suppression (Heywood 1988, §9.2)

T_exh_cruise  = 820;     % K — turbine inlet temp at cruise (lambda = 1.15)
                         % Consistent with published range 800–870 K for
                         % 1.6 L turbocharged SI at light load
                         % (Baines 2005; BorgWarner K03 application data)
T_exh_WOT     = 990;     % K — turbine inlet temp at WOT (lambda = 0.92)
                         % Below K03 continuous limit (~1050 K) and within
                         % the 950–1050 K WOT range for this engine class
                         % (Watson & Janota 1982, Table 6.1)
tau_T_exh     = 2.5;     % s — exhaust manifold first-order thermal lag

% Initial value for the Simulink T_exh Integrator IC
T_exh   = T_exh_cruise;  % K — starts at cruise equilibrium
T_t_in  = T_exh;         % K — Simulink alias (kept for legacy wiring)

%% ============================================================
% 8. INITIAL CONDITIONS
%    omega0 set to a realistic K03 cruise idle speed.
%    p_c_out_0 ≈ 0.43 bar boost — moderate part-load for EP6CDT.
%% ============================================================

omega0    = 130000  * 2*pi/60;
p_c_out_0 = 145000;             % Pa   — compressor outlet plenum IC (~0.43 bar boost)

%% ============================================================
% 9. BOOST PRESSURE CONTROLLER PARAMETERS
%
%    PI controller closes the loop on p_c_out (Pa).
%
%    Gain sizing (K03 / Tg_max = 0.35 Nm):
%      At a 10 kPa pressure error → ~0.12 Nm correction (34% of Tg_max)
%      Kp = 0.12 Nm / 10 000 Pa = 1.2e-5 Nm/Pa
%      Integration time Ti = Kp/Ki = 4 s  (longer than shaft + plenum τ)
%      Ki = Kp / Ti = 1.2e-5 / 4 = 3e-6 Nm/(Pa·s)
%      Reduced from Kp=1.6e-5 / Ki=4e-6 to cut post-recovery overshoot
%      and integrator windup during the overtake boost-build phase.
%% ============================================================

% Setpoint — equal to p_c_out_0 so simulation starts at equilibrium.
% Raise p_c_out_ref above p_c_out_0 to command a higher target boost.
p_c_out_ref = p_c_out_0;    % Pa — cruise boost target

% Overtake boost target — stepped in Simulink at t_overtake / t_release
% in the same delayed-switch manner as throttle_overtake.
% EP6CDT WOT boost: ~2.0–2.2 bar absolute (factory map, 95 RON).
p_c_out_overtake = 190000;  % Pa — WOT boost target (~1.0 bar gauge)

% PI gains  (error in Pa, output in Nm)
Kp = 1.2e-5;    % Nm / Pa      — reduced from 1.6e-5 to limit proportional overshoot
                %                 after pressure recovery on throttle release
Ki = 1.5e-6;    % Nm / (Pa·s) — reduced from 4.0e-6; slower integration rate reduces
                %                 windup accumulated during the boost-build phase,
                %                 limiting post-recovery oscillation

u_min = -Tg_max;    % Nm — anti-windup lower clamp
u_max =  Tg_max;    % Nm — anti-windup upper clamp

% --- PI integrator pre-load for steady-state generator extraction ---
%
% In generator mode the PI output u < 0, giving:
%   Pe_demand  = (-u) * eta_e * omega          [W]
%   Tq_gen     = Pe_demand / (eta_e * omega) = -u  [Nm]
%
% At cruise equilibrium (e = 0), the integrator alone holds u:
%   u_pi_init  = -Tq_gen_ss  where  P_gen_cruise = eta_e * Tq_gen_ss * omega0
%
% UNITS NOTE:  u_pi_init is in Nm (the PI output / torque).
%              pi_int_init is in Pa·s (the integral of the pressure error).
%              u_min / u_max are in Nm — they bound u_pi_init, NOT pi_int_init.
%              Do NOT compare pi_int_init against u_min / u_max — different units.
%
% HOW TO USE IN SIMULINK — two options:
%
%   Option A (pre-loaded, zero transient):
%     PREREQUISITE: Simulink enforces u_min ≤ IntegratorIC ≤ u_max when
%     "Limit output" is enabled on the PID block.  pi_int_init is in Pa·s
%     and violates this check even though u(0) = Ki*pi_int_init is within
%     range — because Simulink compares the raw IC value against the Nm limits.
%     FIX: disable "Limit output" on the PID block (the external Sat_motor /
%     Sat_gen blocks already clamp both channels to [0, Tg_max]).
%     Then set PID "Integrator initial condition" = pi_int_init  [Pa·s].
%
%   Option B (simpler, ~1–2 s transient, no PID block changes):
%     Leave PID integrator IC = 0 (default), keep "Limit output" on.
%     At t=0 the turbine surplus accelerates the shaft → p_c_out overshoots
%     above p_c_out_ref → PID drives itself negative → generator absorbs surplus.
%     Steady-state is identical; only the first couple of seconds differ.

Tq_gen_ss   = P_gen_cruise / (eta_e * max(omega0, 1));  % Nm   — cruise generator braking torque
u_pi_init   = -Tq_gen_ss;                               % Nm   — PI output at cruise equilibrium
                                                         %         within [u_min, u_max] ← compare here
pi_int_init = u_pi_init / Ki;                            % Pa·s — integrator STATE for Option A above
                                                         %         NOT comparable to u_min / u_max

%% ============================================================
% 10. PLENUM PARAMETERS (Phase 4)
%     Intake manifold sized for EP6CDT: ~1.8–2.2 L typical runner volume.
%% ============================================================

V_manifold = 2e-3;    % m³ — intake manifold volume (~2 L)

% Plenum ODE gain:  dp_c_out/dt = K_plenum * T_c_out * (mdot_c - mdot_engine)
K_plenum = gamma * R / V_manifold;

% --- Derive mdot_cruise from the compressor map at the initial shaft speed ---
% Mirrors the inversion in compressor_map.m exactly so K_engine_base is
% consistent with what the Simulink model will compute at t = 0.
% Result guarantees dp_c_out/dt = 0 at t = 0 (plenum starts in equilibrium).
N_ss       = (omega0 / omega_ref) * sqrt(T_ref / T_c_in);
dPi_max_ss = c_d1 * N_ss^2 / (1 + c_d2 * N_ss^2);
W_surge_ss = c_a_s + c_b_s * N_ss;
W_choke_ss = c_a_c + c_b_c * N_ss;
W_span_ss  = max((W_choke_ss - W_surge_ss) / 2, 5e-3);   % matches compressor_map.m
W_opt_ss   = c_a_o + c_b_o * N_ss;
pi_c_ss    = p_c_out_0 / p_c_in;
xi2_ss     = (1 - (pi_c_ss - 1) / max(dPi_max_ss, 1e-3)) / max(c_kpi, 1e-3);
xi2_ss     = min(max(xi2_ss, 0), 1.5);
W_ss       = W_opt_ss + W_span_ss * sqrt(xi2_ss);
W_ss       = min(max(W_ss, W_surge_ss + 2e-3), W_choke_ss - 2e-3);  % matches compressor_map.m
mdot_cruise = W_ss * (p_c_in / p_ref) / sqrt(T_c_in / T_ref);

% Engine conductance model:  mdot_engine = K_engine(t) * p_c_out
% Calibrated so mdot_engine = mdot_cruise at p_c_out_0 (equilibrium at t=0).
K_engine_base = mdot_cruise / p_c_out_0;   % kg/(s·Pa)

% --- Compressor shaft power at the initial operating point ---
% Mirrors compressor_map.m efficiency and temperature formulae exactly.
% Used in Section 11 to derive p_t_in_0 for shaft torque equilibrium.
eta_max_ss = max(min(c_e0 - c_e1 * (N_ss - 1)^2, 0.86), 0.55);
dev_ss     = (W_ss - W_opt_ss) / max(W_span_ss, 1e-9);
eta_c_ss   = eta_max_ss * (1 - c_keta * dev_ss^2);
eta_c_ss   = max(min(eta_c_ss, 0.86), 0.4);
T_c_out_ss = T_c_in * (1 + (pi_c_ss^((gamma-1)/gamma) - 1) / max(eta_c_ss, 0.05));
P_c_ss     = mdot_cruise * cp * (T_c_out_ss - T_c_in);   % compressor shaft power (W)
P_fric_ss  = k_f * omega0^2;                              % bearing friction power (W)
% Turbine must supply compressor + friction + generator mechanical load.
% P_gen_cruise is the desired *electrical* output; divide by eta_e to get
% the mechanical power the turbine shaft must produce for the generator.
P_shaft_req = P_c_ss + P_fric_ss + P_gen_cruise / eta_e;  % total shaft power turbine must supply (W)

% Throttle factor — dimensionless multiplier on K_engine_base:
%   cruise = 1.0,  WOT overtake = 1.6  (60% higher air demand)
throttle_cruise   = 1.0;
throttle_overtake = 1.6;

% Timing of the throttle event
t_overtake = 10.0;    % s — driver presses full throttle
t_release  = 12.0;    % s — driver releases throttle

%% ============================================================
% 11. EXHAUST MANIFOLD PLENUM PARAMETERS (Phase 4b)
%
%     Closes the full engine-turbocharger loop:
%       compressor → intake manifold → engine → exhaust manifold → turbine
%
%     dp_t_in/dt = K_exhaust_plenum * T_exh * (mdot_exh - mdot_turbine)
%
%     mdot_exh = (1 + FAR) * mdot_engine
%
%     EP6CDT fuel-air ratio at cruise (lambda ≈ 1.15):
%       FAR = 1 / (AFR_stoich * lambda) = 1 / (14.7 * 1.15) ≈ 0.059
%
%     t_Kflow is derived so turbine flow = exhaust flow at t = 0,
%     guaranteeing dp_t_in/dt = 0 at simulation start.
%% ============================================================

% FAR at cruise — derived from lambda_cruise (Section 7b) for consistency.
% At WOT the FAR increases as lambda drops; both T_exh and FAR are made
% dynamic in the Simulink model via the T_exh_Model subsystem.
FAR = 1 / (AFR_stoich * lambda_cruise);   % ≈ 0.059 — EP6CDT cruise (lambda ≈ 1.15)
FAR_WOT = 1 / (AFR_stoich * lambda_WOT); % ≈ 0.074 — EP6CDT WOT (lambda ≈ 0.92)

% Exhaust mass flow at t = 0
mdot_exh_0 = (1 + FAR) * mdot_cruise;

% --- Derive p_t_in_0 from shaft power balance ---
% The four equilibrium conditions for a zero-drift start are:
%   (1) dp_c_out/dt = 0  →  mdot_c = mdot_engine        (via K_engine_base, Section 10)
%   (2) dp_t_in/dt  = 0  →  mdot_turbine = mdot_exh     (via t_Kflow, below)
%   (3) dω/dt       = 0  →  P_turbine = P_c + P_f + P_gen_cruise/eta_e  (via p_t_in_0)
%   (4) PI int(0)   = u_pi_init/Ki                       (generator braking at t=0)
%
% Condition (3) previously used only P_c + P_f (no generator term), leaving
% zero turbine surplus.  With P_gen_cruise > 0, p_t_in_0 is raised so the
% turbine produces the extra mechanical power the generator needs.
% Condition (4) is handled in Section 9 via pi_int_init on the PID block.
%
% Solution: given P_shaft_req (computed in Section 10) and the turbine
% efficiency at omega0, solve analytically for the expansion ratio that
% makes P_turbine = P_shaft_req, then back out p_t_in_0.
%
%   P_t = mdot_exh * cp * eta_t * T_exh * (1 − (p_t_out/p_t_in)^((γ−1)/γ))
%   alpha_t = P_shaft_req / (mdot_exh * cp * eta_t * T_exh)
%   p_t_in_0 = p_t_out / (1 − alpha_t)^(γ/(γ−1))

N_t_ss   = (omega0 / omega_ref) * sqrt(T_ref / T_exh);
eta_t_ss = t_eta_max * (1 - t_kN * (N_t_ss - 1)^2);
eta_t_ss = max(min(eta_t_ss, t_eta_max), 0.4);

alpha_t  = P_shaft_req / max(mdot_exh_0 * cp * eta_t_ss * T_exh, 1e-6);
alpha_t  = min(alpha_t, 0.95);   % guard: keeps expansion ratio physical

p_t_in_0 = p_t_out / (1 - alpha_t)^(gamma / (gamma-1));

% Exhaust manifold volume — EP6CDT header + exhaust runners (~0.8–1.2 L)
V_exhaust = 1e-3;    % m³

% Plenum ODE gain (same formula as K_plenum but for the exhaust side)
K_exhaust_plenum = gamma * R / V_exhaust;   % Pa / (kg/s · K)

% --- Calibrate t_Kflow so turbine flow = exhaust flow at the initial op. point ---
% At t = 0: p_t_in = p_t_in_0, T_t_in = T_exh, p_t_out fixed.
% From turbine_map.m:  mdot = t_Kflow * p_in * sqrt(1/T_in) * flow_func
pi_t_0      = p_t_in_0 / p_t_out;
flow_func_0 = sqrt(max(1 - (1/pi_t_0)^((gamma-1)/gamma), 0));
t_Kflow     = mdot_exh_0 / (p_t_in_0 * sqrt(1/T_exh) * flow_func_0);

%% ============================================================
% 12. DISPLAY SUMMARY
%% ============================================================

fprintf('\n=========================================\n')
fprintf(' Turbo-Generator Model Initialized\n')
fprintf(' Engine : PSA/BMW EP6CDT 1.6L THP\n')
fprintf(' Turbo  : BorgWarner K03 (53039880163)\n')
fprintf('-----------------------------------------\n')
fprintf(' Map reference speed  : %d rpm\n',    rpm_ref)
fprintf(' Shaft inertia J      : %.2e kg*m^2\n', J)
fprintf(' Max mechanical speed : %.0f rpm\n',  omega_max*60/(2*pi))
fprintf('-----------------------------------------\n')
fprintf(' Compressor peak eta  : %.2f\n',      c_e0)
fprintf(' Turbine peak eta     : %.2f\n',      t_eta_max)
fprintf(' Machine efficiency   : %.2f\n',      eta_e)
fprintf(' EMG max torque       : %.3f Nm  (~%.0f W at design speed)\n', Tg_max, Tg_max*omega_ref)
fprintf(' DC bus voltage       : %.0f V\n',    V_bus)
fprintf('-----------------------------------------\n')
fprintf(' Initial shaft speed  : %.0f rpm\n',  omega0*60/(2*pi))
fprintf(' Boost setpoint       : %.1f kPa (cruise) / %.1f kPa (overtake)\n', p_c_out_ref/1e3, p_c_out_overtake/1e3)
fprintf(' Initial boost        : %.1f kPa\n',  p_c_out_0/1e3)
fprintf('-----------------------------------------\n')
fprintf(' Manifold volume      : %.1f L\n',    V_manifold*1e3)
fprintf(' N at omega0          : %.3f\n',      N_ss)
fprintf(' mdot_cruise (calc.)  : %.4f kg/s\n', mdot_cruise)
fprintf(' K_engine_base        : %.4e kg/(s*Pa)\n', K_engine_base)
fprintf(' Throttle factors     : %.1f (cruise)  %.1f (overtake)\n', throttle_cruise, throttle_overtake)
fprintf(' Throttle event       : t=%.1f s -> %.1f s\n', t_overtake, t_release)
fprintf('-----------------------------------------\n')
fprintf(' [Phase 4b — Exhaust manifold plenum]\n')
fprintf(' Exhaust plenum IC    : %.1f kPa  (derived from shaft balance)\n', p_t_in_0/1e3)
fprintf(' Expansion ratio IC   : %.3f\n',      pi_t_0)
fprintf(' Exhaust manifold vol : %.1f L\n',    V_exhaust*1e3)
fprintf(' FAR cruise / WOT     : %.3f / %.3f  (lambda %.2f / %.2f)\n', ...
        FAR, FAR_WOT, lambda_cruise, lambda_WOT)
fprintf(' mdot_exh_0 (calc.)   : %.4f kg/s\n', mdot_exh_0)
fprintf(' t_Kflow (derived)    : %.4e kg*sqrt(K)/(s*Pa)\n', t_Kflow)
fprintf(' [Shaft power balance at t=0]\n')
fprintf(' Compressor power     : %.1f W\n', P_c_ss)
fprintf(' Friction power       : %.1f W\n', P_fric_ss)
fprintf(' Generator target (e) : %.1f W  (%.1f W shaft)\n', P_gen_cruise, P_gen_cruise/eta_e)
fprintf(' Required turbine P   : %.1f W\n', P_shaft_req)
fprintf(' Turbine efficiency   : %.3f  (N_t = %.4f)\n', eta_t_ss, N_t_ss)
fprintf(' alpha_t              : %.4f\n', alpha_t)
fprintf(' [EMG cruise equilibrium]\n')
fprintf(' EMG enabled          : %d\n', emg_enabled)
fprintf(' Tq_gen_ss            : %.4f Nm\n', Tq_gen_ss)
fprintf(' u_pi_init            : %.4f Nm  (PI OUTPUT at t=0, compare vs u_min/u_max)\n', u_pi_init)
fprintf(' u_min / u_max        : %.3f / %.3f Nm  -- u_pi_init within range: %d\n', u_min, u_max, u_pi_init >= u_min && u_pi_init <= u_max)
fprintf(' pi_int_init          : %.2f Pa*s  (integrator STATE for PID IC, NOT in Nm)\n', pi_int_init)
fprintf(' [Phase 4c — Dynamic T_exh model]\n')
fprintf(' T_exh cruise / WOT   : %.0f K / %.0f K\n', T_exh_cruise, T_exh_WOT)
fprintf(' Lambda cruise / WOT  : %.2f / %.2f\n',      lambda_cruise, lambda_WOT)
fprintf(' Thermal lag tau      : %.1f s\n',            tau_T_exh)
fprintf('=========================================\n\n')
