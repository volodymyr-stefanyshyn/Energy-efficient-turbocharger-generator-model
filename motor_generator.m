function [Tq, P_elec] = motor_generator(omega, Pe_demand, Tq_assist, eta_e, Tg_max, V_bus)
%#codegen
%
% motor_generator  Electric motor / generator torque model.
%
% Operating mode is determined exclusively by which input is positive.
% Exactly one of Pe_demand or Tq_assist must be positive at any time;
% any other combination is a configuration error.
%
%   Pe_demand > 0, Tq_assist == 0  →  generator mode
%   Pe_demand == 0, Tq_assist > 0  →  motor-assist mode
%   both > 0                        →  configuration error (assert fires)
%   both == 0                       →  off (zero torque, zero electrical power)
%
% Inputs
%   omega      - shaft angular velocity (rad/s)
%   Pe_demand  - requested electrical generation power (W).
%                Set to 0 when not in generator mode.
%   Tq_assist  - requested motor-assist torque (Nm).
%                Set to 0 when not in motor mode.
%   eta_e      - electro-mechanical round-trip efficiency (-)
%   Tg_max     - maximum allowable machine torque (Nm)
%   V_bus      - DC bus voltage (V); used for current calculation
%
% Outputs
%   Tq     - mechanical torque exchanged with shaft (Nm).
%              > 0 : machine brakes the shaft (generator, load torque).
%              < 0 : machine drives the shaft (motor, assist torque).
%            Wire to the shaft Sum block with a NEGATIVE sign so that:
%              generator subtracts from net torque (brakes shaft), and
%              motor adds to net torque (assists shaft).
%   P_elec - actual electrical power at the DC bus terminals (W).
%              > 0 : power delivered TO the bus (generator mode).
%              < 0 : power drawn FROM the bus (motor mode).
%            Divide by V_bus externally to obtain bus current (A).

epsw = 5;  % rad/s — prevents division blow-up below ~50 rpm

gen_active   = Pe_demand > 0;
motor_active = Tq_assist > 0;

assert(~(gen_active && motor_active), ...
    'motor_generator: Pe_demand and Tq_assist are both positive. Exactly one must be active.');

if ~gen_active && ~motor_active
    Tq     = 0;
    P_elec = 0;
    return;
end

if gen_active
    % ------------------------------------------------------------------ %
    % Generator mode                                                       %
    %   The machine converts shaft mechanical power into electrical power. %
    %   Required mechanical torque to deliver Pe_demand to the bus:        %
    %     Tq = Pe_demand / (eta_e * omega)                                 %
    %   Actual electrical power delivered, accounting for clamping:        %
    %     P_elec = eta_e * Tq * omega                                      %
    % ------------------------------------------------------------------ %
    Tq     = Pe_demand / (eta_e * (omega + epsw));
    Tq     = min(max(Tq, 0), Tg_max);          % clamp to machine limits
    P_elec = eta_e * Tq * (omega + epsw);      % power at bus terminals (> 0)

else
    % ------------------------------------------------------------------ %
    % Motor-assist mode                                                    %
    %   The machine draws electrical power from the bus and adds torque    %
    %   to the shaft.                                                      %
    %     Tq_actual = clamped assist torque                                %
    %     P_elec = -(Tq_actual * omega) / eta_e  (drawn from bus)         %
    % ------------------------------------------------------------------ %
    Tq_actual = min(Tq_assist, Tg_max);
    Tq        = -Tq_actual;                         % negative → assists shaft
    P_elec    = -(Tq_actual * (omega + epsw) / eta_e); % power from bus (< 0)

end

% V_bus is available for downstream current calculation if needed:
%   I_bus = P_elec / V_bus   (> 0 charging bus, < 0 discharging bus)
% To expose bus current, add I_bus as an output port in the wrapper block
% and compute it here: I_bus = P_elec / V_bus;

end
