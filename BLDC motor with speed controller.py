# BLDC motor with speed controller
import math
import matplotlib.pyplot as plt

# ----------------------------- Motor parameters -----------------------------
# BLDC electrical motor parameters
R_s = 0.05      # Phase resistance (ohm). Models copper loss: V_R = R_s * I
L_s = 0.3e-3    # Phase inductance (H). Electrical dynamics: V_L = L_s * dI/dt
K_e = 0.08      # Back-EMF constant (V/(rad/s)). Back-EMF: E = K_e * omega * f(theta)
K_t = 0.08      # Torque constant (Nm/A). Torque: T = K_t * (f_a*I_a + f_b*I_b + f_c*I_c)
V_dc = 400      # DC bus voltage (V) feeding the inverter
p = 4           # Pole pairs. Electrical angle theta_e = p * mechanical_angle theta_m
I_max = 300     # Current limit (A). Used to clamp phase currents

# BLDC mechanical motor parameters
J = 0.015       # Rotor inertia (kg*m^2). Mechanical dynamics: J*domega/dt = sum(torques)
B = 0.001       # Viscous friction (N*m*s/rad). Friction torque ~ B*omega
T_f = 0.2       # Coulomb friction (N*m). (Not used in your final dynamics right now)
omega_max_rpm = 12000  # Max speed (rpm) (not enforced in your code)
dt = 0.00001    # Simulation time-step (s)

integral_error = 0     # Integrator state for PI controller (∫ error dt)
Ia = 0
Ib = 0
Ic = 0
theta_m = 0.001        # Mechanical angle (rad)
pi = math.pi

# ----------------------------- Speed controller -----------------------------
def pid_controller(omega_rps_ref, omega_rps):
    # PI speed controller
    # error = omega_ref - omega_actual
    # u = Kp*error + Ki*∫error dt
    # Here u is a normalized torque/voltage command, later used as PWM duty (magnitude).

    global integral_error

    Kp = 0.02
    Ki = 0.01

    error = omega_rps_ref - omega_rps  # speed tracking error (rad/s)

    # Small-speed "deadzone" reset: avoids integrator drift around zero speed
    if omega_rps_ref < 0.5 and omega_rps < 0.5:
        integral_error = 0
        controller_output = 0
    else:
        # PI output using current integral_error (integration updated later)
        controller_output = Kp * error + Ki * integral_error

        # Saturation (command limiting):
        # u_sat = clamp(u, [-1, +1]) so duty stays within feasible inverter range
        controller_output = max(-1, min(1.0, controller_output))

        # Anti-windup rule:
        # integrate error only when (a) not saturated, OR (b) saturated but error drives output back inward.
        # This prevents ∫error from blowing up while actuator is saturated.
        if (0.0 < controller_output < 1.0) or \
           (controller_output == 1.0 and error < 0.0) or \
           (controller_output == 0.0 and error > 0.0):
            integral_error += error * dt  # numerical integration: ∫e dt ≈ ∫ (e[k]) dt

    return controller_output

# ----------------------------- Inverter model -----------------------------
def phase_voltage(switchA, switchB, switchC, controller_output):
    # Inverter phase voltage output (star connection with floating neutral).
    # switch phase states: +1 means phase tied to +Vdc/2 (high switch on),
    #                     -1 means phase tied to -Vdc/2 (low switch on),
    #                      0 means phase is floating (open).
    #
    # duty = |controller_output| is PWM duty (0..1) scaling the applied phase voltage magnitude.

    duty = abs(controller_output)

    # Phase-to-DC midpoint voltages (before neutral shift)
    # Va_p ∈ {+Vdc/2, -Vdc/2, 0} scaled by duty.
    Va_p = switchA * (V_dc / 2.0) * duty
    Vb_p = switchB * (V_dc / 2.0) * duty
    Vc_p = switchC * (V_dc / 2.0) * duty

    # Neutral (floating star point) voltage:
    # For a 3-phase star load with no neutral wire, Va+Vb+Vc must sum to 0.
    # Vn is chosen so that (Va_p-Vn) + (Vb_p-Vn) + (Vc_p-Vn) = 0
    # => Vn = (Va_p + Vb_p + Vc_p) / 3
    Vn = (Va_p + Vb_p + Vc_p) / 3.0

    # Actual phase-to-neutral voltages applied to motor windings
    Va = Va_p - Vn
    Vb = Vb_p - Vn
    Vc = Vc_p - Vn

    return Va, Vb, Vc

# ----------------------------- Trapezoidal back-EMF shape -----------------------------
def trap_120(theta_deg):
    # Returns trapezoidal waveform value f(theta) for 120-degree conduction BLDC.
    # Output is in [-1, +1] and represents normalized back-EMF shape:
    # E_phase = K_e * omega * f(theta)
    #
    # Piecewise trapezoid over electrical angle (deg).
    th = theta_deg % 360.0

    if 0 <= th < 60:
        return th / 60.0                 # ramp up 0 -> +1
    elif 60 <= th < 180:
        return 1.0                       # flat at +1
    elif 180 <= th < 240:
        return 1.0 - (th - 180) / 60.0   # ramp down +1 -> 0
    elif 240 <= th < 300:
        return -(th - 240) / 60.0        # ramp down 0 -> -1
    else:
        return -1.0                      # flat at -1

def inverter(elec_angle):
    # Commutation logic (6-step / 120° conduction).
    # Each 60° electrical sector picks which phase is high (+), which is low (-), and which floats (0).
    #
    # Convert electrical angle from rad -> deg and wrap to [0, 360)
    elec_angle = (elec_angle * 180.0 / math.pi) % 360.0

    if elec_angle >= 0 and elec_angle < 60:
        switchA, switchB, switchC = 1, -1, 0
    elif elec_angle >= 60 and elec_angle < 120:
        switchA, switchB, switchC = 1, 0, -1
    elif elec_angle >= 120 and elec_angle < 180:
        switchA, switchB, switchC = 0, 1, -1
    elif elec_angle >= 180 and elec_angle < 240:
        switchA, switchB, switchC = -1, 1, 0
    elif elec_angle >= 240 and elec_angle < 300:
        switchA, switchB, switchC = -1, 0, 1
    else:
        switchA, switchB, switchC = 0, -1, 1

    # NOTE: This function uses controller_output, but controller_output is not an argument.
    # It works only because controller_output exists in the global loop scope.
    # Mathematically: negative controller_output means reverse torque command.
    # Implemented by flipping commutation signs (swap high/low), i.e., reverse current direction.
    if controller_output < 0:
        switchA, switchB, switchC = -switchA, -switchB, -switchC

    return switchA, switchB, switchC

# ----------------------------- Electrical dynamics -----------------------------
def electrical_dynamics(Va, Vb, Vc, omega_rps, theta_e, switchA, switchB, switchC):
    # Computes phase currents by integrating the phase RL differential equation:
    # For each phase:  V_phase = R_s*i + L_s*(di/dt) + E_phase
    # => di/dt = (V_phase - R_s*i - E_phase) / L_s
    #
    # Back-EMF is trapezoidal:
    # E_a = K_e * omega * f_a(theta_e), etc.

    global Ia, Ib, Ic

    # Electrical angle in degrees for trapezoid function
    theta_deg = (theta_e * 180.0 / math.pi)

    # Back-EMF shape functions (120° shifted)
    fa = trap_120(theta_deg)
    fb = trap_120(theta_deg - 120.0)
    fc = trap_120(theta_deg - 240.0)

    # Back-EMF per phase (V)
    E_a = K_e * omega_rps * fa
    E_b = K_e * omega_rps * fb
    E_c = K_e * omega_rps * fc

    # 120° conduction means one phase is floating each sector.
    # For the floating phase, current is not directly forced by applied voltage,
    # and under star connection we enforce: Ia + Ib + Ic = 0  (no neutral current).
    #
    # So you integrate the two conducting phases and compute the third as:
    # I_float = -(I_other1 + I_other2)

    if switchA == 0:
        # A floating -> integrate B and C currents
        dIB_dt = (Vb - R_s * Ib - E_b) / L_s
        dIC_dt = (Vc - R_s * Ic - E_c) / L_s
        Ib += dIB_dt * dt                # Euler integration: I[k+1] = I[k] + dI/dt * dt
        Ic += dIC_dt * dt
        Ia = -(Ib + Ic)                  # KCL constraint (star, no neutral)

    elif switchB == 0:
        # B floating -> integrate A and C
        dIA_dt = (Va - R_s * Ia - E_a) / L_s
        dIC_dt = (Vc - R_s * Ic - E_c) / L_s
        Ia += dIA_dt * dt
        Ic += dIC_dt * dt
        Ib = -(Ia + Ic)

    else:
        # C floating -> integrate A and B
        dIA_dt = (Va - R_s * Ia - E_a) / L_s
        dIB_dt = (Vb - R_s * Ib - E_b) / L_s
        Ia += dIA_dt * dt
        Ib += dIB_dt * dt
        Ic = -(Ia + Ib)

    # Current limiting (clamping) after integration:
    # Ensures |I| ≤ I_max. This is a numerical/physical constraint representing current protection.
    Ia = max(-I_max, min(I_max, Ia))
    Ib = max(-I_max, min(I_max, Ib))
    Ic = max(-I_max, min(I_max, Ic))

    # Electromagnetic torque:
    # For trapezoidal BLDC model: T = K_t * Σ (f_phase * i_phase)
    # This matches power consistency: electrical power ~ Σ(E_phase*i_phase) and mechanical power ~ T*omega
    Torque_motor = K_t * (fa * Ia + fb * Ib + fc * Ic)

    return Torque_motor, Ia, Ib, Ic, fa, fb, fc

# ----------------------------- Mechanical dynamics -----------------------------
def mechanical_dynamics(Torque_motor, omega_rps):
    # Rotor dynamics:
    # J * domega/dt = Torque_motor - Torque_load - B*omega  (viscous friction)
    # => alpha = domega/dt = (T_motor - T_load - B*omega) / J

    Torque_load = 0  # external load torque (Nm) (set to 0 here)

    global theta_m

    alpha_rps = (Torque_motor - Torque_load - B * omega_rps) / J  # angular acceleration (rad/s^2)
    omega_rps = omega_rps + alpha_rps * dt                        # Euler integrate speed
    theta_m = theta_m + omega_rps * dt                            # integrate mechanical position

    # Electrical angle for commutation/back-EMF:
    # theta_e = p * theta_m  (pole-pairs conversion)
    theta_e = theta_m * p

    # Wrap to [-pi, +pi) to keep angle bounded and avoid numeric growth:
    # theta_wrapped = ((theta + pi) mod 2pi) - pi
    theta_e = (theta_e + math.pi) % (2 * math.pi) - math.pi

    return omega_rps, theta_e

# ----------------------------- Simulation loop -----------------------------
sim_time = 35  # simulation time (s)
t = 0          # initial time

# Log vectors
time_vector = []
omega_rps_actual_vector = []
omega_rps_ref_vector = []
actual_torque = []
controller_output_vector = []
Va_vector, Vb_vector, Vc_vector = [], [], []
Ia_vector, Ib_vector, Ic_vector = [], [], []
fa_vector, fb_vector, fc_vector = [], [], []
elec_angle_vector = []

omega_rps = 0
elec_angle = 0

while t < sim_time:

    # Speed reference profile (rad/s). Piecewise steps:
    # 0–2s: 0, 2–15s: 500, 15–25s: 300, else: 500
    if t < 2:
        omega_ref = 0
    elif 2 < t < 15:
        omega_ref = 500
    elif 15 < t < 25:
        omega_ref = 300
    else:
        omega_ref = 500

    omega_rps_ref = omega_ref

    # Controller computes normalized command u in [-1, 1]
    controller_output = pid_controller(omega_rps_ref, omega_rps)

    # Commutation based on electrical angle
    switchA, switchB, switchC = inverter(elec_angle)

    # Convert commutation + duty to phase voltages
    Va, Vb, Vc = phase_voltage(switchA, switchB, switchC, controller_output)

    # Electrical dynamics gives currents + torque
    Torque_motor, Ia, Ib, Ic, fa, fb, fc = electrical_dynamics(
        Va, Vb, Vc, omega_rps, elec_angle, switchA, switchB, switchC
    )

    # Mechanical dynamics updates speed and electrical angle
    omega_rps, elec_angle = mechanical_dynamics(Torque_motor, omega_rps)

    # Log signals
    time_vector.append(t)
    omega_rps_actual_vector.append(omega_rps)
    omega_rps_ref_vector.append(omega_rps_ref)
    controller_output_vector.append(controller_output)
    Va_vector.append(Va); Vb_vector.append(Vb); Vc_vector.append(Vc)
    Ia_vector.append(Ia); Ib_vector.append(Ib); Ic_vector.append(Ic)
    fa_vector.append(fa); fb_vector.append(fb); fc_vector.append(fc)
    elec_angle_vector.append(elec_angle)
    actual_torque.append(Torque_motor)

    t = t + dt

# ----------------------------- Plotting -----------------------------
plt.figure()
plt.plot(time_vector, omega_rps_ref_vector, label="omega_ref")
plt.plot(time_vector, omega_rps_actual_vector, label="omega_actual")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, controller_output_vector, label="Controller (duty)")
plt.xlabel("Time (s)")
plt.ylabel("Output Controller (0-1)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, Va_vector, label="Voltage Phase A (V)")
plt.plot(time_vector, Vb_vector, label="Voltage Phase B (V)")
plt.plot(time_vector, Vc_vector, label="Voltage Phase C (V)")
plt.xlabel("Time (s)")
plt.ylabel("Voltage (V)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, actual_torque, label="Torque (Nm)")
plt.xlabel("Time (s)")
plt.ylabel("Torque (Nm)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, elec_angle_vector, label="Electrical Angle (radian)")
plt.xlabel("Time (s)")
plt.ylabel("Electrical Angle (radian)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, Ia_vector, label="Current Phase A (A)")
plt.plot(time_vector, Ib_vector, label="Current Phase B (A)")
plt.plot(time_vector, Ic_vector, label="Current Phase C (A)")
plt.xlabel("Time (s)")
plt.ylabel("Current (A)")
plt.legend()
plt.grid(True)

plt.figure()
plt.plot(time_vector, fa_vector, label="fa")
plt.plot(time_vector, fb_vector, label="fb")
plt.plot(time_vector, fc_vector, label="fc")
plt.xlabel("Time (s)")
plt.ylabel("Back-EMF shape f(theta)")
plt.legend()
plt.grid(True)

plt.show()
