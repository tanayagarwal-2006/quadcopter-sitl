import time
import numpy as np
from config import PARAMS
from physics_engine import current_drone_state

from udp_bridge import SITLBridge
from controller_adapter import build_euler_controller_packet
from packet_builder import build_euler_packet

def create_initial_state():
    return {
        "pos": np.zeros(3),
        "vel": np.zeros(3),
        "omegas": np.zeros(3),
        "q": np.array([1.0, 0.0, 0.0, 0.0]),
        "acc_world": np.zeros(3),
        "motor_actual": np.zeros(4)
    }

def main():
    bridge = SITLBridge()
    state = create_initial_state()
    #motor_speeds = np.zeros(4)
    motor_speeds = np.array([650.0, 650.0, 650.0, 650.0])
    dt = PARAMS["dt"]
    tick = 0

    # --- TABLE HEADER ---
    # Print exactly once before the high-frequency loop starts
    print("\n" + "="*155)
    print(
        f"{'TIME(s)':>7} | "
        f"{'ANGLES (R/P/Y) deg':>20} | "
        f"{'RATES (P/Q/R)':>20} | "
        f"{'POS (X/Y/Z)':>20} | "
        f"{'VEL (X/Y/Z)':>20} | "
        f"{'ACCEL (X/Y/Z)':>20} | "
        f"{'MOTORS CMD':>19} | "
        f"{'MOTORS ACT':>19}"
    )
    print("="*155)

    while True:
        state = current_drone_state(motor_speeds,state,PARAMS)

        controller_state = build_euler_controller_packet(state)
        packet = build_euler_packet(controller_state["angles"],controller_state["rates"])

        bridge.send_packet(packet)
        motors = bridge.receive_motors()

        #if motors is not None:
         #   motor_speeds = motors

        if motors is None:
            break

        motor_speeds=motors

        # --- CLEAN TABLE LOGGING ---
        if tick % 100 == 0:
            current_time = tick * dt
            
            # Unpack and convert angles to degrees for readability
            r_deg, p_deg, y_deg = np.degrees(controller_state['angles'])
            
            # Unpack the rest of the vectors
            p_rate, q_rate, r_rate = state['omegas']
            px, py, pz = state['pos']
            vx, vy, vz = state['vel']
            ax, ay, az = state['acc_world']
            
            m1_c, m2_c, m3_c, m4_c = motor_speeds
            m1_a, m2_a, m3_a, m4_a = state['motor_actual']

            # Enforce fixed widths and alignment to prevent column jitter
            # Precision is truncated to save horizontal terminal space
            print(
                f"{current_time:>7.2f} | "
                f"{r_deg:>6.1f} {p_deg:>6.1f} {y_deg:>6.1f} | "
                f"{p_rate:>6.2f} {q_rate:>6.2f} {r_rate:>6.2f} | "
                f"{px:>6.2f} {py:>6.2f} {pz:>6.2f} | "
                f"{vx:>6.2f} {vy:>6.2f} {vz:>6.2f} | "
                f"{ax:>6.2f} {ay:>6.2f} {az:>6.2f} | "
                f"{m1_c:>4.0f} {m2_c:>4.0f} {m3_c:>4.0f} {m4_c:>4.0f} | "
                f"{m1_a:>4.0f} {m2_a:>4.0f} {m3_a:>4.0f} {m4_a:>4.0f}"
            )

        tick += 1
        #time.sleep(dt)

if __name__ == "__main__":
    main()