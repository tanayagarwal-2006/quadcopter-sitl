import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def plot_all(logger):
    t = logger.time
    fig = plt.figure(figsize=(20, 12))

    gs = gridspec.GridSpec(3, 3, width_ratios=[1, 1, 1.8])

    ax_pos = fig.add_subplot(gs[0, 0])
    ax_att = fig.add_subplot(gs[0, 1])
    ax_vel = fig.add_subplot(gs[1, 0])
    ax_rates = fig.add_subplot(gs[1, 1])
    ax_acc = fig.add_subplot(gs[2, 0])

    ax_mot = fig.add_subplot(gs[2, 1])
    
    ax_traj = fig.add_subplot(gs[:, 2], projection='3d') 

    # Position
    ax_pos.plot(t, logger.x, label="North (X)")
    ax_pos.plot(t, logger.y, label="East (Y)")
    ax_pos.plot(t, logger.z, label="Down (Z)")
    ax_pos.set_title("Position (NED)")
    ax_pos.set_xlabel("Time (s)")
    ax_pos.set_ylabel("Distance (m)")
    ax_pos.grid()
    ax_pos.legend()

    # Attitude
    ax_att.plot(t, logger.x_deg_phi, label="Roll")
    ax_att.plot(t, logger.y_deg_theta, label="Pitch")
    ax_att.plot(t, logger.z_deg_psi, label="Yaw")
    ax_att.set_title("Attitude")
    ax_att.set_xlabel("Time (s)")
    ax_att.set_ylabel("Angle (deg)")
    ax_att.grid()
    ax_att.legend()

    # Velocity
    ax_vel.plot(t, logger.vx, label="North (Vx)")
    ax_vel.plot(t, logger.vy, label="East (Vy)")
    ax_vel.plot(t, logger.vz, label="Down (Vz)")
    ax_vel.set_title("Velocity")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("m/s")
    ax_vel.grid()
    ax_vel.legend()

    # Rates
    ax_rates.plot(t, logger.roll_rate, label="P (Roll Rate)")
    ax_rates.plot(t, logger.pitch_rate, label="Q (Pitch Rate)")
    ax_rates.plot(t, logger.yaw_rate, label="R (Yaw Rate)")
    ax_rates.set_title("Angular Rates (Body Frame)")
    ax_rates.set_ylabel("deg/s")
    ax_rates.grid()
    ax_rates.legend()

    # Acceleration
    ax_acc.plot(t, logger.ax, label="North (Ax)")
    ax_acc.plot(t, logger.ay, label="East (Ay)")
    ax_acc.plot(t, logger.az, label="Down (Az)")
    ax_acc.set_title("Acceleration")
    ax_acc.set_xlabel("Time (s)")
    ax_acc.set_ylabel("m/s²")
    ax_acc.grid()
    ax_acc.legend()

    # Motors
    ax_mot.plot(t, logger.m1, label="M1 (Front-Right)")
    ax_mot.plot(t, logger.m2, label="M2 (Rear-Left)")
    ax_mot.plot(t, logger.m3, label="M3 (Front-Left)")
    ax_mot.plot(t, logger.m4, label="M4 (Rear-Right)")
    ax_mot.set_title("Motor Commands")
    ax_mot.set_xlabel("Time (s)")
    ax_mot.set_ylabel("Speed (rad/s)")
    ax_mot.grid()
    ax_mot.legend(fontsize='small')

    # 3D Trajectory
    ax_traj.plot(logger.x, logger.y, logger.z, linewidth=2)
    if len(logger.x) > 0:
        ax_traj.scatter(logger.x[0], logger.y[0], logger.z[0], s=80, label="Start")
        ax_traj.scatter(logger.x[-1], logger.y[-1], logger.z[-1], s=80, marker='x', label="End")

    ax_traj.set_title("Drone Trajectory (NED)")
    ax_traj.set_xlabel("North (X)")
    ax_traj.set_ylabel("East (Y)")
    ax_traj.set_zlabel("Down (Z)")
    
    ax_traj.invert_zaxis()

    ax_traj.legend()

    plt.tight_layout()
    plt.show()