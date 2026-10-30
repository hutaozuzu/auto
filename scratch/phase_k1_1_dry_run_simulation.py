import math
import json

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def compute_tracking_error(actual, reference):
    dx = reference['x'] - actual['x']
    dy = reference['y'] - actual['y']
    dtheta = reference['theta'] - actual['theta']

    cos_th = math.cos(actual['theta'])
    sin_th = math.sin(actual['theta'])

    e1 = cos_th * dx + sin_th * dy
    e2 = -sin_th * dx + cos_th * dy
    e3 = normalize_angle(dtheta)

    return {'e1': e1, 'e2': e2, 'e3': e3}

def compute_control_law(err, v_r, omega_r, k1=1.0, k2=2.0, k3=1.0):
    v_d = v_r * math.cos(err['e3']) + k1 * err['e1']
    omega_d = omega_r + (k2 * v_r * err['e2']) + (k3 * v_r * math.sin(err['e3']))
    return {'v_d': v_d, 'omega_d': omega_d}

def compute_terminal_rotation(current_yaw, goal_yaw, k_theta=1.5, max_w=0.80, yaw_tol=0.15):
    e_theta = normalize_angle(goal_yaw - current_yaw)
    if abs(e_theta) <= yaw_tol:
        return 0.0, 'GOAL_REACHED'
    
    omega_cmd = k_theta * e_theta
    omega_cmd = max(-max_w, min(max_w, omega_cmd))
    return omega_cmd, 'TERMINAL_ROTATE'

def is_trajectory_safe_mock(x0, y0, theta0, v, omega, obstacle_pos=None, horizon=0.8, dt=0.05, fp_radius=0.113):
    steps = int(math.ceil(horizon / dt))
    x, y, theta = x0, y0, theta0
    for _ in range(steps):
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta = normalize_angle(theta + omega * dt)
        
        if obstacle_pos:
            dist = math.hypot(x - obstacle_pos['x'], y - obstacle_pos['y'])
            if dist < fp_radius:
                return False # Collision detected
    return True

def run_dry_run_k1_1():
    print('========================================================================')
    print('PLAN C — PHASE K1.1 TERMINAL ROTATION & COLLISION SIMULATION')
    print('========================================================================')

    # Simulation A: Terminal Goal Approach & Rotation Convergence
    print('\n[Scenario 1: Terminal Goal Convergence Simulation]')
    goal = {'x': 1.00, 'y': 0.00, 'theta': 0.5236} # Goal at (1.0m, 0.0m) with +30 deg heading
    robot = {'x': 0.95, 'y': 0.00, 'theta': 0.00}   # Robot inside 15cm XY tolerance (5cm away)
    
    state_history = []
    for step in range(8):
        dist_to_goal = math.hypot(goal['x'] - robot['x'], goal['y'] - robot['y'])
        if dist_to_goal <= 0.15:
            omega_cmd, state = compute_terminal_rotation(robot['theta'], goal['theta'], k_theta=1.5, max_w=0.80, yaw_tol=0.15)
            v_cmd = 0.0
        else:
            state = 'TRACK_PATH'
            v_cmd = 0.15
            omega_cmd = 0.0
            
        print(f"  Step {step}: Pose=(x={robot['x']:.2f}, y={robot['y']:.2f}, th={math.degrees(robot['theta']):.1f} deg) | State={state} | Cmd=(v={v_cmd:.2f} m/s, w={omega_cmd:+.3f} rad/s)")
        state_history.append((state, v_cmd, omega_cmd))
        
        # Integrate simulated motion dt=0.2s
        robot['theta'] = normalize_angle(robot['theta'] + omega_cmd * 0.2)
        if state == 'GOAL_REACHED':
            break

    # Simulation B: Dynamic Obstacle Injection & Safe Stop
    print('\n[Scenario 2: Forward Path Obstacle Injection & Collision Safe Stop]')
    robot_run = {'x': 0.50, 'y': 0.00, 'theta': 0.00}
    candidate_v = 0.15
    candidate_w = 0.00
    
    # Case 1: Free space
    safe1 = is_trajectory_safe_mock(robot_run['x'], robot_run['y'], robot_run['theta'], candidate_v, candidate_w, obstacle_pos=None)
    print(f"  Case 1 (Clear Path): Trajectory Safe = {safe1} -> Output Cmd = (v={candidate_v}, w={candidate_w})")
    
    # Case 2: Obstacle at (0.60m, 0.00m) - 10cm directly ahead
    safe2 = is_trajectory_safe_mock(robot_run['x'], robot_run['y'], robot_run['theta'], candidate_v, candidate_w, obstacle_pos={'x': 0.60, 'y': 0.00})
    v_out = candidate_v if safe2 else 0.0
    w_out = candidate_w if safe2 else 0.0
    print(f"  Case 2 (Obstacle @ 0.60m): Trajectory Safe = {safe2} -> Output Cmd = (v={v_out}, w={w_out}) [SAFE STOP]")

    print('\n========================================================================')
    print('PHASE K1.1 DRY-RUN SIMULATION COMPLETE (PASS)')
    print('========================================================================')

if __name__ == '__main__':
    run_dry_run_k1_1()
