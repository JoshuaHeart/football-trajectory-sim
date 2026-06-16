import math
import plotly.graph_objects as go
import streamlit as st

# --- STREAMLIT PAGE CONFIG ---
st.set_page_config(page_title="RK4 3D Flight Engine", layout="wide", initial_sidebar_state="expanded")

# Force Dark Mode Styling
st.markdown("""
    <style>
    .main { background-color: #0e1117; color: #ffffff; }
    .sidebar .sidebar-content { background-color: #161b22; }
    h1, h2, h3 { color: #00fff0 !important; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ RK4 3D Aerodynamic Flight Trajectory Engine")
st.write("High-fidelity fluid dynamics simulation using 4th-Order Runge-Kutta integration.")

# --- SIDEBAR INPUT CONTROLS ---
st.sidebar.header("📥 Initial Launch Parameters")
v0 = st.sidebar.slider("Initial Velocity (v0 - m/s)", 5.0, 35.0, 13.2, 0.1)
theta_deg = st.sidebar.slider("Vertical Launch Angle (θ - degrees)", -10.0, 50.0, 25.2, 0.1)
phi_deg = st.sidebar.slider("Horizontal Launch Angle (φ - degrees)", -45.0, 45.0, 0.0, 0.1)

st.sidebar.header("🌀 Spin Vector Configuration")
spin_mag_init = st.sidebar.slider("Spin Magnitude (rad/s)", 0.0, 100.0, 47.2, 0.1)
nx = st.sidebar.slider("Spin Axis X (nx - Back/Topspin)", -1.0, 1.0, 0.0, 0.1)
ny = st.sidebar.slider("Spin Axis Y (ny)", -1.0, 1.0, 0.0, 0.1)
nz = st.sidebar.slider("Spin Axis Z (nz - Sidespin)", -1.0, 1.0, 1.0, 0.1)

# Ensure spin axis is a normalized unit vector
axis_mag = math.sqrt(nx**2 + ny**2 + nz**2)
if axis_mag > 0:
    nx /= axis_mag
    ny /= axis_mag
    nz /= axis_mag
else:
    nx, ny, nz = 0.0, 0.0, 1.0

# --- SIMULATION CONSTANTS ---
MASS = 0.400
RADIUS = 0.11
RHO = 1.184
G = 9.81
MU = 0.0000185
LENG = 0.22
A = math.pi * (RADIUS**2)
I_DIST = 35.0
GOAL_CENTER_K = -1.2
DT = 0.002  # RK4 allows for a highly stable, slightly larger time step
SPIN_DECAY = 0.999

# --- HELPER FUNCTIONS FOR RK4 ---
def get_acceleration(v_vec, current_spin):
    """Calculates instantaneous 3D acceleration vector due to Gravity, Drag, and Magnus forces."""
    vi, vj, vk = v_vec
    v_mag = math.sqrt(vi**2 + vj**2 + vk**2)
    if v_mag == 0:
        return 0.0, -G, 0.0

    # Fluid dynamics configurations
    Sp = (RADIUS * current_spin) / v_mag
    Cl = 0.6 * Sp
    Re = RHO * v_mag * LENG / MU

    if Re > 150000:
        Cd = 0.20
    elif 100000 <= Re <= 150000:
        Cd = 0.30
    else:
        Cd = 0.50

    S = 0.5 * RHO * A * v_mag

    # Calculate Magnus components via cross product framework (Spin x Velocity)
    # Spin components: wx = current_spin * nx, etc.
    wx, wy, wz = current_spin * nx, current_spin * ny, current_spin * nz
    
    # Cross product components: w x v
    cross_i = wy * vk - wz * vj
    cross_j = wz * vi - wx * vk
    cross_k = wx * vj - wy * vi

    # Force components
    f_drag_i = -Cd * S * vi
    f_drag_j = -Cd * S * vj
    f_drag_k = -Cd * S * vk

    f_magnus_i = Cl * 0.5 * RHO * A * RADIUS * cross_i
    f_magnus_j = Cl * 0.5 * RHO * A * RADIUS * cross_j
    f_magnus_k = Cl * 0.5 * RHO * A * RADIUS * cross_k

    # Total accelerations (F / m)
    ai = (f_drag_i + f_magnus_i) / MASS
    aj = (f_drag_j + f_magnus_j) / MASS - G
    ak = (f_drag_k + f_magnus_k) / MASS

    return ai, aj, ak

# --- INITIAL STATES ---
theta = math.radians(theta_deg)
phi = math.radians(phi_deg)

# System State Vector: [i, j, k, vi, vj, vk]
state = [
    0.0, 
    0.11, 
    0.0, 
    v0 * math.cos(theta) * math.cos(phi), 
    v0 * math.sin(theta), 
    v0 * math.cos(theta) * math.sin(phi)
]

spin_mag = spin_mag_init

i_path, j_path, k_path = [state[0]], [state[1]], [state[2]]

# --- RK4 INTEGRATION LOOP ---
max_iterations = 25000
iterations = 0

while state[1] >= 0.11 and state[0] < I_DIST:
    iterations += 1
    if iterations > max_iterations:
        break

    # State extraction
    curr_pos = state[0:3]
    curr_vel = state[3:6]

    # --- RK4 COEFFICIENTS COMPONENT ---
    # k1 steps
    k1_vel = curr_vel
    k1_acc = get_acceleration(curr_vel, spin_mag)

    # k2 steps
    k2_vel = [curr_vel[m] + 0.5 * DT * k1_acc[m] for m in range(3)]
    k2_acc = get_acceleration(k2_vel, spin_mag)

    # k3 steps
    k3_vel = [curr_vel[m] + 0.5 * DT * k2_acc[m] for m in range(3)]
    k3_acc = get_acceleration(k3_vel, spin_mag)

    # k4 steps
    k4_vel = [curr_vel[m] + DT * k3_acc[m] for m in range(3)]
    k4_acc = get_acceleration(k4_vel, spin_mag)

    # Update Position and Velocity states using weighted RK4 averages
    for m in range(3):
        state[m] += (DT / 6.0) * (k1_vel[m] + 2.0 * k2_vel[m] + 2.0 * k3_vel[m] + k4_vel[m])
        state[m+3] += (DT / 6.0) * (k1_acc[m] + 2.0 * k2_acc[m] + 2.0 * k3_acc[m] + k4_acc[m])

    # Store computed positions
    i_path.append(state[0])
    j_path.append(state[1])
    k_path.append(state[2])

    # Apply exponential decay to spin magnitude over time step
    spin_mag *= SPIN_DECAY

# --- DATA OUTPUT CARDS ---
straight_distance = math.sqrt(state[0]**2 + (state[1] - 0.11)**2 + state[2]**2)

col1, col2, col3, col4 = st.columns(4)
with col1:
    st.metric("Total Travel Distance", f"{straight_distance:.2f} m")
with col2:
    st.metric("Final Depth (i)", f"{state[0]:.2f} m")
with col3:
    st.metric("Final Height (j)", f"{state[1]:.2f} m")
with col4:
    st.metric("Final Width (k)", f"{state[2]:.2f} m")

# --- 3D PLOTLY RENDER ---
fig = go.Figure()

# Downsample coordinates (every 4th point) to keep the visualizer lag-free
fig.add_trace(go.Scatter3d(
    x=k_path[::4], y=i_path[::4], z=j_path[::4],
    mode='lines',
    line=dict(color='#00fff0', width=6),
    name='RK4 Trajectory'
))

# Target Goal Dimensions Configuration
gw, gh = 7.32, 2.44
goal_x_left = GOAL_CENTER_K - gw/2
goal_x_right = GOAL_CENTER_K + gw/2

fig.add_trace(go.Scatter3d(
    x=[goal_x_left, goal_x_right, goal_x_right, goal_x_left, goal_x_left],
    y=[I_DIST, I_DIST, I_DIST, I_DIST, I_DIST],
    z=[0, 0, gh, gh, 0],
    mode='lines',
    line=dict(color='#ff0055', width=8),
    name='Target Goalmouth'
))

fig.update_layout(
    template="plotly_dark",
    scene=dict(
        xaxis_title="Width (k)",
        yaxis_title="Depth (i)",
        zaxis_title="Height (j)",
        aspectmode='data',
        camera=dict(eye=dict(x=-1.2, y=-1.5, z=0.8))
    ),
    margin=dict(l=0, r=0, b=0, t=0),
    showlegend=True
)

st.plotly_chart(fig, use_container_width=True)
