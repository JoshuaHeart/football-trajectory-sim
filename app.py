import math
import plotly.graph_objects as go
import streamlit as st

# --- STREAMLIT PAGE CONFIG ---
st.set_page_config(page_title="Pro-Pitch Flight Engine", layout="wide")

# CUSTOM STYLING: Darker Blue sidebar headers and clean layout
st.markdown("""
    <style>
    .main { background-color: #ffffff; color: #000000; }
    h1 { color: #003366 !important; font-family: 'Arial'; }
    /* Darker Blue for Sidebar Headers */
    .sidebar .sidebar-content h2, .sidebar .sidebar-content h1, h3 { 
        color: #003366 !important; 
    }
    div[data-testid="stMetricValue"] { color: #003366; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ Elite Trajectory Analytics")

# --- SIDEBAR INPUTS ---
st.sidebar.header("📥 Initial Launch")
v0 = st.sidebar.slider("Velocity (m/s)", 5.0, 35.0, 24.5)
theta_deg = st.sidebar.slider("Vertical Angle (degrees)", -10.0, 50.0, 18.0)
phi_deg = st.sidebar.slider("Horizontal Angle (degrees)", -30.0, 30.0, 0.0)

st.sidebar.header("🌀 Spin Configuration")
kick_style = st.sidebar.selectbox(
    "Kick Style",
    ["Pure Topspin", "Pure Backspin", "Clockwise Curler", "Counter-Clockwise Curler", "No Spin"]
)
spin_mag_init = st.sidebar.slider("Spin Intensity (rad/s)", 0.0, 100.0, 55.0)

# Map Selection to Vectors
if "Topspin" in kick_style: nx, ny, nz = 1.0, 0.0, 0.0
elif "Backspin" in kick_style: nx, ny, nz = -1.0, 0.0, 0.0
elif "Clockwise" in kick_style: nx, ny, nz = 0.0, 0.0, 1.0
elif "Counter-Clockwise" in kick_style: nx, ny, nz = 0.0, 0.0, -1.0
else: nx, ny, nz, spin_mag_init = 0.0, 0.0, 0.0, 0.0

# --- CONSTANTS & PHYSICS (RK4) ---
MASS, RADIUS, RHO, G, MU, LENG = 0.42, 0.11, 1.225, 9.81, 1.8e-5, 0.22
A = math.pi * RADIUS**2
DT = 0.005
SPIN_DECAY = 0.998
I_DIST = 30.0 # Goal Line distance

def get_accel(v_vec, current_spin):
    vx, vy, vz = v_vec
    v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
    if v_mag < 0.1: return 0, -G, 0
    
    # Aerodynamics
    Re = RHO * v_mag * LENG / MU
    Cd = 0.2 if Re > 1.5e5 else 0.4
    Cl = (RADIUS * current_spin) / v_mag
    
    # Magnus via Cross Product (w x v)
    wx, wy, wz = current_spin*nx, current_spin*ny, current_spin*nz
    cx, cy, cz = (wy*vz - wz*vy), (wz*vx - wx*vz), (wx*vy - wy*vx)
    
    S = 0.5 * RHO * A
    ax = (-Cd * S * v_mag * vx + Cl * S * RADIUS * cx) / MASS
    ay = (-Cd * S * v_mag * vy + Cl * S * RADIUS * cy) / MASS - G
    az = (-Cd * S * v_mag * vz + Cl * S * RADIUS * cz) / MASS
    return ax, ay, az

# --- INITIALIZE STATE ---
tr, pr = math.radians(theta_deg), math.radians(phi_deg)
state = [0.0, 0.11, 0.0, v0*math.cos(tr)*math.cos(pr), v0*math.sin(tr), v0*math.cos(tr)*math.sin(pr)]
spin = spin_mag_init
x_p, y_p, z_p = [state[0]], [state[1]], [state[2]]

# --- RK4 LOOP ---
while state[1] >= 0.1 and state[0] < I_DIST:
    v_curr = state[3:6]
    k1 = get_accel(v_curr, spin)
    k2 = get_accel([v_curr[i] + 0.5*DT*k1[i] for i in range(3)], spin)
    k3 = get_accel([v_curr[i] + 0.5*DT*k2[i] for i in range(3)], spin)
    k4 = get_accel([v_curr[i] + DT*k3[i] for i in range(3)], spin)
    
    for i in range(3):
        # Update Pos and Vel
        state[i] += v_curr[i]*DT
        state[i+3] += (DT/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
    
    x_p.append(state[0]); y_p.append(state[1]); z_p.append(state[2])
    spin *= SPIN_DECAY

# --- METRICS ---
c1, c2, c3 = st.columns(3)
c1.metric("Impact Distance", f"{state[0]:.2f} m")
c2.metric("Final Height", f"{state[1]:.2f} m")
c3.metric("Lateral Deviation", f"{state[2]:.2f} m")

# --- 3D RENDERING ---
fig = go.Figure()

# 1. THE GRASS PITCH (FLOOR)
fig.add_trace(go.Mesh3d(
    x=[-10, -10, I_DIST+5, I_DIST+5], # Lateral
    y=[0, I_DIST+5, I_DIST+5, 0],    # Depth
    z=[0, 0, 0, 0],                  # Height
    color='#2d5a27', opacity=1.0, name='Pitch'
))

# 2. RED TRAJECTORY LINE
fig.add_trace(go.Scatter3d(
    x=z_p, y=x_p, z=y_p,
    mode='lines', line=dict(color='red', width=7),
    name='Ball Path'
))

# 3. THE FOOTBALL (Marker at end)
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=10, color='white', symbol='circle', line=dict(color='black', width=2)),
    name='Football Position'
))

# 4. GREY GOALPOSTS
gw, gh, gk = 7.32, 2.44, -1.2
fig.add_trace(go.Scatter3d(
    x=[gk-gw/2, gk-gw/2, gk+gw/2, gk+gw/2],
    y=[I_DIST, I_DIST, I_DIST, I_DIST],
    z=[0, gh, gh, 0],
    mode='lines', line=dict(color='#808080', width=10),
    name='Goal Frame'
))

# LAYOUT CLEANUP: Remove "Black Stuff" and Grid
fig.update_layout(
    scene=dict(
        xaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        zaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        aspectmode='data'
    ),
    legend=dict(
        orientation="h",
        yanchor="bottom",
        y=-0.1,
        xanchor="center",
        x=0.5
    ),
    margin=dict(l=0, r=0, b=0, t=0),
    height=700
)

st.plotly_chart(fig, use_container_width=True)
