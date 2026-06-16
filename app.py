import math
import plotly.graph_objects as go
import streamlit as st

# --- STREAMLIT PAGE CONFIG ---
st.set_page_config(page_title="Pro-Pitch Flight Engine", layout="wide")

st.markdown("""
    <style>
    .main { background-color: #ffffff; color: #000000; }
    h1 { color: #003366 !important; font-family: 'Arial Black'; }
    .sidebar .sidebar-content h2, .sidebar .sidebar-content h1, h3 { color: #003366 !important; }
    div[data-testid="stMetricValue"] { color: #003366; font-weight: bold; }
    /* Goal Alert Styling */
    .goal-banner { text-align: center; font-size: 45px; font-weight: bold; padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: 'Arial Black'; }
    .status-goal { background-color: #d4edda; color: #155724; border: 4px solid #c3e6cb; }
    .status-miss { background-color: #f8d7da; color: #721c24; border: 4px solid #f5c6cb; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ Elite Trajectory Analytics")

# --- SIDEBAR INPUTS ---
st.sidebar.header("📥 Initial Launch")
v0 = st.sidebar.slider("Velocity (m/s)", 5.0, 35.0, 24.5)
theta_deg = st.sidebar.slider("Vertical Angle (degrees)", -10.0, 50.0, 14.5)
phi_deg = st.sidebar.slider("Horizontal Angle (degrees)", -30.0, 30.0, -1.8)

st.sidebar.header("🌀 Spin Configuration")
kick_style = st.sidebar.selectbox(
    "Kick Style",
    ["Pure Topspin", "Pure Backspin", "Clockwise Curler", "Counter-Clockwise Curler", "No Spin"]
)
spin_mag_init = st.sidebar.slider("Spin Intensity (rad/s)", 0.0, 100.0, 65.0)

# Map Selection to Vectors
if "Topspin" in kick_style: nx, ny, nz = 1.0, 0.0, 0.0
elif "Backspin" in kick_style: nx, ny, nz = -1.0, 0.0, 0.0
elif "Clockwise" in kick_style: nx, ny, nz = 0.0, 0.0, 1.0
elif "Counter-Clockwise" in kick_style: nx, ny, nz = 0.0, 0.0, -1.0
else: nx, ny, nz, spin_mag_init = 0.0, 0.0, 0.0, 0.0

# --- CONSTANTS & PHYSICS (RK4) ---
MASS, RADIUS, RHO, G, MU, LENG = 0.42, 0.11, 1.225, 9.81, 1.8e-5, 0.22
A = math.pi * RADIUS**2
DT = 0.004
SPIN_DECAY = 0.997
I_DIST = 25.0  # Professional penalty spot / box setup boundary

def get_accel(v_vec, current_spin):
    vx, vy, vz = v_vec
    v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
    if v_mag < 0.1: return 0, -G, 0
    
    Re = RHO * v_mag * LENG / MU
    Cd = 0.18 if Re > 1.5e5 else 0.41
    Cl = (RADIUS * current_spin) / v_mag
    
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
        state[i] += v_curr[i]*DT
        state[i+3] += (DT/6)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
    
    x_p.append(state[0]); y_p.append(state[1]); z_p.append(state[2])
    spin *= SPIN_DECAY

# --- GOAL DETECTOR LOGIC (7.32m x 2.44m Goal Frame at I_DIST) ---
gw, gh, gk = 7.32, 2.44, 0.0
is_goal = False

if abs(state[0] - I_DIST) < 0.5:
    # Ball cross-section checks against crossbar/post boundaries
    inside_posts = (-gw/2 <= state[2] <= gw/2)
    under_crossbar = (0.11 <= state[1] <= gh)
    if inside_posts and under_crossbar:
        is_goal = True

if is_goal:
    st.markdown('<div class="goal-banner status-goal">🚀 GOALL!!!! GOALL!!! ⚽🔥</div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="goal-banner status-miss">❌ MISSED / SAVED 🧤</div>', unsafe_allow_html=True)

# --- METRICS ---
c1, c2, c3 = st.columns(3)
c1.metric("Impact Distance", f"{state[0]:.2f} m")
c2.metric("Final Height", f"{state[1]:.2f} m")
c3.metric("Lateral Deviation", f"{state[2]:.2f} m")

# --- 3D RENDERING ---
fig = go.Figure()

# 1. REALISTIC STRIPED GRASS PITCH
pitch_width = 20
for start_y in range(-5, int(I_DIST) + 10, 4):
    color = '#1e4620' if (start_y // 4) % 2 == 0 else '#2d5a27'
    fig.add_trace(go.Mesh3d(
        x=[-pitch_width/2, -pitch_width/2, pitch_width/2, pitch_width/2],
        y=[start_y, start_y+4, start_y+4, start_y],
        z=[0, 0, 0, 0],
        color=color, opacity=1.0, showlegend=False, hoverinfo='skip'
    ))

# 2. RED TRAJECTORY LINE
fig.add_trace(go.Scatter3d(
    x=z_p, y=x_p, z=y_p,
    mode='lines', line=dict(color='#ff1100', width=8),
    name='Ball Path'
))

# 3. PANELD PATTERN SOCCER BALL (Layered dual-color marker mapping)
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=14, color='white', symbol='circle', line=dict(color='#111111', width=3)),
    name='Soccer Ball'
))
# Nested inner dark core to mimic panel pattern alignment
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=7, color='#222222', symbol='x'),
    showlegend=False
))

# 4. 3D CYLINDRICAL ALUMINUM GOALPOSTS
def add_3d_post(x0, y0, z0, x1, y1, z1, name, show=False):
    fig.add_trace(go.Scatter3d(
        x=[x0, x1], y=[y0, y1], z=[z0, z1],
        mode='lines', line=dict(color='#d1d5db', width=12),
        name=name, showlegend=show
    ))

xl, xr, y_line = gk - gw/2, gk + gw/2, I_DIST
# Main Frame Posts
add_3d_post(xl, y_line, 0, xl, y_line, gh, 'Goal Frame', show=True) # Left Post
add_3d_post(xr, y_line, 0, xr, y_line, gh, 'Goal Frame')          # Right Post
add_3d_post(xl, y_line, gh, xr, y_line, gh, 'Goal Frame')         # Crossbar

# 5. BOX NET BACKING STRUCTURE (Drawn 2 meters deep)
net_depth = y_line + 2.0
add_3d_post(xl, net_depth, 0, xl, net_depth, gh, 'Net Support')   # Left Back Post
add_3d_post(xr, net_depth, 0, xr, net_depth, gh, 'Net Support')   # Right Back Post
add_3d_post(xl, net_depth, gh, xr, net_depth, gh, 'Net Support')  # Back Top Tension Bar
add_3d_post(xl, y_line, gh, xl, net_depth, gh, 'Net Support')     # Left Top Depth Bar
add_3d_post(xr, y_line, gh, xr, net_depth, gh, 'Net Support')     # Right Top Depth Bar

# 6. BOX NET MESH FABRIC (Highly detailed transparent mesh grid lines)
net_intervals = 12
for i in range(net_intervals + 1):
    frac = i / net_intervals
    x_curr = xl + frac * gw
    # Vertical back mesh grid strands
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[net_depth, net_depth], z=[0, gh],
        mode='lines', line=dict(color='rgba(200, 200, 200, 0.4)', width=1.5), showlegend=False, hoverinfo='skip'
    ))
    # Roof mesh longitudinal strands
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[y_line, net_depth], z=[gh, gh],
        mode='lines', line=dict(color='rgba(200, 200, 200, 0.4)', width=1.5), showlegend=False, hoverinfo='skip'
    ))

# Horizontal back mesh panel bars
z_intervals = 6
for i in range(z_intervals + 1):
    z_curr = (i / z_intervals) * gh
    fig.add_trace(go.Scatter3d(
        x=[xl, xr], y=[net_depth, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(200, 200, 200, 0.3)', width=1.5), showlegend=False, hoverinfo='skip'
    ))

# --- CAMERA & CONFIGURATION SETUPS ---
fig.update_layout(
    template="plotly_dark",
    scene=dict(
        xaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        zaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        aspectmode='data',
        camera=dict(
            eye=dict(x=-0.9, y=-1.4, z=0.5),   # Perfectly frames the striker's viewpoint looking down the pitch
            up=dict(x=0, y=0, z=1)
        )
    ),
    legend=dict(orientation="h", yanchor="bottom", y=-0.1, xanchor="center", x=0.5),
    margin=dict(l=0, r=0, b=0, t=0),
    height=750
)

st.plotly_chart(fig, use_container_width=True)
