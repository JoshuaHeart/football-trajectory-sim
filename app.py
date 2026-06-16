import math
import plotly.graph_objects as go
import streamlit as st

# --- STREAMLIT PAGE CONFIG ---
st.set_page_config(page_title="3D Trajectory Simulator", layout="wide")

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

st.title("⚽ 3D Trajectory Simulation")

# --- SIDEBAR INPUTS ---
st.sidebar.header("📍 Starting Position")
# Adjustable starting coordinates (2D pitch floor position)
start_depth = st.sidebar.slider("Shot Distance", 11.0, 35.0, 22.0, 0.5)
start_lateral = st.sidebar.slider("Right or left shift", -15.0, 15.0, 0.0, 0.5)

st.sidebar.header("📥 Initial Launch")
v0 = st.sidebar.slider("Velocity (m/s)", 5.0, 35.0, 24.5)
theta_deg = st.sidebar.slider("Vertical Angle (degrees)", -10.0, 50.0, 14.5)
phi_deg = st.sidebar.slider("Horizontal Angle (degrees)", -45.0, 45.0, -1.8)

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
GOAL_LINE_DEPTH = 30.0  # Fixed absolute coordinate for the goalmouth line

def get_accel(v_vec, current_spin):
    vx, vy, vz = v_vec
    v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
    if v_mag < 0.1: return 0, -G, 0
    
    # Magnus Vector calculation (Cross product of spin axis and velocity)
    wx, wy, wz = current_spin*nx, current_spin*ny, current_spin*nz
    cx = (wy * vz - wz * vy)
    cy = (wz * vx - wx * vz)
    cz = (wx * vy - wy * vx)
    
    # Calculate Drag and Lift
    Re = RHO * v_mag * LENG / MU
    Cd = 0.18 if Re > 1.5e5 else 0.41
    Cl = 0.15 # Stable Magnus coefficient for realistic ball curve
    
    # Apply acceleration
    ax = (-Cd * 0.5 * RHO * A * v_mag * vx + Cl * RHO * A * RADIUS * cx) / MASS
    ay = (-Cd * 0.5 * RHO * A * v_mag * vy + Cl * RHO * A * RADIUS * cy) / MASS - G
    az = (-Cd * 0.5 * RHO * A * v_mag * vz + Cl * RHO * A * RADIUS * cz) / MASS
    
    return ax, ay, az
# --- INITIALIZE STATE ---
tr, pr = math.radians(theta_deg), math.radians(phi_deg)

# Derive initial positions based on user input
# Depth runs from 0 at player baseline towards GOAL_LINE_DEPTH
init_x = GOAL_LINE_DEPTH - start_depth 
init_y = 0.11
init_z = start_lateral

state = [
    init_x, init_y, init_z, 
    v0 * math.cos(tr) * math.cos(pr), 
    v0 * math.sin(tr), 
    v0 * math.cos(tr) * math.sin(pr)
]
spin = spin_mag_init
x_p, y_p, z_p = [state[0]], [state[1]], [state[2]]

# --- RK4 LOOP ---
while state[1] >= 0.1 and state[0] < GOAL_LINE_DEPTH:
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

# --- GOAL DETECTOR LOGIC (7.32m x 2.44m Goal Frame at GOAL_LINE_DEPTH) ---
gw, gh, gk = 7.32, 2.44, 0.0
is_goal = False

if abs(state[0] - GOAL_LINE_DEPTH) < 0.5:
    inside_posts = (-gw/2 <= state[2] <= gw/2)
    under_crossbar = (0.11 <= state[1] <= gh)
    if inside_posts and under_crossbar:
        is_goal = True

if is_goal:
    st.markdown('<div class="goal-banner status-goal">🚀 GOALL!!!! GOALL!!! ⚽🔥</div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="goal-banner status-miss">❌ MISSED / SAVED 🧤</div>', unsafe_allow_html=True)

# --- METRICS ---
total_dist = math.sqrt((state[0]-init_x)**2 + (state[1]-init_y)**2 + (state[2]-init_z)**2)
c1, c2, c3 = st.columns(3)
c1.metric("Total Shot Trajectory Distance", f"{total_dist:.2f} m")
c2.metric("Final Landing Height", f"{state[1]:.2f} m")
c3.metric("Goal Line Cross Position (k)", f"{state[2]:.2f} m")

# --- 3D RENDERING ---
fig = go.Figure()

# 1. REALISTIC STRIPED GRASS PITCH
pitch_width = 30
for start_y in range(-5, int(GOAL_LINE_DEPTH) + 15, 4):
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

# 3. HIGH-REALISM PANEL-MAPPED SOCCER BALL (Stacked multi-marker array)
# Base white leather profile
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=16, color='#ffffff', symbol='circle', line=dict(color='#000000', width=2.5)),
    name='Soccer Ball'
))
# Hexagonal/Pentagonal black contrast panels overlay
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=11, color='#111111', symbol='diamond-open', line=dict(width=3)),
    showlegend=False
))
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=5, color='#111111', symbol='circle'),
    showlegend=False
))

# 4. 3D ALUMINUM GOALPOSTS
def add_3d_post(x0, y0, z0, x1, y1, z1, name, show=False):
    fig.add_trace(go.Scatter3d(
        x=[x0, x1], y=[y0, y1], z=[z0, z1],
        mode='lines', line=dict(color='#e5e7eb', width=12),
        name=name, showlegend=show
    ))

xl, xr, y_line = gk - gw/2, gk + gw/2, GOAL_LINE_DEPTH
# Goal Front Frame
add_3d_post(xl, y_line, 0, xl, y_line, gh, 'Goal Frame', show=True) 
add_3d_post(xr, y_line, 0, xr, y_line, gh, 'Goal Frame')          
add_3d_post(xl, y_line, gh, xr, y_line, gh, 'Goal Frame')         

# Box Net Depth Structure
net_depth = y_line + 2.0
add_3d_post(xl, net_depth, 0, xl, net_depth, gh, 'Net Support Framework')   
add_3d_post(xr, net_depth, 0, xr, net_depth, gh, 'Net Support Framework')   
add_3d_post(xl, net_depth, gh, xr, net_depth, gh, 'Net Support Framework')  
add_3d_post(xl, y_line, gh, xl, net_depth, gh, 'Net Support Framework')     
add_3d_post(xr, y_line, gh, xr, net_depth, gh, 'Net Support Framework')     
# Base Ground Bars
add_3d_post(xl, y_line, 0, xl, net_depth, 0, 'Net Support Framework')
add_3d_post(xr, y_line, 0, xr, net_depth, 0, 'Net Support Framework')

# 5. REINFORCED MESH GEOMETRY (Thicker Lines, Rear Backing & Side Nettings)
net_density_w = 14
net_density_d = 6

# Back net face + roof lines
for i in range(net_density_w + 1):
    frac = i / net_density_w
    x_curr = xl + frac * gw
    # Rear Face Vertical Strands
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[net_depth, net_depth], z=[0, gh],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.65)', width=3.0), showlegend=False, hoverinfo='skip'
    ))
    # Roof Longitudinal Strands
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[y_line, net_depth], z=[gh, gh],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.65)', width=3.0), showlegend=False, hoverinfo='skip'
    ))

# Rear Face Horizontal strands
for i in range(7):
    z_curr = (i / 6) * gh
    fig.add_trace(go.Scatter3d(
        x=[xl, xr], y=[net_depth, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.55)', width=3.0), showlegend=False, hoverinfo='skip'
    ))

# --- SIDE NETTING SECTIONS ---
for i in range(net_density_d + 1):
    frac = i / net_density_d
    y_curr = y_line + frac * 2.0
    # Left Side Wall Vertical Strands
    fig.add_trace(go.Scatter3d(
        x=[xl, xl], y=[y_curr, y_curr], z=[0, gh],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.55)', width=2.5), showlegend=False, hoverinfo='skip'
    ))
    # Right Side Wall Vertical Strands
    fig.add_trace(go.Scatter3d(
        x=[xr, xr], y=[y_curr, y_curr], z=[0, gh],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.55)', width=2.5), showlegend=False, hoverinfo='skip'
    ))

# Side Wall Horizontal structural mesh weaves
for i in range(7):
    z_curr = (i / 6) * gh
    # Left side panel sheet
    fig.add_trace(go.Scatter3d(
        x=[xl, xl], y=[y_line, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.55)', width=2.5), showlegend=False, hoverinfo='skip'
    ))
    # Right side panel sheet
    fig.add_trace(go.Scatter3d(
        x=[xr, xr], y=[y_line, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(220, 220, 220, 0.55)', width=2.5), showlegend=False, hoverinfo='skip'
    ))

# --- CAMERA CONFIGURATION ---
fig.update_layout(
    template="plotly_dark",
    scene=dict(
        xaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        zaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        aspectmode='data',
        camera=dict(
            eye=dict(x=-0.9, y=-1.5, z=0.6),
            up=dict(x=0, y=0, z=1)
        )
    ),
    legend=dict(orientation="h", yanchor="bottom", y=-0.1, xanchor="center", x=0.5),
    margin=dict(l=0, r=0, b=0, t=0),
    height=750
)

st.plotly_chart(fig, use_container_width=True)
