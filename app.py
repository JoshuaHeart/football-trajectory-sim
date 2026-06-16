import math
import plotly.graph_objects as go
import streamlit as st
from streamlit_plotly_events import plotly_events

# --- STREAMLIT PAGE CONFIG ---
st.set_page_config(page_title="Pro-Pitch Flight Engine", layout="wide")

st.markdown("""
    <style>
    .main { background-color: #ffffff; color: #000000; }
    h1 { color: #003366 !important; font-family: 'Arial Black'; }
    .sidebar .sidebar-content h2, .sidebar .sidebar-content h1, h3 { color: #003366 !important; }
    div[data-testid="stMetricValue"] { color: #003366; font-weight: bold; }
    .goal-banner { text-align: center; font-size: 45px; font-weight: bold; padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: 'Arial Black'; }
    .status-goal { background-color: #d4edda; color: #155724; border: 4px solid #c3e6cb; }
    .status-miss { background-color: #f8d7da; color: #721c24; border: 4px solid #f5c6cb; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ Elite Trajectory Analytics")

# --- SIDEBAR INTERACTIVE MAP ZONE ---
st.sidebar.header("📍 Shot Position Setup")
st.sidebar.write("Click anywhere on the pitch below to place the ball:")

# Generate the Green 2D Selector Map Layout
fig_map = go.Figure()

# Pitch Field Surface (Green Field Base)
fig_map.add_trace(go.Scaleanchor=dict(), x=[-16, 16, 16, -16, -16], y=[-2, -2, 32, 32, -2],
                  fill="toself", fillcolor="#1e4620", opacity=1.0, mode="none", showlegend=False, hoverinfo='skip')

# Pitch White Outline Boundary (30m depth x 30m width matrix)
fig_map.add_trace(go.Scatter(
    x=[-15, 15, 15, -15, -15], y=[0, 0, 30, 30, 0],
    mode='lines', line=dict(color='#ffffff', width=2), showlegend=False, hoverinfo='skip'
))
# Goal Outline represented at the top baseline (y=30)
fig_map.add_trace(go.Scatter(
    x=[-3.66, 3.66], y=[30, 30],
    mode='lines', line=dict(color='#ffffff', width=6), name='Goal Line Outlines', showlegend=False
))
# Goal Depth Net Visual Box (Drawn behind the line)
fig_map.add_trace(go.Scatter(
    x=[-3.66, -3.66, 3.66, 3.66], y=[30, 31.5, 31.5, 30],
    mode='lines', line=dict(color='rgba(255,255,255,0.4)', width=1.5, dash='dash'), showlegend=False, hoverinfo='skip'
))
# Penalty Spot Marker (11 meters out from y=30)
fig_map.add_trace(go.Scatter(
    x=[0], y=[19], mode='markers', marker=dict(color='#ffffff', size=6), showlegend=False, hoverinfo='skip'
))

# Initialize state-safe defaults for selections
if 'click_x' not in st.session_state: st.session_state.click_x = 0.0
if 'click_y' not in st.session_state: st.session_state.click_y = 11.0 # default 11m penalty distance

# Render current point selection overlay on 2D tactical map
fig_map.add_trace(go.Scatter(
    x=[st.session_state.click_x], y=[30.0 - st.session_state.click_y],
    mode='markers', marker=dict(color='#ff1100', size=12, symbol='x', line=dict(color='white', width=1)), name='Ball Placement', showlegend=False
))

fig_map.update_layout(
    template="plotly_dark",
    xaxis=dict(range=[-17, 17], showgrid=False, zeroline=False, visible=False),
    yaxis=dict(range=[-3, 33], showgrid=False, zeroline=False, visible=False),
    width=260, height=260, margin=dict(l=0, r=0, b=0, t=0),
    clickmode='event+select'
)

# Capture Click coordinate returns directly inside the sidebar container
with st.sidebar:
    selected_point = plotly_events(fig_map, click_event=True, hover_event=False, override_height=265)

if selected_point:
    st.session_state.click_x = round(selected_point[0]['x'], 1)
    st.session_state.click_y = round(30.0 - selected_point[0]['y'], 1)

# Display real-time readouts underneath map graphic selector
st.sidebar.markdown(f"**Coordinates:** Distance: `{st.session_state.click_y}m` | Lateral: `{st.session_state.click_x}m`")

# --- FLIGHT CONTROLS ---
st.sidebar.header("📥 Launch Profile")
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
GOAL_LINE_DEPTH = 30.0

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

# --- INITIALIZE STATE ENGINE ---
tr, pr = math.radians(theta_deg), math.radians(phi_deg)
init_x = GOAL_LINE_DEPTH - st.session_state.click_y
init_y = 0.11
init_z = st.session_state.click_x

state = [
    init_x, init_y, init_z, 
    v0 * math.cos(tr) * math.cos(pr), 
    v0 * math.sin(tr), 
    v0 * math.cos(tr) * math.sin(pr)
]
spin = spin_mag_init
x_p, y_p, z_p = [state[0]], [state[1]], [state[2]]

# --- RK4 SIMULATION LOOP ---
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

# --- GOAL DETECTOR CHECK ---
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

# --- DISPLAY METRICS ---
total_dist = math.sqrt((state[0]-init_x)**2 + (state[1]-init_y)**2 + (state[2]-init_z)**2)
c1, c2, c3 = st.columns(3)
c1.metric("Trajectory Length", f"{total_dist:.2f} m")
c2.metric("Final Flight Height", f"{state[1]:.2f} m")
c3.metric("Goal Line Cross Deviation", f"{state[2]:.2f} m")

# --- MAIN 3D RENDER ENGINE ---
fig = go.Figure()

# 1. STRIPED TURF GRASS CORES
pitch_width = 32
for start_y in range(-5, int(GOAL_LINE_DEPTH) + 15, 4):
    color = '#1e4620' if (start_y // 4) % 2 == 0 else '#2d5a27'
    fig.add_trace(go.Mesh3d(
        x=[-pitch_width/2, -pitch_width/2, pitch_width/2, pitch_width/2],
        y=[start_y, start_y+4, start_y+4, start_y],
        z=[0, 0, 0, 0],
        color=color, opacity=1.0, showlegend=False, hoverinfo='skip'
    ))

# 2. FLIGHT PATH TRACE
fig.add_trace(go.Scatter3d(
    x=z_p, y=x_p, z=y_p,
    mode='lines', line=dict(color='#ff1100', width=8), name='Ball Flight Line'
))

# 3. FIXED HIGH-REALISM PANEL-MAPPED SOCCER BALL
bx, by, bz = z_p[-1], x_p[-1], y_p[-1]

# Base white leather sphere shape
fig.add_trace(go.Scatter3d(
    x=[bx], y=[by], z=[bz], mode='markers',
    marker=dict(size=18, color='#ffffff', symbol='circle', line=dict(color='#111111', width=1.5)),
    name='Soccer Ball'
))
# Alternating Pentagonal Panel Layer (Using valid 3D shapes to prevent crashing)
fig.add_trace(go.Scatter3d(
    x=[bx, bx, bx, bx, bx], 
    y=[by, by, by, by, by], 
    z=[bz+0.05, bz-0.05, bz, bz+0.02, bz-0.02],
    mode='markers',
    marker=dict(size=7, color='#111111', symbol='diamond'),
    showlegend=False
))
fig.add_trace(go.Scatter3d(
    x=[bx, bx], y=[by+0.03, by-0.03], z=[bz, bz],
    mode='markers',
    marker=dict(size=6, color='#111111', symbol='square'),
    showlegend=False
))

# 4. ALUMINUM STYLED GOALPOST RIGGING
def add_3d_post(x0, y0, z0, x1, y1, z1, name, show=False):
    fig.add_trace(go.Scatter3d(
        x=[x0, x1], y=[y0, y1], z=[z0, z1],
        mode='lines', line=dict(color='#f3f4f6', width=11), name=name, showlegend=show
    ))

xl, xr, y_line = gk - gw/2, gk + gw/2, GOAL_LINE_DEPTH
add_3d_post(xl, y_line, 0, xl, y_line, gh, 'Goal Frame', show=True) 
add_3d_post(xr, y_line, 0, xr, y_line, gh, 'Goal Frame')          
add_3d_post(xl, y_line, gh, xr, y_line, gh, 'Goal Frame')         

# Deep Box framework anchors
net_depth = y_line + 2.0
add_3d_post(xl, net_depth, 0, xl, net_depth, gh, 'Net Support Framework')   
add_3d_post(xr, net_depth, 0, xr, net_depth, gh, 'Net Support Framework')   
add_3d_post(xl, net_depth, gh, xr, net_depth, gh, 'Net Support Framework')  
add_3d_post(xl, y_line, gh, xl, net_depth, gh, 'Net Support Framework')     
add_3d_post(xr, y_line, gh, xr, net_depth, gh, 'Net Support Framework')     
add_3d_post(xl, y_line, 0, xl, net_depth, 0, 'Net Support Framework')
add_3d_post(xr, y_line, 0, xr, net_depth, 0, 'Net Support Framework')

# 5. INDUSTRIAL STRENGTH HIGH-DENSITY NET FABRIC
net_density_w = 16
net_density_d = 7

# Rear wall meshes and roof strands
for i in range(net_density_w + 1):
    frac = i / net_density_w
    x_curr = xl + frac * gw
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[net_depth, net_depth], z=[0, gh],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.85)', width=3.5), showlegend=False, hoverinfo='skip'
    ))
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[y_line, net_depth], z=[gh, gh],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.85)', width=3.5), showlegend=False, hoverinfo='skip'
    ))

# Rear horizontal cross strings
for i in range(7):
    z_curr = (i / 6) * gh
    fig.add_trace(go.Scatter3d(
        x=[xl, xr], y=[net_depth, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.75)', width=3.5), showlegend=False, hoverinfo='skip'
    ))

# Reinforced side structural netting panels
for i in range(net_density_d + 1):
    frac = i / net_density_d
    y_curr = y_line + frac * 2.0
    fig.add_trace(go.Scatter3d(
        x=[xl, xl], y=[y_curr, y_curr], z=[0, gh],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.75)', width=3.0), showlegend=False, hoverinfo='skip'
    ))
    fig.add_trace(go.Scatter3d(
        x=[xr, xr], y=[y_curr, y_curr], z=[0, gh],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.75)', width=3.0), showlegend=False, hoverinfo='skip'
    ))

for i in range(7):
    z_curr = (i / 6) * gh
    fig.add_trace(go.Scatter3d(
        x=[xl, xl], y=[y_line, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.75)', width=3.0), showlegend=False, hoverinfo='skip'
    ))
    fig.add_trace(go.Scatter3d(
        x=[xr, xr], y=[y_line, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(243, 244, 246, 0.75)', width=3.0), showlegend=False, hoverinfo='skip'
    ))

# --- CAMERA PERSPECTIVE SETUP ---
fig.update_layout(
    template="plotly_dark",
    scene=dict(
        xaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        yaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        zaxis=dict(showgrid=False, showbackground=False, zeroline=False, visible=False),
        aspectmode='data',
        camera=dict(
            eye=dict(x=-0.8, y=-1.4, z=0.55),
            up=dict(x=0, y=0, z=1)
        )
    ),
    legend=dict(orientation="h", yanchor="bottom", y=-0.1, xanchor="center", x=0.5),
    margin=dict(l=0, r=0, b=0, t=0),
    height=750
)

st.plotly_chart(fig, use_container_width=True)
