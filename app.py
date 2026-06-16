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
    .goal-banner { text-align: center; font-size: 45px; font-weight: bold; padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: 'Arial Black'; }
    .status-goal { background-color: #d4edda; color: #155724; border: 4px solid #c3e6cb; }
    .status-miss { background-color: #f8d7da; color: #721c24; border: 4px solid #f5c6cb; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ 3D Trajectory Simulation")

# --- SIDEBAR INPUTS ---
st.sidebar.header("📍 Starting Position")
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

# --- COORDINATE SYSTEM ---
# X = lateral (left is negative, right is positive from kicker's perspective)
# Y = depth (increases toward goal, from init_x toward GOAL_LINE_DEPTH)
# Z = height (up is positive)
#
# Spin axis vectors (ω̂) — right-hand rule, Magnus force = ω × v
#
# TOPSPIN: ball spins like a wheel rolling forward.
#   Spin axis points in +X (to the right). 
#   ω × v ≈ (+X) × (+Y·vy) = +vy · (X×Y) = +vy · Z  → wait, let's be careful:
#   Actually ω × v where ω=(1,0,0), v≈(0,vy,0):
#   (1,0,0)×(0,vy,0) = (0·0−0·vy, 0·0−1·0, 1·vy−0·0) = (0, 0, vy)
#   That gives +Z (upward)... but topspin should push DOWN.
#   So for topspin (ball dips), spin axis = -X:
#   (-1,0,0)×(0,vy,0) = (0, 0, -vy) → -Z (downward) ✓
#
# BACKSPIN: spin axis = +X → force in +Z (upward) ✓
#
# CLOCKWISE CURL (ball curves to kicker's right, i.e. +X direction):
#   We need the Magnus force to have a +X lateral component.
#   ω × v where v≈(0,vy,0): need result to have +X.
#   (0,0,ω_z)×(0,vy,0) = (0·0−ω_z·vy, ω_z·0−0·0, 0·vy−0·0) = (-ω_z·vy, 0, 0)
#   For +X result: -ω_z·vy > 0 → ω_z < 0 → spin axis = -Z
#   So clockwise (from above) = spin axis -Z ✓
#
# COUNTER-CLOCKWISE: spin axis = +Z → force in +X... 
#   (0,0,+1)×(0,vy,0) = (-vy, 0, 0) → -X (curves left) ✓

SPIN_AXES = {
    "Pure Topspin":            (0.0,  0.0, -1.0),   # -X: ball dips down
    "Pure Backspin":           (0.0,  0.0,  1.0),   # +X: ball floats up
    "Clockwise Curler":        (0.0, -1.0,  0.0),   # -Z: curves right (+X)
    "Counter-Clockwise Curler":(0.0,  1.0,  0.0),   # +Z: curves left  (-X)
    "No Spin":                 (0.0,  0.0,  0.0),
}

# IMPORTANT: your plotting maps (z_p → scene-x, x_p → scene-y, y_p → scene-z)
# and your state vector is: state[0]=x(lateral), state[1]=z(height), state[2]=y(depth)
# Wait — let's re-read the original carefully:
# state = [init_x, init_y, init_z, vx, vy, vz]
# init_x = GOAL_LINE_DEPTH - start_depth  ← this is a depth value
# init_z = start_lateral                  ← lateral
# init_y = 0.11                           ← height (starts just above ground)
# vx = v0*cos(theta)*cos(phi)             ← mainly the forward velocity component
# vy = v0*sin(theta)                      ← upward
# vz = v0*cos(theta)*sin(phi)             ← lateral
#
# So in the original code:
# state[0] = depth (x in the sense of "distance downfield")
# state[1] = height
# state[2] = lateral
# And the loop runs while state[0] < GOAL_LINE_DEPTH (ball moving toward goal)
#
# Plotting: x=z_p (lateral), y=x_p (depth), z=y_p (height) ← scene coords
#
# Corrected spin axes using ORIGINAL state convention:
# state[0]=depth, state[1]=height, state[2]=lateral
# velocity: (vdepth, vheight, vlateral) = (state[3], state[4], state[5])
#
# For TOPSPIN (ball dips): need force component in -height direction
#   ω × v, where v ≈ (vd, 0, 0) forward motion dominant
#   We want result ≈ (0, -1, 0) in (depth, height, lateral)
#   Try ω = (0, 0, +1) in lateral:
#   (0,0,1)×(vd,0,0) = (0·0−1·0, 1·vd−0·0, 0·0−0·vd) = (0, vd, 0) → +height (backspin!)
#   Try ω = (0, 0, -1):
#   (0,0,-1)×(vd,0,0) = (0, -vd, 0) → -height ✓ topspin dips down
#
# For BACKSPIN: ω = (0, 0, +1) → force +height ✓
#
# For CLOCKWISE CURL (curves to kicker's left in lateral +z direction... 
#   actually let's define: clockwise when viewed from above = ball curves in +lateral):
#   Want force ≈ (0, 0, +1) in (depth, height, lateral)
#   ω × v where v≈(vd,0,0): need result (0,0,+1)·vd
#   Try ω = (0, +1, 0) in height direction:
#   (0,1,0)×(vd,0,0) = (1·0−0·0, 0·vd−0·0, 0·0−1·vd) = (0, 0, -vd) → -lateral
#   Try ω = (0, -1, 0):
#   (0,-1,0)×(vd,0,0) = (0, 0, +vd) → +lateral ✓ clockwise curls to +z side

# Re-define spin axes in original state coordinate order (depth, height, lateral):
SPIN_AXES_CORRECTED = {
    "Pure Topspin":             (0.0,  0.0, -1.0),   # → force in -height (dips)
    "Pure Backspin":            (0.0,  0.0,  1.0),   # → force in +height (floats)
    "Clockwise Curler":         (0.0, -1.0,  0.0),   # → force in +lateral (curves right)
    "Counter-Clockwise Curler": (0.0,  1.0,  0.0),   # → force in -lateral (curves left)
    "No Spin":                  (0.0,  0.0,  0.0),
}

omega_hat = SPIN_AXES_CORRECTED[kick_style]

# --- CONSTANTS & PHYSICS ---
MASS, RADIUS, RHO, G = 0.42, 0.11, 1.225, 9.81
A = math.pi * RADIUS**2
DT = 0.004
SPIN_DECAY = 0.997
GOAL_LINE_DEPTH = 30.0

Cd = 0.30   # Drag coefficient
Cl = 0.25   # Magnus lift coefficient (slightly increased — original 0.15 was weak)

def get_accel(v_vec, spin_magnitude):
    """
    Compute acceleration using drag + Magnus force.
    
    v_vec: (v_depth, v_height, v_lateral)
    spin_magnitude: scalar ω (rad/s), direction given by omega_hat
    
    Drag:   F_drag = -0.5 * Cd * rho * A * |v| * v
    Magnus: F_mag  = Cl * rho * A * R * (ω × v)
              where ω = spin_magnitude * omega_hat
    
    Cross product ω × v:
      ω = (ωd, ωh, ωl) = spin_magnitude * omega_hat
      v = (vd, vh, vl)
      (ω × v) = ( ωh*vl - ωl*vh,
                  ωl*vd - ωd*vl,
                  ωd*vh - ωh*vd )
    """
    vd, vh, vl = v_vec
    v_mag = math.sqrt(vd**2 + vh**2 + vl**2)
    if v_mag < 0.1:
        return (0.0, -G, 0.0)

    # Drag acceleration components
    drag_scale = -0.5 * Cd * RHO * A * v_mag / MASS
    ad_drag = drag_scale * vd
    ah_drag = drag_scale * vh
    al_drag = drag_scale * vl

    # Magnus force: F = Cl * rho * A * R * (ω × v)
    wd = spin_magnitude * omega_hat[0]
    wh = spin_magnitude * omega_hat[1]
    wl = spin_magnitude * omega_hat[2]

    # Full cross product ω × v
    cross_d = wh * vl - wl * vh
    cross_h = wl * vd - wd * vl
    cross_l = wd * vh - wh * vd

    magnus_scale = Cl * RHO * A * RADIUS / MASS
    ad_mag = magnus_scale * cross_d
    ah_mag = magnus_scale * cross_h
    al_mag = magnus_scale * cross_l

    # Total: drag + Magnus + gravity (only on height component)
    return (
        ad_drag + ad_mag,           # depth acceleration
        ah_drag + ah_mag - G,       # height acceleration (gravity here)
        al_drag + al_mag            # lateral acceleration
    )

# --- INITIALIZE STATE ---
tr = math.radians(theta_deg)
pr = math.radians(phi_deg)

# state = [depth, height, lateral, v_depth, v_height, v_lateral]
init_depth   = GOAL_LINE_DEPTH - start_depth
init_height  = 0.11
init_lateral = start_lateral

state = [
    init_depth,
    init_height,
    init_lateral,
    v0 * math.cos(tr) * math.cos(pr),   # v_depth (toward goal)
    v0 * math.sin(tr),                   # v_height (upward)
    v0 * math.cos(tr) * math.sin(pr),    # v_lateral
]
spin = spin_mag_init

# Store as x_p=depth, y_p=height, z_p=lateral to match original plot mapping
x_p = [state[0]]
y_p = [state[1]]
z_p = [state[2]]

# --- RK4 INTEGRATION LOOP ---
MAX_STEPS = 20000
steps = 0
while state[1] >= 0.05 and state[0] < GOAL_LINE_DEPTH and steps < MAX_STEPS:
    v_curr = state[3:6]

    k1 = get_accel(v_curr, spin)
    k2 = get_accel([v_curr[i] + 0.5*DT*k1[i] for i in range(3)], spin)
    k3 = get_accel([v_curr[i] + 0.5*DT*k2[i] for i in range(3)], spin)
    k4 = get_accel([v_curr[i] + DT*k3[i] for i in range(3)], spin)

    for i in range(3):
        state[i]   += v_curr[i] * DT
        state[i+3] += (DT / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])

    x_p.append(state[0])
    y_p.append(state[1])
    z_p.append(state[2])
    spin *= SPIN_DECAY
    steps += 1

# --- GOAL DETECTION ---
gw, gh = 7.32, 2.44
is_goal = False

if abs(state[0] - GOAL_LINE_DEPTH) < 0.5:
    inside_posts   = (-gw/2 <= state[2] <= gw/2)
    under_crossbar = (0.11 <= state[1] <= gh)
    if inside_posts and under_crossbar:
        is_goal = True

if is_goal:
    st.markdown('<div class="goal-banner status-goal">🚀 GOALL!!!! GOALL!!! ⚽🔥</div>', unsafe_allow_html=True)
else:
    st.markdown('<div class="goal-banner status-miss">❌ MISSED / SAVED 🧤</div>', unsafe_allow_html=True)

# --- METRICS ---
total_dist = math.sqrt(
    (state[0]-init_depth)**2 + (state[1]-init_height)**2 + (state[2]-init_lateral)**2
)
c1, c2, c3 = st.columns(3)
c1.metric("Total Shot Trajectory Distance", f"{total_dist:.2f} m")
c2.metric("Final Landing Height", f"{state[1]:.2f} m")
c3.metric("Goal Line Cross Position (lateral)", f"{state[2]:.2f} m")

# --- 3D RENDERING ---
# Plot mapping (same as original): scene-x=z_p(lateral), scene-y=x_p(depth), scene-z=y_p(height)
fig = go.Figure()

# 1. STRIPED GRASS PITCH
pitch_width = 30
for start_y in range(-5, int(GOAL_LINE_DEPTH) + 15, 4):
    color = '#1e4620' if (start_y // 4) % 2 == 0 else '#2d5a27'
    fig.add_trace(go.Mesh3d(
        x=[-pitch_width/2, -pitch_width/2, pitch_width/2, pitch_width/2],
        y=[start_y, start_y+4, start_y+4, start_y],
        z=[0, 0, 0, 0],
        color=color, opacity=1.0, showlegend=False, hoverinfo='skip'
    ))

# 2. TRAJECTORY LINE
fig.add_trace(go.Scatter3d(
    x=z_p, y=x_p, z=y_p,
    mode='lines', line=dict(color='#ff1100', width=8),
    name='Ball Path'
))

# 3. SOCCER BALL (at final position)
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=16, color='#ffffff', symbol='circle',
                line=dict(color='#000000', width=2.5)),
    name='Soccer Ball'
))
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=11, color='#111111', symbol='diamond-open',
                line=dict(width=3)),
    showlegend=False
))
fig.add_trace(go.Scatter3d(
    x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]],
    mode='markers',
    marker=dict(size=5, color='#111111', symbol='circle'),
    showlegend=False
))

# 4. GOALPOSTS
def add_3d_post(x0, y0, z0, x1, y1, z1, name, show=False):
    fig.add_trace(go.Scatter3d(
        x=[x0, x1], y=[y0, y1], z=[z0, z1],
        mode='lines', line=dict(color='#e5e7eb', width=12),
        name=name, showlegend=show
    ))

xl = -gw/2
xr =  gw/2
y_line = GOAL_LINE_DEPTH

add_3d_post(xl, y_line, 0, xl, y_line, gh, 'Goal Frame', show=True)
add_3d_post(xr, y_line, 0, xr, y_line, gh, 'Goal Frame')
add_3d_post(xl, y_line, gh, xr, y_line, gh, 'Goal Frame')

net_depth = y_line + 2.0
add_3d_post(xl, net_depth, 0,  xl, net_depth, gh,  'Net Support Framework')
add_3d_post(xr, net_depth, 0,  xr, net_depth, gh,  'Net Support Framework')
add_3d_post(xl, net_depth, gh, xr, net_depth, gh,  'Net Support Framework')
add_3d_post(xl, y_line,    gh, xl, net_depth,  gh, 'Net Support Framework')
add_3d_post(xr, y_line,    gh, xr, net_depth,  gh, 'Net Support Framework')
add_3d_post(xl, y_line,    0,  xl, net_depth,  0,  'Net Support Framework')
add_3d_post(xr, y_line,    0,  xr, net_depth,  0,  'Net Support Framework')

# 5. NET MESH
net_density_w = 14
net_density_d = 6

for i in range(net_density_w + 1):
    frac = i / net_density_w
    x_curr = xl + frac * gw
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[net_depth, net_depth], z=[0, gh],
        mode='lines', line=dict(color='rgba(220,220,220,0.65)', width=3.0),
        showlegend=False, hoverinfo='skip'
    ))
    fig.add_trace(go.Scatter3d(
        x=[x_curr, x_curr], y=[y_line, net_depth], z=[gh, gh],
        mode='lines', line=dict(color='rgba(220,220,220,0.65)', width=3.0),
        showlegend=False, hoverinfo='skip'
    ))

for i in range(7):
    z_curr = (i / 6) * gh
    fig.add_trace(go.Scatter3d(
        x=[xl, xr], y=[net_depth, net_depth], z=[z_curr, z_curr],
        mode='lines', line=dict(color='rgba(220,220,220,0.55)', width=3.0),
        showlegend=False, hoverinfo='skip'
    ))

for i in range(net_density_d + 1):
    frac = i / net_density_d
    y_curr = y_line + frac * 2.0
    for x_side in [xl, xr]:
        fig.add_trace(go.Scatter3d(
            x=[x_side, x_side], y=[y_curr, y_curr], z=[0, gh],
            mode='lines', line=dict(color='rgba(220,220,220,0.55)', width=2.5),
            showlegend=False, hoverinfo='skip'
        ))

for i in range(7):
    z_curr = (i / 6) * gh
    for x_side in [xl, xr]:
        fig.add_trace(go.Scatter3d(
            x=[x_side, x_side], y=[y_line, net_depth], z=[z_curr, z_curr],
            mode='lines', line=dict(color='rgba(220,220,220,0.55)', width=2.5),
            showlegend=False, hoverinfo='skip'
        ))

# --- LAYOUT ---
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
