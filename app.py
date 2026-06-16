import math
import plotly.graph_objects as go
import streamlit as st

st.set_page_config(page_title="3D Trajectory Simulator", layout="wide")

st.markdown("""
    <style>
    .main { background-color:
    h1 { color:
    .sidebar .sidebar-content h2, .sidebar .sidebar-content h1, h3 { color:
    div[data-testid="stMetricValue"] { color:
    .goal-banner { text-align: center; font-size: 45px; font-weight: bold; padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: 'Arial Black'; }
    .status-goal { background-color:
    .status-miss { background-color:
    </style>
""", unsafe_allow_html=True)

st.title("⚽ 3D Trajectory Simulation")

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


SPIN_AXES = {
    "Pure Topspin":            (0.0,  0.0, -1.0),
    "Pure Backspin":           (0.0,  0.0,  1.0),
    "Clockwise Curler":        (0.0, -1.0,  0.0),
    "Counter-Clockwise Curler":(0.0,  1.0,  0.0),
    "No Spin":                 (0.0,  0.0,  0.0),
}


SPIN_AXES_CORRECTED = {
    "Pure Topspin":             (0.0,  0.0, -1.0),
    "Pure Backspin":            (0.0,  0.0,  1.0),
    "Clockwise Curler":         (0.0, -1.0,  0.0),
    "Counter-Clockwise Curler": (0.0,  1.0,  0.0),
    "No Spin":                  (0.0,  0.0,  0.0),
}

omega_hat = SPIN_AXES_CORRECTED[kick_style]

MASS, RADIUS, RHO, G = 0.42, 0.11, 1.225, 9.81
A = math.pi * RADIUS**2
DT = 0.004
SPIN_DECAY = 0.997
GOAL_LINE_DEPTH = 30.0

Cd = 0.30
Cl = 0.25

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

    drag_scale = -0.5 * Cd * RHO * A * v_mag / MASS
    ad_drag = drag_scale * vd
    ah_drag = drag_scale * vh
    al_drag = drag_scale * vl

    wd = spin_magnitude * omega_hat[0]
    wh = spin_magnitude * omega_hat[1]
    wl = spin_magnitude * omega_hat[2]

    cross_d = wh * vl - wl * vh
    cross_h = wl * vd - wd * vl
    cross_l = wd * vh - wh * vd

    magnus_scale = Cl * RHO * A * RADIUS / MASS
    ad_mag = magnus_scale * cross_d
    ah_mag = magnus_scale * cross_h
    al_mag = magnus_scale * cross_l

    return (
        ad_drag + ad_mag,
        ah_drag + ah_mag - G,
        al_drag + al_mag
    )

tr = math.radians(theta_deg)
pr = math.radians(phi_deg)

init_depth   = GOAL_LINE_DEPTH - start_depth
init_height  = 0.11
init_lateral = start_lateral

state = [
    init_depth,
    init_height,
    init_lateral,
    v0 * math.cos(tr) * math.cos(pr),
    v0 * math.sin(tr),
    v0 * math.cos(tr) * math.sin(pr),
]
spin = spin_mag_init

x_p = [state[0]]
y_p = [state[1]]
z_p = [state[2]]

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

total_dist = math.sqrt(
    (state[0]-init_depth)**2 + (state[1]-init_height)**2 + (state[2]-init_lateral)**2
)
c1, c2, c3 = st.columns(3)
c1.metric("Total Shot Trajectory Distance", f"{total_dist:.2f} m")
c2.metric("Final Landing Height", f"{state[1]:.2f} m")
c3.metric("Goal Line Cross Position (lateral)", f"{state[2]:.2f} m")

fig = go.Figure()

pitch_width = 30
for start_y in range(-5, int(GOAL_LINE_DEPTH) + 15, 4):
    color = '#1e4620' if (start_y // 4) % 2 == 0 else '#2d5a27'
    fig.add_trace(go.Mesh3d(
        x=[-pitch_width/2, -pitch_width/2, pitch_width/2, pitch_width/2],
        y=[start_y, start_y+4, start_y+4, start_y],
        z=[0, 0, 0, 0],
        color=color, opacity=1.0, showlegend=False, hoverinfo='skip'
    ))

fig.add_trace(go.Scatter3d(
    x=z_p, y=x_p, z=y_p,
    mode='lines', line=dict(color='#ff1100', width=8),
    name='Ball Path'
))

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
