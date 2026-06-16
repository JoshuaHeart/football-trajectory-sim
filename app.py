import math
import plotly.graph_objects as go
import streamlit as st
from streamlit_plotly_events import plotly_events

# --- CONFIG ---
st.set_page_config(page_title="Pro-Pitch Flight Engine", layout="wide")

# --- UI STYLING ---
st.markdown("""
    <style>
    .goal-banner { text-align: center; font-size: 40px; font-weight: bold; padding: 15px; border-radius: 10px; margin-bottom: 20px; font-family: 'Arial Black'; }
    .status-goal { background-color: #d4edda; color: #155724; border: 4px solid #c3e6cb; }
    .status-miss { background-color: #f8d7da; color: #721c24; border: 4px solid #f5c6cb; }
    </style>
""", unsafe_allow_html=True)

st.title("⚽ Elite Trajectory Analytics")

# --- SIDEBAR: TACTICAL MAP ---
st.sidebar.header("📍 Tactical Position")
fig_map = go.Figure()
fig_map.add_trace(go.Scatter(x=[-16, 16, 16, -16, -16], y=[-2, -2, 32, 32, -2], fill="toself", fillcolor="#1e4620", line=dict(color='white')))
fig_map.add_trace(go.Scatter(x=[-3.66, 3.66], y=[30, 30], mode='lines', line=dict(color='white', width=5)))

if 'click_x' not in st.session_state: st.session_state.click_x = 0.0
if 'click_y' not in st.session_state: st.session_state.click_y = 11.0

fig_map.add_trace(go.Scatter(x=[st.session_state.click_x], y=[30.0 - st.session_state.click_y], mode='markers', marker=dict(color='red', size=12, symbol='x')))
fig_map.update_layout(template="plotly_dark", xaxis=dict(visible=False), yaxis=dict(visible=False), width=250, height=250, margin=dict(l=0, r=0, b=0, t=0))

with st.sidebar:
    pts = plotly_events(fig_map, click_event=True)
    if pts:
        st.session_state.click_x = round(pts[0]['x'], 1)
        st.session_state.click_y = round(30.0 - pts[0]['y'], 1)

# --- PHYSICS CONTROLS ---
v0 = st.sidebar.slider("Velocity (m/s)", 5.0, 35.0, 24.5)
theta_deg = st.sidebar.slider("Vertical Angle", -10.0, 50.0, 14.5)
phi_deg = st.sidebar.slider("Horizontal Angle", -45.0, 45.0, -1.8)
spin_mag = st.sidebar.slider("Spin Intensity", 0.0, 100.0, 65.0)

# --- PHYSICS ENGINE (RK4) ---
MASS, RADIUS, RHO, G, MU, LENG = 0.42, 0.11, 1.225, 9.81, 1.8e-5, 0.22
A = math.pi * RADIUS**2
DT = 0.004
GOAL_LINE = 30.0

tr, pr = math.radians(theta_deg), math.radians(phi_deg)
state = [GOAL_LINE - st.session_state.click_y, 0.11, st.session_state.click_x, 
         v0*math.cos(tr)*math.cos(pr), v0*math.sin(tr), v0*math.cos(tr)*math.sin(pr)]
x_p, y_p, z_p = [state[0]], [state[1]], [state[2]]

while state[1] >= 0.1 and state[0] < GOAL_LINE:
    vx, vy, vz = state[3:6]
    v_mag = math.sqrt(vx**2 + vy**2 + vz**2)
    Re = RHO * v_mag * LENG / MU
    Cd = 0.18 if Re > 1.5e5 else 0.41
    ax = (-Cd * 0.5 * RHO * A * v_mag * vx) / MASS
    ay = (-Cd * 0.5 * RHO * A * v_mag * vy) / MASS - G
    az = (-Cd * 0.5 * RHO * A * v_mag * vz) / MASS
    for i in range(3):
        state[i] += state[i+3]*DT
        state[i+3] += [ax, ay, az][i]*DT
    x_p.append(state[0]); y_p.append(state[1]); z_p.append(state[2])

# --- 3D RENDERING ARCHITECTURE ---
fig = go.Figure()

# 1. Pitch Tiling
for i in range(0, 32, 4):
    fig.add_trace(go.Mesh3d(x=[-15, -15, 15, 15], y=[i, i+4, i+4, i], z=[0,0,0,0], color='#2d5a27'))

# 2. FULL NET STRUCTURE (Restored)
xl, xr, y_line = -3.66, 3.66, GOAL_LINE
for i in range(0, 8): # Vertical Strands (Thickened)
    x_c = xl + (i/7)*7.32
    fig.add_trace(go.Scatter3d(x=[x_c, x_c], y=[y_line, y_line+2], z=[0, 2.44], mode='lines', line=dict(color='white', width=4)))
for i in range(0, 4): # Horizontal Strands
    z_c = (i/3)*2.44
    fig.add_trace(go.Scatter3d(x=[xl, xr], y=[y_line+2, y_line+2], z=[z_c, z_c], mode='lines', line=dict(color='white', width=4)))
# Side Nettings
for y_c in [y_line, y_line+2]:
    fig.add_trace(go.Scatter3d(x=[xl, xl], y=[y_line, y_line+2], z=[0, 0], mode='lines', line=dict(color='white', width=4)))
    fig.add_trace(go.Scatter3d(x=[xr, xr], y=[y_line, y_line+2], z=[0, 0], mode='lines', line=dict(color='white', width=4)))

# 3. Trajectory and Refined Ball
fig.add_trace(go.Scatter3d(x=z_p, y=x_p, z=y_p, mode='lines', line=dict(color='red', width=8)))
fig.add_trace(go.Scatter3d(x=[z_p[-1]], y=[x_p[-1]], z=[y_p[-1]], mode='markers', marker=dict(size=16, color='white', line=dict(color='black', width=3))))
# Ball Panels
fig.add_trace(go.Scatter3d(x=[z_p[-1], z_p[-1]], y=[x_p[-1], x_p[-1]], z=[y_p[-1]+0.05, y_p[-1]-0.05], mode='markers', marker=dict(size=8, color='black')))

fig.update_layout(scene=dict(xaxis=dict(visible=False), yaxis=dict(visible=False), zaxis=dict(visible=False)), height=750, margin=dict(l=0,r=0,b=0,t=0))
st.plotly_chart(fig, use_container_width=True)
