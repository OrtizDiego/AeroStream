import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import numpy as np

# 1. PAGE CONFIG
st.set_page_config(page_title="AeroStream GCS", layout="wide", page_icon="🚁")
st.title("🚁 AeroStream: Intelligent Flight Control System")

# --- PHYSICS METRICS ENGINE ---
def calculate_metrics(df, target=100.0, tolerance_percent=0.02):
    """
    Calculates key control theory metrics:
    1. Settling Time: Time to enter and stay within 2% of target.
    2. Max Overshoot: Max peak above target.
    3. RMSE: Root Mean Squared Error (Overall accuracy).
    """
    # 1. RMSE (Accuracy)
    error_series = df['Target'] - df['Actual']
    rmse = np.sqrt((error_series ** 2).mean())

    # 2. Max Overshoot (Stability)
    max_alt = df['Actual'].max()
    overshoot = max(0, max_alt - target)
    overshoot_percent = (overshoot / target) * 100

    # 3. Settling Time (Speed)
    # Define the "Stability Band" (e.g., 98m to 102m)
    upper_bound = target * (1 + tolerance_percent)
    lower_bound = target * (1 - tolerance_percent)

    # Find time points where we are OUTSIDE the band
    out_of_band = df[(df['Actual'] > upper_bound) | (df['Actual'] < lower_bound)]
    
    if out_of_band.empty:
        settling_time = 0.0 # Perfectly stable from start (unlikely)
    else:
        # The settling time is the timestamp of the LAST point we were outside the band
        settling_time = out_of_band['Time'].iloc[-1]
        
        # If the last point of the simulation is still out of band, we never settled
        if settling_time == df['Time'].iloc[-1]:
            settling_time = float('inf') 

    return rmse, overshoot_percent, settling_time

# --- HELPER: Run Simulation & Get Composite Cost ---
def run_simulation_headless(kp, ki, kd, steps=500):
    build_dir = "../build"
    exe_name = "./flight_controller"
    csv_path = os.path.join(build_dir, "telemetry.csv")
    
    try:
        subprocess.run(
            [exe_name, str(kp), str(ki), str(kd), str(steps)], 
            cwd=build_dir, check=True, stdout=subprocess.DEVNULL
        )
    except subprocess.CalledProcessError:
        return float('inf')

    if not os.path.exists(csv_path): return float('inf')
    
    df = pd.read_csv(csv_path)
    
    # Calculate Metrics
    rmse, _, settling_time = calculate_metrics(df, target=100.0)
    
    # --- THE MAGIC FORMULA ---
    # Cost = Accuracy (RMSE) + Speed (Settling Time)
    # We penalize settling time heavily (0.5 cost per second) to force faster stabilization
    if settling_time == float('inf'):
        time_penalty = 100.0 # Huge penalty if it never stabilizes
    else:
        time_penalty = settling_time * 0.5 

    total_cost = rmse + time_penalty
    return total_cost

# --- ALGORITHM: Twiddle (Coordinate Descent) ---
def optimize_pid(progress_bar):
    p = [0.5, 0.0, 0.0] 
    dp = [0.1, 0.01, 0.01] 
    best_err = run_simulation_headless(p[0], p[1], p[2])
    threshold = 0.005
    iteration = 0
    max_iter = 30 
    
    while sum(dp) > threshold and iteration < max_iter:
        for i in range(len(p)):
            p[i] += dp[i]
            if p[i] < 0: p[i] = 0 # Constraint
            
            err = run_simulation_headless(p[0], p[1], p[2])
            
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                if p[i] < 0: p[i] = 0
                err = run_simulation_headless(p[0], p[1], p[2])
                
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
            
            iteration += 1
            progress_bar.progress(min(iteration / max_iter, 1.0))
            
    return p, best_err

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def run_optimization_callback():
    status = st.sidebar.empty()
    status.write("Running Physics Optimization...")
    bar = st.sidebar.progress(0)
    
    best_p, min_err = optimize_pid(bar)
    
    st.session_state['kp'] = clamp(best_p[0], 0.0, 5.0)
    st.session_state['ki'] = clamp(best_p[1], 0.0, 1.0)
    st.session_state['kd'] = clamp(best_p[2], 0.0, 1.0)
    
    status.success(f"Optimized! Cost: {min_err:.2f}")
    bar.empty()

# 2. SIDEBAR CONFIG
st.sidebar.header("🕹️ Simulation Settings")

if 'kp' not in st.session_state: st.session_state['kp'] = 0.6
if 'ki' not in st.session_state: st.session_state['ki'] = 0.01
if 'kd' not in st.session_state: st.session_state['kd'] = 0.05

with st.sidebar.form("pid_form"):
    st.subheader("Manual Tuning")
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, key='kp', step=0.01)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, key='ki', step=0.001)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, key='kd', step=0.01)
    
    st.subheader("Duration")
    steps = st.slider("Simulation Steps", 50, 2000, 500, 50)
    
    submitted = st.form_submit_button("🚀 Run Simulation")

st.sidebar.divider()
st.sidebar.subheader("🤖 AI Auto-Tuner")
st.sidebar.button("✨ Find Optimal Gains", on_click=run_optimization_callback)
st.sidebar.info("Optimizes for accuracy and settling time.")

# 3. MAIN LOGIC
if submitted:
    # --- A. RUN SIMULATION ---
    build_dir = "../build"
    exe_name = "./flight_controller" 
    try:
        subprocess.run([exe_name, str(kp), str(ki), str(kd), str(steps)], cwd=build_dir, check=True)
    except Exception as e:
        st.error(f"Error: {e}")
        st.stop()

    csv_path = os.path.join(build_dir, "telemetry.csv")
    if not os.path.exists(csv_path):
        st.error("Telemetry file missing.")
        st.stop()
        
    df = pd.read_csv(csv_path)

    # --- B. CALCULATE FLIGHT METRICS ---
    rmse, overshoot, settling_time = calculate_metrics(df, target=100.0)

    # --- C. DISPLAY METRICS (The "How do I know" part) ---
    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Settling Time (2%)", f"{settling_time:.2f} s", delta_color="inverse")
    col2.metric("Overshoot", f"{overshoot:.1f} %", delta_color="inverse")
    col3.metric("RMSE Error", f"{rmse:.2f}", delta_color="inverse")
    col4.metric("Current Kp", f"{kp:.2f}")

    # --- D. BUILD ANIMATION ---
    total_rows = len(df)
    max_frames = 100 
    step = max(1, total_rows // max_frames)
    
    fig = make_subplots(
        rows=1, cols=2, 
        column_widths=[0.2, 0.8],
        subplot_titles=("Drone View", "Altitude Response"),
        specs=[[{"type": "xy"}, {"type": "xy"}]]
    )

    # Static Traces
    fig.add_trace(go.Scatter(x=[-0.5, 0.5], y=[df['Target'][0], df['Target'][0]], mode='lines', line=dict(color='red', dash='dash'), name='Target'), row=1, col=1)
    
    initial_alt = df['Actual'][0]
    fig.add_trace(go.Scatter(
        x=[0], y=[initial_alt], mode='text', 
        text=[f"🚁<br><b>{initial_alt:.1f} m</b>"], textposition="bottom center",
        textfont=dict(size=18, color="black"), name='Drone'
    ), row=1, col=1)

    fig.add_trace(go.Scatter(x=df['Time'], y=df['Actual'], mode='lines', line=dict(color='lightgrey'), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=[df['Time'][0]], y=[df['Actual'][0]], mode='lines', line=dict(color='blue', width=2), name='Response'), row=1, col=2)

    # Add Settling Time Marker (Green Line)
    if settling_time != float('inf') and settling_time > 0:
        fig.add_vline(x=settling_time, line_width=1, line_dash="dash", line_color="green", annotation_text="Settled")

    # Frames
    frames = []
    for i in range(0, total_rows, step):
        row = df.iloc[i]
        current_data = df.iloc[:i+1]
        frames.append(go.Frame(
            data=[
                go.Scatter(x=[0], y=[row['Actual']], mode='text', text=[f"🚁<br><b>{row['Actual']:.1f} m</b>"]),
                go.Scatter(x=current_data['Time'], y=current_data['Actual'])
            ],
            traces=[1, 3], name=f"frame_{i}"
        ))
    fig.frames = frames

    fig.update_layout(
        height=500, hovermode="x unified", template="plotly_white",
        yaxis=dict(range=[-20, 150], title="Altitude (m)"), xaxis=dict(visible=False, range=[-1, 1]),
        yaxis2=dict(range=[-20, 150]), xaxis2=dict(title="Time (s)"),
        updatemenus=[{"type": "buttons", "showactive": False, "x": 1.05, "y": 0, 
                      "buttons": [{"label": "▶ Play Flight", "method": "animate", "args": [None, {"frame": {"duration": 20, "redraw": True}, "fromcurrent": True}]}]}]
    )

    st.plotly_chart(fig, use_container_width=True)
    
    st.divider()
    csv_data = df.to_csv(index=False).encode('utf-8')
    st.download_button("📥 Download Telemetry CSV", csv_data, "flight_data.csv", "text/csv")