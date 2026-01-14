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
def calculate_metrics(df, target, tolerance_percent=0.02):
    # 1. RMSE (Accuracy)
    error_series = df['Target'] - df['Actual']
    rmse = np.sqrt((error_series ** 2).mean())

    # 2. Max Overshoot (Stability)
    max_alt = df['Actual'].max()
    overshoot = max(0, max_alt - target)
    overshoot_percent = (overshoot / target) * 100 if target != 0 else 0

    # 3. Settling Time (Speed)
    upper_bound = target * (1 + tolerance_percent)
    lower_bound = target * (1 - tolerance_percent)

    out_of_band = df[(df['Actual'] > upper_bound) | (df['Actual'] < lower_bound)]
    
    if out_of_band.empty:
        settling_time = 0.0 
    else:
        settling_time = out_of_band['Time'].iloc[-1]
        if settling_time == df['Time'].iloc[-1]:
            settling_time = float('inf') 

    return rmse, overshoot_percent, settling_time

# --- HELPER: Run Simulation ---
def run_simulation_headless(kp, ki, kd, steps, target_alt, mode='balanced'):
    build_dir = "../build"
    exe_name = "./flight_controller"
    csv_path = os.path.join(build_dir, "telemetry.csv")
    
    try:
        subprocess.run(
            [exe_name, str(kp), str(ki), str(kd), str(steps), str(target_alt)], 
            cwd=build_dir, check=True, stdout=subprocess.DEVNULL
        )
    except subprocess.CalledProcessError:
        return float('inf')

    if not os.path.exists(csv_path): return float('inf')
    
    df = pd.read_csv(csv_path)
    rmse, _, settling_time = calculate_metrics(df, target=target_alt)
    
    # --- COST FUNCTION SWITCH ---
    if mode == 'accuracy':
        # Strategy A: Only care about being close to the line (RMSE)
        # This might result in very slow, "lazy" convergence
        return rmse
    else:
        # Strategy B: Balanced (RMSE + Speed)
        # Penalize slow settling times heavily
        if settling_time == float('inf'):
            time_penalty = 100.0
        else:
            time_penalty = settling_time * 0.5 
        return rmse + time_penalty

# --- OPTIMIZATION ALGO ---
def optimize_pid(progress_bar, target_alt, steps, mode='balanced'):
    p = [0.5, 0.0, 0.0] 
    dp = [0.1, 0.01, 0.01] 
    
    # Initial run
    best_err = run_simulation_headless(p[0], p[1], p[2], steps, target_alt, mode)
    
    threshold = 0.005
    iteration = 0
    max_iter = 30 
    
    while sum(dp) > threshold and iteration < max_iter:
        for i in range(len(p)):
            p[i] += dp[i]
            if p[i] < 0: p[i] = 0 
            
            err = run_simulation_headless(p[0], p[1], p[2], steps, target_alt, mode)
            
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                if p[i] < 0: p[i] = 0
                err = run_simulation_headless(p[0], p[1], p[2], steps, target_alt, mode)
                
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

# --- CALLBACKS ---
def run_accuracy_opt():
    _run_optimization(mode='accuracy', label="Accuracy")

def run_balanced_opt():
    _run_optimization(mode='balanced', label="Balanced")

def _run_optimization(mode, label):
    status = st.sidebar.empty()
    status.write(f"Optimizing for {label}...")
    bar = st.sidebar.progress(0)
    
    tgt = st.session_state.get('target', 100.0)
    stp = st.session_state.get('steps', 1000)

    best_p, min_err = optimize_pid(bar, tgt, stp, mode=mode)
    
    st.session_state['kp'] = clamp(best_p[0], 0.0, 5.0)
    st.session_state['ki'] = clamp(best_p[1], 0.0, 1.0)
    st.session_state['kd'] = clamp(best_p[2], 0.0, 1.0)
    
    status.success(f"{label} Optimized! Cost: {min_err:.2f}")
    bar.empty()

# 2. SIDEBAR CONFIG
st.sidebar.header("🕹️ Simulation Settings")

if 'kp' not in st.session_state: st.session_state['kp'] = 0.6
if 'ki' not in st.session_state: st.session_state['ki'] = 0.01
if 'kd' not in st.session_state: st.session_state['kd'] = 0.05
if 'target' not in st.session_state: st.session_state['target'] = 100.0

with st.sidebar.form("pid_form"):
    st.subheader("Flight Parameters")
    target_alt = st.slider("Target Altitude (m)", 10.0, 300.0, key='target', step=10.0)
    steps = st.slider("Simulation Steps", 50, 2000, 1000, 50, key='steps')

    st.subheader("PID Gains")
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, key='kp', step=0.01)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, key='ki', step=0.001)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, key='kd', step=0.01)
    
    submitted = st.form_submit_button("🚀 Run Simulation")

st.sidebar.divider()
st.sidebar.subheader("🤖 AI Auto-Tuner")

col_a, col_b = st.sidebar.columns(2)
with col_a:
    st.button("🎯 Accuracy Only", on_click=run_accuracy_opt, help="Minimizes Error (RMSE). Ignores time.")
with col_b:
    st.button("⚡ Accuracy + Speed", on_click=run_balanced_opt, help="Minimizes Error AND Settling Time.")

# 3. MAIN LOGIC
if submitted:
    build_dir = "../build"
    exe_name = "./flight_controller" 
    try:
        subprocess.run([exe_name, str(kp), str(ki), str(kd), str(steps), str(target_alt)], cwd=build_dir, check=True)
    except Exception as e:
        st.error(f"Error: {e}")
        st.stop()

    csv_path = os.path.join(build_dir, "telemetry.csv")
    if not os.path.exists(csv_path):
        st.error("Telemetry file missing.")
        st.stop()
        
    df = pd.read_csv(csv_path)

    rmse, overshoot, settling_time = calculate_metrics(df, target=target_alt)

    col1, col2, col3, col4 = st.columns(4)
    col1.metric("Settling Time", f"{settling_time:.2f} s", delta_color="inverse")
    col2.metric("Overshoot", f"{overshoot:.1f} %", delta_color="inverse")
    col3.metric("RMSE Error", f"{rmse:.2f}", delta_color="inverse")
    col4.metric("Target", f"{target_alt} m")

    y_max = max(target_alt * 1.2, df['Actual'].max() + 10)
    
    fig = make_subplots(
        rows=1, cols=2, 
        column_widths=[0.2, 0.8],
        subplot_titles=("Drone View", "Altitude Response"),
        specs=[[{"type": "xy"}, {"type": "xy"}]]
    )

    fig.add_trace(go.Scatter(x=[-0.5, 0.5], y=[df['Target'][0], df['Target'][0]], mode='lines', line=dict(color='red', dash='dash'), name='Target'), row=1, col=1)
    
    initial_alt = df['Actual'][0]
    fig.add_trace(go.Scatter(
        x=[0], y=[initial_alt], mode='text', 
        text=[f"🚁<br><b>{initial_alt:.1f} m</b>"], textposition="bottom center",
        textfont=dict(size=18, color="black"), name='Drone'
    ), row=1, col=1)

    fig.add_trace(go.Scatter(x=df['Time'], y=df['Actual'], mode='lines', line=dict(color='lightgrey'), showlegend=False), row=1, col=2)
    fig.add_trace(go.Scatter(x=[df['Time'][0]], y=[df['Actual'][0]], mode='lines', line=dict(color='blue', width=2), name='Response'), row=1, col=2)

    if settling_time != float('inf') and settling_time > 0:
        fig.add_vline(x=settling_time, line_width=1, line_dash="dash", line_color="green", annotation_text="Settled")

    total_rows = len(df)
    max_frames = 100 
    step = max(1, total_rows // max_frames)
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
        yaxis=dict(range=[-20, y_max], title="Altitude (m)"), xaxis=dict(visible=False, range=[-1, 1]),
        yaxis2=dict(range=[-20, y_max]), xaxis2=dict(title="Time (s)"),
        updatemenus=[{"type": "buttons", "showactive": False, "x": 1.05, "y": 0, 
                      "buttons": [{"label": "▶ Play Flight", "method": "animate", "args": [None, {"frame": {"duration": 20, "redraw": True}, "fromcurrent": True}]}]}]
    )

    st.plotly_chart(fig, use_container_width=True)
    st.divider()
    csv_data = df.to_csv(index=False).encode('utf-8')
    st.download_button("📥 Download Telemetry CSV", csv_data, "flight_data.csv", "text/csv")