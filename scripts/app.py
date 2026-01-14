import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import numpy as np

# 1. PAGE CONFIG
st.set_page_config(page_title="AeroStream GCS", layout="wide", page_icon="🚁")
st.title("🚁 AeroStream: Multi-Mode PID Flight Control")

# --- PHYSICS METRICS ENGINE ---
def calculate_metrics(df, switch_step, mode_type, tolerance_percent=0.02):
    if mode_type == "Standard Takeoff":
        segment = df 
        target = segment['Target'].iloc[-1]
        start_val = 0.0 
    else:
        switch_idx = int(switch_step)
        if switch_idx >= len(df): switch_idx = 0
        segment = df.iloc[switch_idx:].copy()
        target = segment['Target'].iloc[0]
        start_val = df['Target'].iloc[0] 

    # 1. RMSE
    error_series = segment['Target'] - segment['Actual']
    rmse = np.sqrt((error_series ** 2).mean())

    # 2. Overshoot
    max_alt = segment['Actual'].max()
    min_alt = segment['Actual'].min()
    
    if target > start_val: 
        overshoot = max(0, max_alt - target)
    else: 
        overshoot = max(0, target - min_alt)
        
    overshoot_percent = (overshoot / target) * 100 if target != 0 else 0

    # 3. Settling Time
    upper_bound = target * (1 + tolerance_percent)
    lower_bound = target * (1 - tolerance_percent)

    out_of_band = segment[(segment['Actual'] > upper_bound) | (segment['Actual'] < lower_bound)]
    
    if out_of_band.empty:
        settling_time = 0.0
    else:
        settling_time = out_of_band['Time'].iloc[-1] - segment['Time'].iloc[0]
        if out_of_band['Time'].iloc[-1] == segment['Time'].iloc[-1]:
            settling_time = float('inf')

    return rmse, overshoot_percent, settling_time

# --- HELPER: Run Simulation ---
def run_simulation_headless(kp, ki, kd, steps, t1, t2, switch, mission_mode, opt_strategy):
    build_dir = "../build"
    exe_name = "./flight_controller"
    csv_path = os.path.join(build_dir, "telemetry.csv")
    
    try:
        subprocess.run(
            [exe_name, str(kp), str(ki), str(kd), str(steps), str(t1), str(t2), str(switch)], 
            cwd=build_dir, check=True, stdout=subprocess.DEVNULL
        )
    except subprocess.CalledProcessError:
        return float('inf')

    if not os.path.exists(csv_path): return float('inf')
    
    df = pd.read_csv(csv_path)
    rmse, _, settling_time = calculate_metrics(df, switch, mission_mode)
    
    if opt_strategy == "accuracy":
        return rmse 
    else:
        if settling_time == float('inf'):
            time_penalty = 100.0
        else:
            time_penalty = settling_time * 0.5 
        return rmse + time_penalty

# --- OPTIMIZATION ALGO ---
def optimize_pid(progress_bar, t1, t2, switch, steps, mission_mode, opt_strategy):
    p = [0.5, 0.0, 0.0] 
    dp = [0.1, 0.01, 0.01] 
    
    best_err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch, mission_mode, opt_strategy)
    
    threshold = 0.005
    iteration = 0
    max_iter = 30 
    
    while sum(dp) > threshold and iteration < max_iter:
        for i in range(len(p)):
            p[i] += dp[i]
            if p[i] < 0: p[i] = 0 
            
            err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch, mission_mode, opt_strategy)
            
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                if p[i] < 0: p[i] = 0
                err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch, mission_mode, opt_strategy)
                
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
            iteration += 1
            progress_bar.progress(min(iteration / max_iter, 1.0))
            
    return p, best_err

def clamp(n, minn, maxn): return max(min(maxn, n), minn)

# --- SHARED CALLBACK ---
def run_optimization(strategy):
    label = "Accuracy" if strategy == "accuracy" else "Balanced"
    status = st.sidebar.empty()
    status.write(f"Optimizing ({label})...")
    bar = st.sidebar.progress(0)
    
    mode = st.session_state.get('mission_mode', "Standard Takeoff")
    steps = st.session_state.get('steps', 1000)
    
    if mode == "Standard Takeoff":
        tgt = st.session_state.get('target', 100.0)
        t1, t2, switch = tgt, tgt, 0
    else:
        t1 = st.session_state.get('t1', 50.0)
        t2 = st.session_state.get('t2', 100.0)
        switch = int(steps * 0.3)

    best_p, min_err = optimize_pid(bar, t1, t2, switch, steps, mode, strategy)
    
    st.session_state['kp'] = clamp(best_p[0], 0.0, 5.0)
    st.session_state['ki'] = clamp(best_p[1], 0.0, 1.0)
    st.session_state['kd'] = clamp(best_p[2], 0.0, 1.0)
    
    status.success(f"{label} Optimized! Cost: {min_err:.2f}")
    bar.empty()

# 2. SIDEBAR CONFIG
st.sidebar.header("🕹️ Mission Control", help="Press 'Run Mission' to calculate the flight and then 'Play' to view the resulting telemetry data. Use the 'AI Auto-Tuner' to optimize the PID parameters to the flight mission.")

if 'kp' not in st.session_state: st.session_state['kp'] = 0.6
if 'ki' not in st.session_state: st.session_state['ki'] = 0.01
if 'kd' not in st.session_state: st.session_state['kd'] = 0.05
if 'target' not in st.session_state: st.session_state['target'] = 100.0
if 't1' not in st.session_state: st.session_state['t1'] = 50.0
if 't2' not in st.session_state: st.session_state['t2'] = 100.0

mission_mode = st.sidebar.radio("Select Mission Profile", ["Standard Takeoff", "Step Response"],captions=[
        "Simulates takeoff from Ground (0m) to Target.",
        "Simulates a mid-flight altitude change."
    ], key="mission_mode")

with st.sidebar.form("pid_form"):
    if mission_mode == "Standard Takeoff":
        t_final = st.slider("Target Altitude (m)", 10.0, 300.0, key='target', step=10.0)
        t1_val, t2_val, switch_val = t_final, t_final, 0
    else:
        col_f1, col_f2 = st.columns(2)
        with col_f1: t1_val = st.number_input("Start Altitude (m)", value=50.0, key='t1')
        with col_f2: t2_val = st.number_input("Final Altitude (m)", value=100.0, key='t2')
        switch_val = -1 

    steps = st.slider("Simulation Steps", 50, 3000, 500, 50, key='steps')
    if switch_val == -1: switch_val = int(steps * 0.3)

    st.divider()
    st.subheader("PID Gains")
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, key='kp', step=0.01)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, key='ki', step=0.001)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, key='kd', step=0.01)
    
    submitted = st.form_submit_button("🚀 Run Mission")

# --- AI AUTO-TUNER SECTION ---
st.sidebar.divider()
st.sidebar.subheader("🤖 AI Auto-Tuner", help="Calculate best PID parameters for the flight mission.")
col_a, col_b = st.sidebar.columns(2)
with col_a: st.button("🎯 Accuracy", on_click=lambda: run_optimization("accuracy"), help="Minimizes Error only.")
with col_b: st.button("⚡ Balanced", on_click=lambda: run_optimization("balanced"), help="Minimizes Error + Time.")

# --- SPLASH SCREEN LOGIC ---
if not submitted:
    st.markdown("""
        <style>
        @keyframes bigHover {
            0% { transform: translateY(0px) rotate(0deg); }
            25% { transform: translateY(-20px) rotate(-5deg); }
            50% { transform: translateY(0px) rotate(0deg); }
            75% { transform: translateY(-20px) rotate(5deg); }
            100% { transform: translateY(0px) rotate(0deg); }
        }
        .splash-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 60vh; /* Occupy 60% of screen height */
            opacity: 0.4; /* Faded "Background" look */
        }
        .giant-drone {
            font-size: 15rem; /* Massive Emoji */
            animation: bigHover 4s ease-in-out infinite;
        }
        .splash-text {
            font-size: 2rem;
            font-weight: bold;
            color: #888;
            margin-top: 20px;
        }
        </style>
        <div class="splash-container">
            <div class="giant-drone">🚁</div>
            <div class="splash-text">SYSTEM STANDBY</div>
            <div>Configure parameters and click "Run Mission"</div>
        </div>
    """, unsafe_allow_html=True)

# 3. MAIN LOGIC
if submitted:
    build_dir = "../build"
    exe_name = "./flight_controller"
    
    try:
        subprocess.run([exe_name, str(kp), str(ki), str(kd), str(steps), str(t1_val), str(t2_val), str(switch_val)], cwd=build_dir, check=True)
    except Exception as e:
        st.error(f"Error: {e}")
        st.stop()

    csv_path = os.path.join(build_dir, "telemetry.csv")
    if not os.path.exists(csv_path):
        st.error("Telemetry file missing.")
        st.stop()
        
    df = pd.read_csv(csv_path)
    rmse, overshoot, settling_time = calculate_metrics(df, switch_val, mission_mode)

    col1, col2, col3 = st.columns(3)
    col1.metric("Settling Time", f"{settling_time:.2f} s", delta_color="inverse")
    col2.metric("Overshoot", f"{overshoot:.1f} %", delta_color="inverse")
    col3.metric("RMSE Error", f"{rmse:.2f}", delta_color="inverse")

    y_max = max(max(t1_val, t2_val) * 1.2, df['Actual'].max() + 10)
    
    fig = make_subplots(
        rows=1, cols=2, 
        column_widths=[0.2, 0.8],
        subplot_titles=("Drone View", mission_mode),
        specs=[[{"type": "xy"}, {"type": "xy"}]]
    )

    fig.add_trace(go.Scatter(x=[-0.5, 0.5], y=[t2_val, t2_val], mode='lines', line=dict(color='red', dash='dash'), name='Target'), row=1, col=1)
    
    initial_alt = df['Actual'][0]
    fig.add_trace(go.Scatter(
        x=[0], y=[initial_alt], mode='text', 
        text=[f"🚁<br><b>{initial_alt:.1f} m</b>"], textposition="bottom center",
        textfont=dict(size=18, color="black"), name='Drone'
    ), row=1, col=1)

    fig.add_trace(go.Scatter(x=df['Time'], y=df['Target'], mode='lines', line=dict(color='red', dash='dash'), name='Target'), row=1, col=2)
    fig.add_trace(go.Scatter(x=df['Time'], y=df['Actual'], mode='lines', line=dict(color='blue', width=2), name='Response'), row=1, col=2)

    # SETTLING LINE
    if settling_time != float('inf') and settling_time > 0:
        start_time = df['Time'].iloc[switch_val] if switch_val < len(df) else 0.0
        settled_abs_time = start_time + settling_time
        
        fig.add_vline(
            x=settled_abs_time, 
            line_width=2, line_dash="dash", line_color="green", 
            annotation_text="Settled", annotation_position="top right"
        )

    total_rows = len(df)
    max_frames = 100 
    step = max(1, total_rows // max_frames)
    frames = []
    for i in range(0, total_rows, step):
        row = df.iloc[i]
        current_data = df.iloc[:i+1]
        curr_tgt_line = row['Target']
        
        frames.append(go.Frame(
            data=[
                go.Scatter(y=[curr_tgt_line, curr_tgt_line]), 
                go.Scatter(y=[row['Actual']], text=[f"🚁<br><b>{row['Actual']:.1f} m</b>"]), 
                go.Scatter(x=df['Time'], y=df['Target']), 
                go.Scatter(x=current_data['Time'], y=current_data['Actual'])
            ],
            traces=[0, 1, 2, 3], name=f"frame_{i}"
        ))
    fig.frames = frames

    fig.update_layout(
        height=500, hovermode="x unified", template="plotly_white",
        yaxis=dict(range=[-10, y_max], title="Altitude (m)"), xaxis=dict(visible=False, range=[-1, 1]),
        yaxis2=dict(range=[-10, y_max]), xaxis2=dict(title="Time (s)"),
        updatemenus=[{
            "type": "buttons", 
            "showactive": True, 
            "x": 1.05, 
            "y": -0.1, 
            "buttons": [{
                "label": "▶ Play", 
                "method": "animate", 
                "args": [
                    None, 
                    {
                        "frame": {"duration": 60, "redraw": True}, # <--- Slower Speed (60ms)
                        "fromcurrent": True
                    }
                ]
            }]
        }]
    )

    st.plotly_chart(fig, use_container_width=True)
    st.divider()
    csv_data = df.to_csv(index=False).encode('utf-8')
    st.download_button("📥 Download Telemetry CSV", csv_data, "flight_data.csv", "text/csv")