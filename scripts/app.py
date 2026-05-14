import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import numpy as np
import sys

# --- PATH CONFIGURATION ---
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPTS_DIR, ".."))
BUILD_DIR = os.path.join(ROOT_DIR, "build")
EXE_PATH = os.path.join(BUILD_DIR, "flight_controller")
CSV_PATH = os.path.join(BUILD_DIR, "telemetry.csv")


# --- AUTO-COMPILE C++ (cloud/devcontainer support) ---
def ensure_cpp_executable():
    if not os.path.exists(EXE_PATH):
        print("C++ binary not found. Compiling...")
        os.makedirs(BUILD_DIR, exist_ok=True)
        try:
            subprocess.run(["cmake", ".."], cwd=BUILD_DIR, check=True)
            subprocess.run(["cmake", "--build", "."], cwd=BUILD_DIR, check=True)
        except Exception as e:
            st.error(f"Compilation failed: {e}")

ensure_cpp_executable()

# --- PAGE CONFIG ---
st.set_page_config(page_title="AeroStream GCS", layout="wide", page_icon="🚁")
st.title("🚁 AeroStream: 1D Flight Control Learning Platform")


# --- METRICS ENGINE ---
def calculate_metrics(df, switch_step, mode_type, tolerance_percent=0.02):
    if mode_type == "Standard Takeoff":
        segment = df
        target = segment['Target'].iloc[-1]
        start_val = 0.0
    else:
        switch_idx = int(switch_step)
        if switch_idx >= len(df):
            switch_idx = 0
        segment = df.iloc[switch_idx:].copy()
        target = segment['Target'].iloc[0]
        start_val = df['Target'].iloc[0]

    error_series = segment['Target'] - segment['Actual']
    rmse = np.sqrt((error_series ** 2).mean())

    max_alt = segment['Actual'].max()
    min_alt = segment['Actual'].min()
    if target > start_val:
        overshoot = max(0, max_alt - target)
    else:
        overshoot = max(0, target - min_alt)
    overshoot_percent = (overshoot / target) * 100 if target != 0 else 0

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


# --- SIMULATION RUNNER ---
def run_simulation_headless(kp, ki, kd, steps, t1, t2, switch,
                             mission_mode, opt_strategy, noise_sigma=0.5):
    try:
        subprocess.run(
            [EXE_PATH, str(kp), str(ki), str(kd), str(steps),
             str(t1), str(t2), str(switch), str(noise_sigma)],
            cwd=BUILD_DIR, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    except Exception:
        return float('inf')

    if not os.path.exists(CSV_PATH):
        return float('inf')

    df = pd.read_csv(CSV_PATH)
    rmse, _, settling_time = calculate_metrics(df, switch, mission_mode)

    if opt_strategy == "accuracy":
        return rmse
    else:
        time_penalty = 100.0 if settling_time == float('inf') else settling_time * 0.5
        return rmse + time_penalty


# --- TWIDDLE OPTIMIZER ---
def optimize_pid(progress_bar, t1, t2, switch, steps, mission_mode, opt_strategy,
                 noise_sigma=0.5):
    p = [0.5, 0.0, 0.0]
    dp = [0.1, 0.01, 0.01]
    best_err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch,
                                       mission_mode, opt_strategy, noise_sigma)
    iteration = 0
    max_iter = 30

    while sum(dp) > 0.005 and iteration < max_iter:
        for i in range(len(p)):
            p[i] += dp[i]
            if p[i] < 0:
                p[i] = 0
            err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch,
                                          mission_mode, opt_strategy, noise_sigma)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]
                if p[i] < 0:
                    p[i] = 0
                err = run_simulation_headless(p[0], p[1], p[2], steps, t1, t2, switch,
                                              mission_mode, opt_strategy, noise_sigma)
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


# --- OPTIMIZATION CALLBACKS ---
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

    best_p, min_err = optimize_pid(bar, t1, t2, switch, steps, mode, strategy,
                                   noise_sigma=0.5)

    st.session_state['kp'] = clamp(best_p[0], 0.0, 5.0)
    st.session_state['ki'] = clamp(best_p[1], 0.0, 1.0)
    st.session_state['kd'] = clamp(best_p[2], 0.0, 1.0)

    status.success(f"{label} optimized! Cost: {min_err:.2f}")
    bar.empty()


# --- SIDEBAR ---
st.sidebar.header("🕹️ Mission Control",
                  help="Configure the PID controller and mission, then click 'Run Mission'.")

if 'kp' not in st.session_state: st.session_state['kp'] = 0.6
if 'ki' not in st.session_state: st.session_state['ki'] = 0.01
if 'kd' not in st.session_state: st.session_state['kd'] = 0.05
if 'target' not in st.session_state: st.session_state['target'] = 100.0
if 't1' not in st.session_state: st.session_state['t1'] = 50.0
if 't2' not in st.session_state: st.session_state['t2'] = 100.0

mission_mode = st.sidebar.radio(
    "Select Mission Profile",
    ["Standard Takeoff", "Step Response"],
    captions=[
        "Takeoff from 0 m to target altitude.",
        "Mid-flight altitude change (tests agility)."
    ],
    key="mission_mode"
)

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
    if switch_val == -1:
        switch_val = int(steps * 0.3)

    st.divider()
    st.subheader("PID Gains")
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, key='kp', step=0.01)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, key='ki', step=0.001)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, key='kd', step=0.01)

    st.divider()
    st.subheader("Sensor Noise")
    noise_sigma = st.slider("Noise σ (m)", 0.0, 3.0, 0.5, 0.01, key='noise_sigma',
                             help="Standard deviation of Gaussian altimeter noise.")

    submitted = st.form_submit_button("🚀 Run Mission")

st.sidebar.divider()
st.sidebar.subheader("🤖 AI Auto-Tuner",
                     help="Coordinate Descent (Twiddle) runs at σ=0.5 m baseline noise.")
col_a, col_b = st.sidebar.columns(2)
with col_a: st.button("🎯 Accuracy", on_click=lambda: run_optimization("accuracy"))
with col_b: st.button("⚡ Balanced", on_click=lambda: run_optimization("balanced"))


# --- TABS ---
tab_mission, tab_noise = st.tabs(["Mission Simulation", "Noise Analysis"])


# ============================================================
# TAB 1: MISSION SIMULATION
# ============================================================
with tab_mission:
    if not submitted:
        st.markdown("""
            <style>
            @keyframes bigHover {
                0%   { transform: translateY(0px) rotate(0deg); }
                25%  { transform: translateY(-20px) rotate(-5deg); }
                50%  { transform: translateY(0px) rotate(0deg); }
                75%  { transform: translateY(-20px) rotate(5deg); }
                100% { transform: translateY(0px) rotate(0deg); }
            }
            .splash-container {
                display: flex; flex-direction: column;
                align-items: center; justify-content: center;
                height: 60vh; opacity: 0.4;
            }
            .giant-drone { font-size: 15rem; animation: bigHover 4s ease-in-out infinite; }
            .splash-text { font-size: 2rem; font-weight: bold; color: #888; margin-top: 20px; }
            </style>
            <div class="splash-container">
                <div class="giant-drone">🚁</div>
                <div class="splash-text">SYSTEM STANDBY</div>
                <div>Configure parameters and click "Run Mission"</div>
            </div>
        """, unsafe_allow_html=True)

    if submitted:
        try:
            subprocess.run(
                [EXE_PATH, str(kp), str(ki), str(kd), str(steps),
                 str(t1_val), str(t2_val), str(switch_val), str(noise_sigma)],
                cwd=BUILD_DIR, check=True
            )
        except Exception as e:
            st.error(f"Simulation error: {e}")
            st.stop()

        if not os.path.exists(CSV_PATH):
            st.error("Telemetry file missing.")
            st.stop()

        df = pd.read_csv(CSV_PATH)
        rmse, overshoot, settling_time = calculate_metrics(df, switch_val, mission_mode)

        col1, col2, col3 = st.columns(3)
        col1.metric("Settling Time", f"{settling_time:.2f} s", delta_color="inverse")
        col2.metric("Overshoot", f"{overshoot:.1f} %", delta_color="inverse")
        col3.metric("RMSE Error", f"{rmse:.2f} m", delta_color="inverse")

        y_max = max(max(t1_val, t2_val) * 1.2, df['Actual'].max() + 10)

        fig = make_subplots(
            rows=1, cols=2,
            column_widths=[0.2, 0.8],
            subplot_titles=("Drone View", mission_mode),
            specs=[[{"type": "xy"}, {"type": "xy"}]]
        )

        fig.add_trace(go.Scatter(
            x=[-0.5, 0.5], y=[t2_val, t2_val],
            mode='lines', line=dict(color='red', dash='dash'), name='Target'
        ), row=1, col=1)

        initial_alt = df['Actual'][0]
        fig.add_trace(go.Scatter(
            x=[0], y=[initial_alt], mode='text',
            text=[f"🚁<br><b>{initial_alt:.1f} m</b>"],
            textposition="bottom center",
            textfont=dict(size=18, color="black"), name='Drone'
        ), row=1, col=1)

        fig.add_trace(go.Scatter(
            x=df['Time'], y=df['Target'],
            mode='lines', line=dict(color='red', dash='dash'), name='Target'
        ), row=1, col=2)
        fig.add_trace(go.Scatter(
            x=df['Time'], y=df['Actual'],
            mode='lines', line=dict(color='blue', width=2), name='Response'
        ), row=1, col=2)

        if settling_time != float('inf') and settling_time > 0:
            start_time = df['Time'].iloc[switch_val] if switch_val < len(df) else 0.0
            fig.add_vline(
                x=start_time + settling_time,
                line_width=2, line_dash="dash", line_color="green",
                annotation_text="Settled", annotation_position="top right"
            )

        total_rows = len(df)
        step_size = max(1, total_rows // 100)
        frames = []
        for i in range(0, total_rows, step_size):
            row = df.iloc[i]
            current_data = df.iloc[:i + 1]
            frames.append(go.Frame(
                data=[
                    go.Scatter(y=[row['Target'], row['Target']]),
                    go.Scatter(y=[row['Actual']],
                               text=[f"🚁<br><b>{row['Actual']:.1f} m</b>"]),
                    go.Scatter(x=df['Time'], y=df['Target']),
                    go.Scatter(x=current_data['Time'], y=current_data['Actual'])
                ],
                traces=[0, 1, 2, 3], name=f"frame_{i}"
            ))
        fig.frames = frames

        fig.update_layout(
            height=500, hovermode="x unified", template="plotly_white",
            yaxis=dict(range=[-10, y_max], title="Altitude (m)"),
            xaxis=dict(visible=False, range=[-1, 1]),
            yaxis2=dict(range=[-10, y_max]),
            xaxis2=dict(title="Time (s)"),
            updatemenus=[{
                "type": "buttons", "showactive": True,
                "x": 1.05, "y": -0.1,
                "buttons": [{"label": "▶ Play", "method": "animate",
                              "args": [None, {"frame": {"duration": 60, "redraw": True},
                                              "fromcurrent": True}]}]
            }]
        )

        st.plotly_chart(fig, use_container_width=True)
        st.divider()
        csv_data = df.to_csv(index=False).encode('utf-8')
        st.download_button("📥 Download Telemetry CSV", csv_data, "flight_data.csv", "text/csv")


# ============================================================
# TAB 2: NOISE ANALYSIS
# ============================================================
with tab_noise:
    st.header("Noise Sensitivity Analysis")
    st.markdown("""
    This sweep runs the simulation with the same PID gains and mission profile five times,
    varying only the altimeter noise standard deviation **σ**. The PID derivative term
    amplifies high-frequency noise, so RMSE degrades non-linearly as σ grows — even though
    the underlying physics is identical across all runs. Use this to understand how sensitive
    your chosen gains are to sensor quality.
    """)

    with st.expander("Sweep Configuration", expanded=True):
        nc1, nc2, nc3 = st.columns(3)
        with nc1:
            sweep_kp = st.number_input("Kp", value=0.6, min_value=0.0, max_value=5.0,
                                       key='sweep_kp', format="%.3f")
            sweep_ki = st.number_input("Ki", value=0.01, min_value=0.0, max_value=1.0,
                                       key='sweep_ki', format="%.3f")
            sweep_kd = st.number_input("Kd", value=0.05, min_value=0.0, max_value=1.0,
                                       key='sweep_kd', format="%.3f")
        with nc2:
            sweep_t1 = st.number_input("Start Altitude (m)", value=50.0, key='sweep_t1')
            sweep_t2 = st.number_input("Final Altitude (m)", value=100.0, key='sweep_t2')
        with nc3:
            sweep_steps = st.number_input("Simulation Steps", value=500,
                                          min_value=50, max_value=3000, key='sweep_steps')

    NOISE_SIGMAS = [0.01, 0.1, 0.5, 1.0, 2.0]
    TRACE_COLORS = ['#2196F3', '#4CAF50', '#FF9800', '#F44336', '#9C27B0']

    if st.button("🔬 Run Noise Sweep", type="primary"):
        sweep_switch = int(sweep_steps * 0.3)
        rmse_results = []
        all_dfs = []

        progress = st.progress(0, text="Running simulations...")
        for idx, sigma in enumerate(NOISE_SIGMAS):
            progress.progress((idx + 1) / len(NOISE_SIGMAS),
                               text=f"Running σ = {sigma} m  ({idx + 1}/{len(NOISE_SIGMAS)})")
            run_simulation_headless(
                sweep_kp, sweep_ki, sweep_kd,
                int(sweep_steps), sweep_t1, sweep_t2, sweep_switch,
                "Step Response", "accuracy", noise_sigma=sigma
            )
            if os.path.exists(CSV_PATH):
                df_s = pd.read_csv(CSV_PATH).copy()
                df_s['sigma'] = sigma
                all_dfs.append(df_s)
                rmse, _, _ = calculate_metrics(df_s, sweep_switch, "Step Response")
                rmse_results.append(rmse)
            else:
                rmse_results.append(float('nan'))
        progress.empty()

        # Plot 1: RMSE bar chart
        fig_bar = go.Figure(go.Bar(
            x=[f"σ={s}" for s in NOISE_SIGMAS],
            y=rmse_results,
            marker_color=TRACE_COLORS,
            text=[f"{r:.3f} m" for r in rmse_results],
            textposition='outside'
        ))
        fig_bar.update_layout(
            title="RMSE vs Altimeter Noise σ",
            xaxis_title="Noise Standard Deviation σ (m)",
            yaxis_title="RMSE (m)",
            template="plotly_white", height=380,
            showlegend=False
        )
        st.plotly_chart(fig_bar, use_container_width=True)

        # Plot 2: Overlaid altitude time-series
        if all_dfs:
            fig_ts = go.Figure()
            for df_s, sigma, color in zip(all_dfs, NOISE_SIGMAS, TRACE_COLORS):
                fig_ts.add_trace(go.Scatter(
                    x=df_s['Time'], y=df_s['Actual'],
                    mode='lines', name=f"σ = {sigma} m",
                    line=dict(color=color, width=1.5)
                ))
            fig_ts.add_trace(go.Scatter(
                x=all_dfs[0]['Time'], y=all_dfs[0]['Target'],
                mode='lines', name='Target',
                line=dict(color='black', dash='dash', width=2)
            ))
            fig_ts.update_layout(
                title="Altitude Response at Each Noise Level",
                xaxis_title="Time (s)", yaxis_title="Altitude (m)",
                template="plotly_white", height=420
            )
            st.plotly_chart(fig_ts, use_container_width=True)

            st.info(
                "**Reading the chart:** At low σ (blue), the response closely tracks the "
                "target. As σ increases the controller receives a noisier altitude reading, "
                "causing the derivative term to react to noise rather than true dynamics — "
                "raising RMSE and roughening the trajectory. Reducing Kd or increasing the "
                "derivative filter coefficient N improves robustness at higher noise levels."
            )
