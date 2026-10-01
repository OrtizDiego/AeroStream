import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os
import numpy as np

# --- PATH CONFIGURATION ---
SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(SCRIPTS_DIR, ".."))
BUILD_DIR = os.path.join(ROOT_DIR, "build")
EXE_PATH = os.path.join(BUILD_DIR, "flight_controller")
CSV_PATH = os.path.join(BUILD_DIR, "telemetry.csv")

# --- CONTROLLER / UI CONSTANTS ---
DT = 0.1                      # must match dt in src/main.cpp
GAIN_LIMITS = {"kp": (0.0, 10.0), "ki": (0.0, 1.0), "kd": (0.0, 10.0)}
FILTER_N_OPTIONS = [1.0, 2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 30.0, 50.0]
DEFAULTS = {
    "kp": 3.0, "ki": 0.2, "kd": 2.0,
    "filter_n": 5.0, "hover_ff": True, "i_zone": 5.0,
    "noise_sigma": 0.5, "fixed_seed": True, "seed": 42,
    "target": 100.0, "t1": 50.0, "t2": 100.0,
}

# Categorical series colors (fixed order, never cycled)
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a"]
TARGET_COLOR = "#52514e"


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


# --- SIMULATION RUNNER ---
def run_sim(kp, ki, kd, steps, t1, t2, switch, noise_sigma=0.5, filter_n=10.0,
            seed=-1, hover_ff=True, i_zone=0.0):
    """Run the C++ simulation and return its telemetry, or None on failure.

    seed < 0 means a fresh random noise sequence on every run.
    """
    args = [EXE_PATH, kp, ki, kd, int(steps), t1, t2, int(switch),
            noise_sigma, filter_n, int(seed), int(bool(hover_ff)), i_zone]
    try:
        subprocess.run([str(a) for a in args], cwd=BUILD_DIR, check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        return None
    if not os.path.exists(CSV_PATH):
        return None
    return pd.read_csv(CSV_PATH)


# --- METRICS ENGINE ---
def calculate_metrics(df, switch_step, mode_type, tolerance_percent=0.02):
    """Step-response metrics, computed on the TRUE altitude (not the noisy sensor).

    Overshoot and the settling band are relative to the step size, so a
    50 m -> 100 m step is judged on its 50 m change, not on the absolute altitude.
    """
    if mode_type == "Standard Takeoff":
        segment = df
        start_val = 0.0
    else:
        switch_idx = int(switch_step)
        if switch_idx >= len(df):
            switch_idx = 0
        segment = df.iloc[switch_idx:]
        start_val = df['Target'].iloc[0]
    target = segment['Target'].iloc[-1]
    step = target - start_val

    error_series = segment['Target'] - segment['Actual']
    rmse = float(np.sqrt((error_series ** 2).mean()))

    if step >= 0:
        overshoot = max(0.0, segment['Actual'].max() - target)
    else:
        overshoot = max(0.0, target - segment['Actual'].min())
    overshoot_percent = (overshoot / abs(step)) * 100 if step != 0 else 0.0

    band = max(abs(step) * tolerance_percent, 0.1)
    out_of_band = segment[error_series.abs() > band]
    if out_of_band.empty:
        settling_time = 0.0
    elif out_of_band.index[-1] == segment.index[-1]:
        settling_time = float('inf')   # still outside the band at the end of the run
    else:
        settling_time = out_of_band['Time'].iloc[-1] - segment['Time'].iloc[0] + DT

    return rmse, overshoot_percent, settling_time


def noise_metrics(df, hold_fraction=0.4):
    """Metrics that isolate the effect of sensor noise on a flight.

    hold_rms: RMS of the true altitude error over the last `hold_fraction` of the
              run (the drone should be holding altitude there) -> position jitter.
    chatter:  standard deviation of the step-to-step thrust change over the same
              window -> how much noise the controller passes through to the motors.
    """
    hold = df.iloc[int(len(df) * (1 - hold_fraction)):]
    hold_rms = float(np.sqrt(((hold['Target'] - hold['Actual']) ** 2).mean()))
    chatter = float(hold['Output'].diff().std())
    return hold_rms, chatter


def mission_setup(mode, steps):
    """(t1, t2, switch) for the selected mission profile."""
    if mode == "Standard Takeoff":
        tgt = st.session_state['target']
        return tgt, tgt, 0
    return st.session_state['t1'], st.session_state['t2'], int(steps * 0.3)


# --- TWIDDLE OPTIMIZER ---
def tuning_cost(p, ctx):
    df = run_sim(p[0], p[1], p[2], ctx['steps'], ctx['t1'], ctx['t2'], ctx['switch'],
                 ctx['noise_sigma'], ctx['filter_n'], ctx['seed'], ctx['hover_ff'],
                 ctx['i_zone'])
    if df is None:
        return float('inf')
    rmse, _, settling_time = calculate_metrics(df, ctx['switch'], ctx['mode'])
    if ctx['strategy'] == "accuracy":
        return rmse
    # Balanced: also reward settling fast and not hammering the motors with noise
    time_penalty = 100.0 if settling_time == float('inf') else settling_time * 0.5
    _, chatter = noise_metrics(df)
    return rmse + time_penalty + 0.2 * chatter


def optimize_pid(progress_bar, ctx, max_rounds=40):
    """Coordinate descent (Twiddle) over (Kp, Ki, Kd).

    Every evaluation uses the same noise seed, so cost differences come from
    the gains and not from a lucky noise draw.
    """
    keys = ["kp", "ki", "kd"]
    p = [st.session_state[k] for k in keys]
    dp = [0.5, 0.02, 0.5]
    tol = 0.01

    def clip(i):
        lo, hi = GAIN_LIMITS[keys[i]]
        p[i] = min(max(p[i], lo), hi)

    best_err = tuning_cost(p, ctx)
    for rnd in range(max_rounds):
        if sum(dp) < tol:
            break
        for i in range(len(p)):
            original = p[i]
            p[i] = original + dp[i]
            clip(i)
            err = tuning_cost(p, ctx)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
                continue
            p[i] = original - dp[i]
            clip(i)
            err = tuning_cost(p, ctx)
            if err < best_err:
                best_err = err
                dp[i] *= 1.1
                continue
            p[i] = original
            dp[i] *= 0.7
        progress_bar.progress((rnd + 1) / max_rounds)

    return p, best_err


# --- OPTIMIZATION CALLBACKS ---
def run_optimization(strategy):
    label = "Accuracy" if strategy == "accuracy" else "Balanced"
    status = st.sidebar.empty()
    status.write(f"Optimizing ({label})...")
    bar = st.sidebar.progress(0)

    mode = st.session_state.get('mission_mode', "Standard Takeoff")
    steps = st.session_state.get('steps', 500)
    t1, t2, switch = mission_setup(mode, steps)
    ctx = dict(
        mode=mode, strategy=strategy, steps=steps, t1=t1, t2=t2, switch=switch,
        noise_sigma=st.session_state['noise_sigma'],
        filter_n=st.session_state['filter_n'],
        hover_ff=st.session_state['hover_ff'],
        i_zone=st.session_state['i_zone'],
        seed=st.session_state['seed'],   # fixed seed: a fair, repeatable objective
    )

    best_p, min_err = optimize_pid(bar, ctx)

    st.session_state['kp'] = round(best_p[0], 2)
    st.session_state['ki'] = round(best_p[1], 3)
    st.session_state['kd'] = round(best_p[2], 2)

    status.success(f"{label} optimized! Cost: {min_err:.2f}")
    bar.empty()


# --- SIDEBAR ---
st.sidebar.header("🕹️ Mission Control",
                  help="Configure the PID controller and mission, then click 'Run Mission'.")

for _k, _v in DEFAULTS.items():
    if _k not in st.session_state:
        st.session_state[_k] = _v

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
    else:
        col_f1, col_f2 = st.columns(2)
        with col_f1: st.number_input("Start Altitude (m)", key='t1')
        with col_f2: st.number_input("Final Altitude (m)", key='t2')

    steps = st.slider("Simulation Steps", 50, 3000, 500, 50, key='steps')

    st.divider()
    st.subheader("PID Gains")
    kp = st.slider("Proportional (Kp)", *GAIN_LIMITS["kp"], key='kp', step=0.05)
    ki = st.slider("Integral (Ki)", *GAIN_LIMITS["ki"], key='ki', step=0.005)
    kd = st.slider("Derivative (Kd)", *GAIN_LIMITS["kd"], key='kd', step=0.05,
                   help="Damping. For this plant (mass 1 kg) a well-damped response "
                        "needs roughly Kd ≈ 1–2 × √Kp.")
    filter_n = st.select_slider(
        "Derivative filter N (rad/s)", FILTER_N_OPTIONS, key='filter_n',
        help="Bandwidth of the low-pass filter on the D term. Lower N = smoother "
             "but more lagged derivative; higher N = sharper but passes more noise.")
    hover_ff = st.checkbox(
        "Hover thrust feed-forward (m·g)", key='hover_ff',
        help="Adds the 9.81 N needed to hover directly to the motor command, so the "
             "PID only corrects deviations. Turn it off to see why the I term is "
             "needed to hold altitude against gravity.")
    i_zone = st.slider(
        "Integral zone (m)", 0.0, 20.0, step=0.5, key='i_zone',
        help="The I term only accumulates while |error| is below this value "
             "(0 = always). This stops error collected during big climbs from "
             "causing overshoot and a slow creep afterwards. Without feed-forward "
             "it must be larger than 9.81/Kp, or the I term never engages.")

    st.divider()
    st.subheader("Sensor Noise")
    noise_sigma = st.slider("Noise σ (m)", 0.0, 3.0, step=0.01, key='noise_sigma',
                            help="Standard deviation of Gaussian altimeter noise.")
    fixed_seed = st.checkbox("Repeatable noise (fixed seed)", key='fixed_seed')
    st.number_input("Seed", min_value=0, step=1, key='seed', disabled=not fixed_seed)

    submitted = st.form_submit_button("🚀 Run Mission")

t1_val, t2_val, switch_val = mission_setup(mission_mode, steps)
if not hover_ff and ki > 0 and 0 < i_zone < 9.81 / max(kp, 1e-9):
    st.sidebar.warning(
        f"Feed-forward is off: the drone hovers about 9.81/Kp = {9.81 / max(kp, 1e-9):.1f} m "
        f"below target, outside the {i_zone:g} m integral zone, so the I term can't "
        "remove that offset. Raise the zone, raise Kp, or enable feed-forward.")
run_seed = st.session_state['seed'] if fixed_seed else -1

st.sidebar.divider()
st.sidebar.subheader("🤖 AI Auto-Tuner",
                     help="Coordinate Descent (Twiddle) starting from the current gains, "
                          "using the current noise σ, filter and feed-forward settings.")
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
        df = run_sim(kp, ki, kd, steps, t1_val, t2_val, switch_val,
                     noise_sigma, filter_n, run_seed, hover_ff, i_zone)
        if df is None:
            st.error("Simulation failed or telemetry file missing.")
            st.stop()

        rmse, overshoot, settling_time = calculate_metrics(df, switch_val, mission_mode)
        hold_rms, chatter = noise_metrics(df)

        col1, col2, col3, col4 = st.columns(4)
        col1.metric("Settling Time (±2%)",
                    "not settled" if settling_time == float('inf') else f"{settling_time:.1f} s",
                    help="Time after the step until the TRUE altitude stays within "
                         "±2% of the step size.")
        col2.metric("Overshoot", f"{overshoot:.1f} %", help="Relative to the step size.")
        col3.metric("RMSE (true altitude)", f"{rmse:.2f} m")
        col4.metric("Thrust chatter", f"{chatter:.2f} N",
                    help="Std. dev. of the step-to-step thrust change while holding "
                         "altitude (final 40% of the flight). Mostly sensor noise "
                         "amplified by Kp and especially Kd.")

        y_max = max(max(t1_val, t2_val) * 1.2, df['Actual'].max() + 10)

        fig = make_subplots(
            rows=1, cols=2,
            column_widths=[0.2, 0.8],
            subplot_titles=("Drone View", mission_mode),
            specs=[[{"type": "xy"}, {"type": "xy"}]]
        )

        fig.add_trace(go.Scatter(
            x=[-0.5, 0.5], y=[t2_val, t2_val],
            mode='lines', line=dict(color='red', dash='dash'), name='Target',
            showlegend=False
        ), row=1, col=1)

        initial_alt = df['Actual'][0]
        fig.add_trace(go.Scatter(
            x=[0], y=[initial_alt], mode='text',
            text=[f"🚁<br><b>{initial_alt:.1f} m</b>"],
            textposition="bottom center",
            textfont=dict(size=18, color="black"), name='Drone', showlegend=False
        ), row=1, col=1)

        fig.add_trace(go.Scatter(
            x=df['Time'], y=df['Target'],
            mode='lines', line=dict(color='red', dash='dash'), name='Target'
        ), row=1, col=2)
        fig.add_trace(go.Scatter(
            x=df['Time'], y=df['Actual'],
            mode='lines', line=dict(color='blue', width=2), name='True altitude'
        ), row=1, col=2)
        fig.add_trace(go.Scatter(
            x=df['Time'], y=df['Measured'],
            mode='lines', line=dict(color='gray', width=1), opacity=0.4,
            name='Sensor reading'
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

        st.plotly_chart(fig, width='stretch')

        fig_thrust = go.Figure(go.Scatter(
            x=df['Time'], y=df['Output'], mode='lines',
            line=dict(color=SERIES_COLORS[0], width=1.5), name='Thrust'
        ))
        fig_thrust.add_hline(y=9.81, line_dash="dot", line_color=TARGET_COLOR,
                             annotation_text="hover (9.81 N)",
                             annotation_position="top left")
        fig_thrust.update_layout(
            title="Motor Thrust", xaxis_title="Time (s)", yaxis_title="Thrust (N)",
            yaxis=dict(range=[-2, 52]), template="plotly_white", height=260,
            hovermode="x unified", margin=dict(t=40, b=40)
        )
        st.plotly_chart(fig_thrust, width='stretch')

        st.divider()
        csv_data = df.to_csv(index=False).encode('utf-8')
        st.download_button("📥 Download Telemetry CSV", csv_data, "flight_data.csv", "text/csv")


# ============================================================
# TAB 2: NOISE ANALYSIS
# ============================================================
with tab_noise:
    st.header("Noise Sensitivity Analysis")
    st.markdown(f"""
    How much does altimeter noise hurt your controller, and what can you do about it?

    The sweep flies the **current sidebar mission and gains**
    (Kp = {kp:.2f}, Ki = {ki:.3f}, Kd = {kd:.2f}) at several noise levels σ,
    several times each with different noise seeds, and compares
    **derivative-filter settings N** side by side. All metrics use the drone's
    **true** altitude, so they show what the noise does to the flight rather
    than how noisy the sensor itself is.

    - **Altitude-hold error**: RMS of the true altitude error during the final 40% of the
      flight, while the drone should be holding altitude. Noise enters the loop through the
      measurement and turns into real position wander.
    - **Thrust chatter**: standard deviation of the step-to-step thrust change over the same
      hold window. The D term
      differentiates the noisy measurement, so it multiplies the noise by roughly Kd/dt.
      That shows up first as motor wear, heat and vibration.
    - **Settling time**: how long the step takes to settle within ±2% (runs that never
      settle are counted at the full remaining flight time).
    """)

    with st.expander("Sweep Configuration", expanded=True):
        nc1, nc2, nc3 = st.columns(3)
        with nc1:
            sweep_sigmas = st.multiselect(
                "Noise levels σ (m)", [0.0, 0.1, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0],
                default=[0.0, 0.1, 0.25, 0.5, 1.0, 2.0], key='sweep_sigmas')
        with nc2:
            default_ns = sorted({2.0, float(filter_n), 20.0})
            sweep_ns = st.multiselect(
                "Derivative filter N to compare (max 3)", FILTER_N_OPTIONS,
                default=default_ns, max_selections=3, key='sweep_ns')
        with nc3:
            sweep_runs = st.slider("Runs per noise level", 1, 20, 8, key='sweep_runs',
                                   help="Different noise seeds per level; the chart "
                                        "shows the mean and ±1 std. dev.")

    if st.button("🔬 Run Noise Sweep", type="primary"):
        sigmas = sorted(sweep_sigmas)
        ns = sorted(sweep_ns)
        if not sigmas or not ns:
            st.warning("Select at least one noise level and one filter setting.")
            st.stop()

        records = []
        examples = {}   # (N, sigma) -> telemetry of seed 0, for the detail plot
        total = len(sigmas) * len(ns) * sweep_runs
        done = 0
        progress = st.progress(0, text="Running simulations...")
        seg_start = switch_val if mission_mode == "Step Response" else 0
        remaining_time = (steps - seg_start) * DT
        for n_val in ns:
            for sigma in sigmas:
                for seed in range(sweep_runs):
                    # Same seeds across N values and noise levels (common random
                    # numbers): differences come from the settings, not luck.
                    df_s = run_sim(kp, ki, kd, steps, t1_val, t2_val, switch_val,
                                   sigma, n_val, seed, hover_ff, i_zone)
                    done += 1
                    progress.progress(done / total,
                                      text=f"N = {n_val:g}, σ = {sigma} m  ({done}/{total})")
                    if df_s is None:
                        continue
                    _, _, ts = calculate_metrics(df_s, switch_val, mission_mode)
                    hold_rms, chatter = noise_metrics(df_s)
                    records.append(dict(
                        N=n_val, sigma=sigma, hold_rms=hold_rms, chatter=chatter,
                        settling=min(ts, remaining_time), settled=ts != float('inf')))
                    if seed == 0:
                        examples[(n_val, sigma)] = df_s
        progress.empty()

        if not records:
            st.error("All simulations failed.")
            st.stop()

        res = pd.DataFrame(records)
        summary = res.groupby(['N', 'sigma']).agg(
            hold_mean=('hold_rms', 'mean'), hold_std=('hold_rms', 'std'),
            chatter_mean=('chatter', 'mean'), chatter_std=('chatter', 'std'),
            settle_mean=('settling', 'mean'), settle_std=('settling', 'std'),
            settled_pct=('settled', lambda s: 100.0 * s.mean()),
        ).reset_index().fillna(0.0)
        st.session_state['noise_sweep'] = dict(summary=summary, examples=examples,
                                               ns=ns, sigmas=sigmas)

    sweep = st.session_state.get('noise_sweep')
    if sweep:
        summary, examples = sweep['summary'], sweep['examples']
        ns, sigmas = sweep['ns'], sweep['sigmas']
        color_of = {n: SERIES_COLORS[i] for i, n in enumerate(ns)}

        def metric_chart(col, std_col, title, y_title):
            fig = go.Figure()
            for n_val in ns:
                d = summary[summary['N'] == n_val]
                fig.add_trace(go.Scatter(
                    x=d['sigma'], y=d[col],
                    error_y=dict(type='data', array=d[std_col], thickness=1.5, width=4),
                    mode='lines+markers', name=f"N = {n_val:g}",
                    line=dict(color=color_of[n_val], width=2), marker=dict(size=8),
                    hovertemplate=f"N = {n_val:g}<br>σ = %{{x}} m<br>{y_title}: "
                                  "%{y:.2f}<extra></extra>"))
            fig.update_layout(title=title, xaxis_title="Noise σ (m)", yaxis_title=y_title,
                              template="plotly_white", height=340,
                              legend=dict(orientation="h", y=-0.25),
                              margin=dict(t=50, b=40))
            fig.update_yaxes(rangemode="tozero")
            return fig

        c1, c2 = st.columns(2)
        c1.plotly_chart(metric_chart('hold_mean', 'hold_std',
                                     "Altitude-hold error vs noise", "Hold RMS error (m)"),
                        width='stretch')
        c2.plotly_chart(metric_chart('chatter_mean', 'chatter_std',
                                     "Thrust chatter vs noise", "Thrust chatter (N)"),
                        width='stretch')
        st.plotly_chart(metric_chart('settle_mean', 'settle_std',
                                     "Settling time vs noise", "Settling time (s)"),
                        width='stretch')

        with st.expander("Results table"):
            st.dataframe(summary.rename(columns={
                'sigma': 'σ (m)', 'hold_mean': 'hold RMS (m)', 'hold_std': '± hold',
                'chatter_mean': 'chatter (N)', 'chatter_std': '± chatter',
                'settle_mean': 'settling (s)', 'settle_std': '± settling',
                'settled_pct': 'settled (%)'}).round(3), width='stretch')

        st.subheader("Inspect a single flight")
        dc1, dc2 = st.columns(2)
        with dc1:
            sel_sigma = st.select_slider("Noise σ (m)", sigmas, value=sigmas[-1])
        with dc2:
            sel_n = st.radio("Filter N", ns, horizontal=True,
                             format_func=lambda v: f"{v:g}")
        df_d = examples.get((sel_n, sel_sigma))
        if df_d is not None:
            fig_d = make_subplots(rows=2, cols=1, shared_xaxes=True,
                                  row_heights=[0.6, 0.4], vertical_spacing=0.08,
                                  subplot_titles=("Altitude", "Motor thrust"))
            fig_d.add_trace(go.Scatter(x=df_d['Time'], y=df_d['Measured'], mode='lines',
                                       name='Sensor reading', opacity=0.5,
                                       line=dict(color='#9a9993', width=1)), row=1, col=1)
            fig_d.add_trace(go.Scatter(x=df_d['Time'], y=df_d['Actual'], mode='lines',
                                       name='True altitude',
                                       line=dict(color=color_of[sel_n], width=2)),
                            row=1, col=1)
            fig_d.add_trace(go.Scatter(x=df_d['Time'], y=df_d['Target'], mode='lines',
                                       name='Target',
                                       line=dict(color=TARGET_COLOR, dash='dash', width=1.5)),
                            row=1, col=1)
            fig_d.add_trace(go.Scatter(x=df_d['Time'], y=df_d['Output'], mode='lines',
                                       name='Thrust',
                                       line=dict(color=color_of[sel_n], width=1)),
                            row=2, col=1)
            fig_d.update_yaxes(title_text="m", row=1, col=1)
            fig_d.update_yaxes(title_text="N", range=[-2, 52], row=2, col=1)
            fig_d.update_xaxes(title_text="Time (s)", row=2, col=1)
            fig_d.update_layout(template="plotly_white", height=520,
                                hovermode="x unified")
            st.plotly_chart(fig_d, width='stretch')

        st.info(
            "**How to read this:** with an ideal sensor (σ = 0) the filter setting barely "
            "matters. As σ grows, a high N (light filtering) lets the D term pass the "
            "noise straight to the motors, so thrust chatter and position wander rise "
            "steeply. A low N smooths the derivative but adds lag, which costs some damping "
            "(more overshoot or slower settling on a clean sensor). Lowering Kd has a "
            "similar trade-off. Pick the setting that keeps the hold error and chatter low "
            "for the noise level your sensor actually has."
        )
