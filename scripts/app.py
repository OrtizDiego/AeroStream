import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import os

# 1. PAGE CONFIG
st.set_page_config(page_title="AeroStream GCS", layout="wide", page_icon="🚁")
st.title("🚁 AeroStream: Flight Telemetry Replay")

# 2. SIDEBAR CONFIG
st.sidebar.header("🕹️ Simulation Settings")
with st.sidebar.form("pid_form"):
    st.subheader("PID Gains")
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, 0.6, 0.1)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, 0.01, 0.01)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, 0.05, 0.01)
    
    st.subheader("Duration")
    steps = st.slider("Simulation Steps", 50, 2000, 200, 50)
    
    submitted = st.form_submit_button("🚀 Calculate & Render")

# 3. MAIN LOGIC
if submitted:
    # --- A. RUN SIMULATION (C++) ---
    build_dir = "../build"
    exe_name = "./flight_controller" 
    
    try:
        # Pass 'steps' as the 4th argument
        subprocess.run([exe_name, str(kp), str(ki), str(kd), str(steps)], cwd=build_dir, check=True)
    except Exception as e:
        st.error(f"Error: {e}")
        st.stop()

    # --- B. LOAD DATA ---
    csv_path = os.path.join(build_dir, "telemetry.csv")
    if not os.path.exists(csv_path):
        st.error("Telemetry file missing.")
        st.stop()
        
    df = pd.read_csv(csv_path)

    # --- C. BUILD ANIMATION ---
    total_rows = len(df)
    max_frames = 100 
    step = max(1, total_rows // max_frames)
    
    # Layout: Drone View (20%) | Graph (80%)
    fig = make_subplots(
        rows=1, cols=2, 
        column_widths=[0.2, 0.8],
        subplot_titles=("Drone View", "Altitude Response"),
        specs=[[{"type": "xy"}, {"type": "xy"}]]
    )

    # 1. STATIC TRACES
    # Target Line (Left)
    fig.add_trace(go.Scatter(
        x=[-0.5, 0.5], y=[df['Target'][0], df['Target'][0]], 
        mode='lines', line=dict(color='red', dash='dash'), name='Target'
    ), row=1, col=1)

    # Drone Emoji (Left - Initial)
    fig.add_trace(go.Scatter(
        x=[0], y=[df['Actual'][0]], 
        mode='text', text=['🚁'], textfont=dict(size=40), name='Drone'
    ), row=1, col=1)

    # Ghost Path (Right - Full History)
    fig.add_trace(go.Scatter(
        x=df['Time'], y=df['Actual'], 
        mode='lines', line=dict(color='lightgrey'), showlegend=False
    ), row=1, col=2)

    # Live Trace (Right - Initial Empty)
    fig.add_trace(go.Scatter(
        x=[df['Time'][0]], y=[df['Actual'][0]], 
        mode='lines', line=dict(color='blue', width=2), name='Response'
    ), row=1, col=2)

    # 2. GENERATE FRAMES
    frames = []
    for i in range(0, total_rows, step):
        row = df.iloc[i]
        current_data = df.iloc[:i+1]
        
        frames.append(go.Frame(
            data=[
                # Update Drone (Trace 1)
                go.Scatter(x=[0], y=[row['Actual']], mode='text', text=['🚁'], textfont=dict(size=40)),
                # Update Live Line (Trace 3)
                go.Scatter(x=current_data['Time'], y=current_data['Actual'])
            ],
            traces=[1, 3],
            name=f"frame_{i}"
        ))

    fig.frames = frames

    # 3. LAYOUT & PLAY BUTTON
    fig.update_layout(
        height=500,
        hovermode="x unified",
        template="plotly_white",
        yaxis=dict(range=[-20, 150], title="Altitude (m)"),
        xaxis=dict(visible=False, range=[-1, 1]),
        yaxis2=dict(range=[-20, 150]),
        xaxis2=dict(title="Time (s)"),
        updatemenus=[{
            "type": "buttons",
            "showactive": False,
            "x": 1.05, "y": 0,
            "buttons": [{
                "label": "▶ Play Flight",
                "method": "animate",
                "args": [None, {"frame": {"duration": 20, "redraw": True}, "fromcurrent": True}]
            }]
        }]
    )

    st.plotly_chart(fig, use_container_width=True)
    
    # --- D. DOWNLOAD BUTTON ---
    st.divider()
    csv_data = df.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="📥 Download Telemetry CSV",
        data=csv_data,
        file_name="flight_data.csv",
        mime="text/csv",
    )