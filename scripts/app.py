import streamlit as st
import pandas as pd
import subprocess
import plotly.graph_objects as go
import time
import os

# Page Config
st.set_page_config(page_title="AeroStream GCS", layout="wide", page_icon="🚁")

st.title("🚁 AeroStream: Real-Time Flight Control")
st.markdown("Adjust PID gains and run the simulation to verify the transient response.")

# --- SIDEBAR: CONTROLLER INPUTS ---
st.sidebar.header("🕹️ PID Parameters")
with st.sidebar.form("pid_form"):
    kp = st.slider("Proportional (Kp)", 0.0, 5.0, 0.6, 0.1)
    ki = st.slider("Integral (Ki)", 0.0, 1.0, 0.01, 0.01)
    kd = st.slider("Derivative (Kd)", 0.0, 1.0, 0.05, 0.01)
    
    # We use a form button so the animation only starts when you click "Launch"
    submitted = st.form_submit_button("🚀 Launch Simulation")

# --- MAIN LOGIC ---
if submitted:
    # DEFINITIONS
    build_dir = "../build"
    # We run the command AS IF we are inside the build folder
    # So we just call "./flight_controller" (Linux/Mac) or "flight_controller.exe" (Windows)
    exe_name = "./flight_controller" 
    
    # 1. Run the C++ Simulation
    # cwd=build_dir tells Python: "Go into ../build, run the command, then come back"
    try:
        subprocess.run([exe_name, str(kp), str(ki), str(kd)], cwd=build_dir, check=True)
    except Exception as e:
        st.error(f"Failed to run C++ Executable: {e}")
        st.stop()

    # 2. Load Data
    # Now we are 100% sure we are reading the file we just created
    csv_path = os.path.join(build_dir, "telemetry.csv")
    
    if os.path.exists(csv_path):
        # Add a timestamp column or random parameter to ensure Streamlit doesn't cache the CSV
        df = pd.read_csv(csv_path)
    else:
        st.error("No telemetry data found. Did the C++ simulation run?")
        st.stop()

    # 3. Setup the Layout Placeholders
    col1, col2 = st.columns([1, 2]) # Split screen: 1/3 drone view, 2/3 graphs

    with col1:
        st.subheader("Drone View")
        drone_placeholder = st.empty()
    
    with col2:
        st.subheader("Live Telemetry")
        chart_placeholder = st.empty()

# 4. OPTIMIZED ANIMATION LOOP
    
    # Calculate a step size to prevent "white flashing"
    # We want to render roughly 50-100 frames TOTAL, regardless of how many rows exist.
    total_rows = len(df)
    target_frames = 100 
    step_size = max(1, total_rows // target_frames) # Ensure at least 1 step
    
    # Pre-define fixed ranges to stop axis jitter
    y_max = max(df['Actual'].max(), df['Target'].max()) + 10
    y_min = min(df['Actual'].min(), -10)
    
    # Progress Bar (Optional, looks professional)
    progress_bar = st.progress(0)
    
    for i in range(0, total_rows, step_size):
        row = df.iloc[i]
        
        # Update Progress
        progress = min(i / total_rows, 1.0)
        progress_bar.progress(progress)

        # --- A. DRONE VISUALIZATION ---
        fig_drone = go.Figure()
        
        # Drone (Blue Cross)
        fig_drone.add_trace(go.Scatter(
            x=[0], y=[row['Actual']],
            mode='markers+text',
            marker=dict(size=20, color='blue', symbol='x'),
            name='Drone'
        ))
        
        # Target (Red Line)
        fig_drone.add_shape(
            type="line", x0=-1, x1=1, 
            y0=row['Target'], y1=row['Target'], # Use row['Target'] in case you make it moving later
            line=dict(color="Red", width=4, dash="dash")
        )

        fig_drone.update_layout(
            yaxis_range=[y_min, y_max],  # Fixed Y-Axis
            xaxis_range=[-0.5, 0.5],
            xaxis=dict(visible=False),
            yaxis=dict(title="Altitude (m)"),
            height=400,
            margin=dict(l=20, r=20, t=30, b=20),
            title=f"Alt: {row['Actual']:.1f} m"
        )
        # Unique key prevents duplicate ID error
        drone_placeholder.plotly_chart(fig_drone, use_container_width=True, key=f"d_{i}")

        # --- B. LIVE GRAPH ---
        # Slice data up to current frame
        # Optimization: Don't plot every single point in the history if it's huge
        current_data = df.iloc[:i+1:step_size] 
        
        fig_chart = go.Figure()
        fig_chart.add_trace(go.Scatter(x=current_data['Time'], y=current_data['Target'], name='Target', line=dict(color='red', dash='dash')))
        fig_chart.add_trace(go.Scatter(x=current_data['Time'], y=current_data['Actual'], name='Actual', line=dict(color='blue')))
        
        fig_chart.update_layout(
            yaxis_range=[y_min, y_max], # Lock Y-axis so it doesn't jump
            xaxis_range=[0, df['Time'].max()], # Lock X-axis to total time
            yaxis_title="Altitude (m)",
            xaxis_title="Time (s)",
            height=400,
            margin=dict(l=0, r=0, t=30, b=0),
            title="Time Domain Response"
        )
        # Unique key prevents duplicate ID error
        chart_placeholder.plotly_chart(fig_chart, use_container_width=True, key=f"c_{i}")

        # Small sleep to allow browser to render the frame
        time.sleep(0.05) 

    # Final "Done" state
    progress_bar.progress(100)
    st.success("Simulation Replay Complete.")
    
    # Download Button
    csv = df.to_csv(index=False).encode('utf-8')
    st.download_button("📥 Download Flight Logs", csv, "telemetry.csv", "text/csv")