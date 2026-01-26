#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys
import os

def plot_benchmark(csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found.")
        return

    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return

    # Check required columns
    required_cols = ['timestamp', 'true_x', 'true_y', 'pred_x', 'pred_y', 'dist_error']
    if not all(col in df.columns for col in required_cols):
        print(f"Error: CSV missing required columns. Found: {df.columns}")
        return

    # Normalize time
    df['time_rel'] = df['timestamp'] - df['timestamp'].iloc[0]

    plt.figure(figsize=(14, 8))

    # --- Plot 1: Trajectory (X-Y) ---
    plt.subplot(2, 2, 1)
    plt.plot(df['true_x'], df['true_y'], 'g-', label='Ground Truth', linewidth=2, alpha=0.7)
    plt.plot(df['pred_x'], df['pred_y'], 'r--', label='Predicted (KF)', linewidth=1.5, alpha=0.7)
    plt.title('Car Trajectory (X-Y)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    # --- Plot 2: Error over Time ---
    plt.subplot(2, 2, 2)
    plt.plot(df['time_rel'], df['dist_error'], 'b-', label='Euclidean Error')
    plt.axhline(y=df['dist_error'].mean(), color='r', linestyle=':', label=f"Mean: {df['dist_error'].mean():.3f}m")
    plt.title('Tracking Error vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Error (m)')
    plt.legend()
    plt.grid(True)

    # --- Plot 3: X vs Time ---
    plt.subplot(2, 2, 3)
    plt.plot(df['time_rel'], df['true_x'], 'g-', label='True X', alpha=0.6)
    plt.plot(df['time_rel'], df['pred_x'], 'r--', label='Pred X', alpha=0.6)
    plt.title('X Coordinate vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    plt.legend()
    plt.grid(True)

    # --- Plot 4: Y vs Time ---
    plt.subplot(2, 2, 4)
    plt.plot(df['time_rel'], df['true_y'], 'g-', label='True Y', alpha=0.6)
    plt.plot(df['time_rel'], df['pred_y'], 'r--', label='Pred Y', alpha=0.6)
    plt.title('Y Coordinate vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    
    # Save plot
    out_file = csv_file.replace('.csv', '_plot.png')
    plt.savefig(out_file)
    print(f"Plot saved to: {out_file}")
    
    # Try to show, or fallback to opening the file
    try:
        import matplotlib
        if matplotlib.get_backend().lower() == 'agg':
            # Agg is non-interactive, open the file directly
            print("Non-interactive backend detected. Opening image via system viewer...")
            import subprocess
            subprocess.call(['xdg-open', out_file])
        else:
            plt.show()
    except Exception as e:
        print(f"Could not display plot interactively: {e}")
        print("Opening saved image...")
        import subprocess
        subprocess.call(['xdg-open', out_file])

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot tracker benchmark results.")
    parser.add_argument("csv_file", help="Path to the benchmark CSV file")
    args = parser.parse_args()

    plot_benchmark(args.csv_file)
