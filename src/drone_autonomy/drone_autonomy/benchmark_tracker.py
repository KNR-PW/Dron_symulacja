#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import csv
import time
import os

class TrackerBenchmark(Node):
    def __init__(self):
        super().__init__('tracker_benchmark')
        
        # PARAMETER: Name prefix for output file
        self.declare_parameter('name', 'run')
        self.name_param = self.get_parameter('name').value

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/debug/tracker_benchmark',
            self.listener_callback,
            10)
        self.errors = []
        self.start_time = time.time()
        
        # PARAMETER: Number of samples (default 2000)
        self.declare_parameter('samples', 2000)
        self.target_samples = self.get_parameter('samples').value
        
        # Ensure 'benchmarks' directory exists
        if not os.path.exists('benchmarks'):
            os.makedirs('benchmarks')
            
        timestamp_str = time.strftime("%Y%m%d-%H%M%S")
        self.filename = f"benchmarks/{self.name_param}_{timestamp_str}.csv"
        
        # Get absolute path for easier copying
        self.abs_filepath = os.path.abspath(self.filename)
        
        # Prepare CSV file
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'true_x', 'true_y', 'pred_x', 'pred_y', 'dist_error'])
        
        self.get_logger().info(f"Benchmark Node Started.")
        self.get_logger().info(f"Subscribed to: /debug/tracker_benchmark")
        self.get_logger().info(f"Auto-stop set to: {self.target_samples} samples")
        self.get_logger().info(f"Saving to: {self.abs_filepath}")
        self.get_logger().info("Recording... (Press Ctrl+C to stop early)")

    def listener_callback(self, msg):
        now = time.time()
        # [true_x, true_y, pred_x, pred_y, dist_error]
        data = msg.data
        if len(data) < 5:
             return

        true_x = data[0]
        true_y = data[1]
        pred_x = data[2]
        pred_y = data[3]
        dist_error = data[4]
        
        self.errors.append(dist_error)
        
        with open(self.filename, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([now, true_x, true_y, pred_x, pred_y, dist_error])
            
        # Check auto-stop condition based on SAMPLES
        if len(self.errors) >= self.target_samples:
            self.get_logger().info(f"Sample limit ({self.target_samples}) reached.")
            self.report()
            raise SystemExit

        # Optional: Print live stats every 50 samples
        if len(self.errors) % 50 == 0:
            current_mean = np.mean(self.errors)
            self.get_logger().info(f"Collected {len(self.errors)}/{self.target_samples}. Mean Err: {current_mean:.3f}m")

    def report(self):
        if not self.errors:
            self.get_logger().warn("No data collected.")
            return
            
        params = np.array(self.errors)
        rmse = np.sqrt(np.mean(params**2))
        mae = np.mean(np.abs(params))
        max_err = np.max(params)
        # mean_err = np.mean(params) # Bias?
        
        print("\n" + "="*40)
        print(f" BENCHMARK REPORT: {self.filename}")
        print("="*40)
        print(f" Samples:     {len(params)}")
        print(f" Duration:    {time.time() - self.start_time:.1f} s")
        print("-" * 40)
        print(f" RMSE:        {rmse:.4f} m  <-- Root Mean Square Error")
        print(f" MAE:         {mae:.4f} m   <-- Mean Absolute Error")
        print(f" Max Error:   {max_err:.4f} m")
        # print(f" Mean Bias:   {mean_err:.4f} m")
        print("="*40 + "\n")

def main(args=None):
    rclpy.init(args=args)
    bench = TrackerBenchmark()
    try:
        rclpy.spin(bench)
    except SystemExit:
        # Handled inside callback
        pass
    except KeyboardInterrupt:
        bench.report()
    finally:
        bench.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
