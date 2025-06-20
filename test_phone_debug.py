#!/usr/bin/env python

"""
Debug script to test phone connection and start signal reception.
This helps troubleshoot gRPC communication issues.
"""

import time
from pathlib import Path

def test_grpc_connection_debug():
    """Test gRPC server and pose reception with detailed logging."""
    
    print("=== Phone Connection Debug Test ===")
    
    try:
        from daxie.src.server.pos_grpc_server import start_grpc_server
        print("✓ Successfully imported gRPC server")
    except ImportError as e:
        print(f"✗ Failed to import gRPC server: {e}")
        return
    
    # Test different ports to see which works
    test_ports = [50055, 8765, 50051]
    
    for port in test_ports:
        print(f"\n--- Testing gRPC server on port {port} ---")
        
        try:
            # Start gRPC server
            server, pose_service = start_grpc_server(port=port, debug=True)
            print(f"✓ gRPC server started successfully on port {port}")
            
            # Set debug mode
            pose_service.set_debug(True)
            
            # Wait for phone connection with timeout
            print("Waiting for phone connection...")
            print("Please connect your phone app to this server")
            print(f"Server address: localhost:{port} (or your computer's IP:{port})")
            
            timeout = 30  # 30 second timeout
            start_time = time.time()
            connected = False
            
            while (time.time() - start_time) < timeout:
                try:
                    # Try to get pose data (non-blocking)
                    pose_data = pose_service.get_latest_pose(block=False)
                    
                    if pose_data is not None:
                        if not connected:
                            print("✓ Phone connected! Receiving pose data:")
                            connected = True
                        
                        # Print detailed pose information
                        print(f"Position: {pose_data['position']}")
                        print(f"Rotation: {pose_data['rotation']}")
                        print(f"Gripper: {pose_data['gripper_open']}")
                        print(f"Start Switch: {pose_data['switch']} ←← THIS IS THE START SIGNAL")
                        print(f"Precision: {pose_data['precision']}")
                        print(f"Resetting: {pose_data['is_resetting']}")
                        print("-" * 50)
                        
                        # Check if start signal is working
                        if pose_data['switch']:
                            print("🎉 START SIGNAL DETECTED! Phone is ready for teleoperation")
                        else:
                            print("⏳ Start signal is False - waiting for user to activate")
                    else:
                        print(".", end="", flush=True)  # Show we're waiting
                    
                    time.sleep(0.5)  # Check every 500ms
                    
                except Exception as e:
                    print(f"Error getting pose data: {e}")
                    time.sleep(1)
            
            if not connected:
                print(f"\n✗ No phone connection received within {timeout} seconds on port {port}")
            
            # Stop server
            server.stop(0)
            print(f"\nStopped server on port {port}")
            
            if connected:
                print(f"✓ Port {port} works! Use this port in your configuration.")
                return  # Exit after first successful connection
                
        except Exception as e:
            print(f"✗ Failed to start server on port {port}: {e}")
    
    print("\n=== Debug Summary ===")
    print("If no ports worked:")
    print("1. Check that your phone app is configured with the correct server IP and port")
    print("2. Ensure your firewall allows connections on the tested ports")
    print("3. Make sure your phone and computer are on the same network")
    print("4. Try manually specifying your computer's IP address instead of localhost")


if __name__ == "__main__":
    test_grpc_connection_debug() 