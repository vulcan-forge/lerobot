#!/usr/bin/env python
"""
Quick test script to verify daxie package access and get robot file paths.
Run this first to make sure everything is accessible.
"""

def test_daxie_access():
    """Test if we can access the daxie package and get robot paths."""
    
    print("Testing daxie package access...")
    
    try:
        # Test basic daxie import
        import daxie
        print("✓ daxie package imported successfully")
        
        # Test getting SO100 paths
        from daxie import get_so100_path
        urdf_path, mesh_path = get_so100_path()
        print(f"✓ URDF path: {urdf_path}")
        print(f"✓ Mesh path: {mesh_path}")
        
        # Verify files exist
        import os
        if os.path.exists(urdf_path):
            print("✓ URDF file exists")
        else:
            print("✗ URDF file not found!")
            
        if os.path.exists(mesh_path):
            print("✓ Mesh directory exists")
        else:
            print("✗ Mesh directory not found!")
            
        return urdf_path, mesh_path
        
    except ImportError as e:
        print(f"✗ Failed to import daxie: {e}")
        return None, None
    except Exception as e:
        print(f"✗ Error accessing robot paths: {e}")
        return None, None

def test_grpc_server():
    """Test if we can import the gRPC server components."""
    
    print("\nTesting gRPC server access...")
    
    try:
        from daxie.src.server.pos_grpc_server import start_grpc_server
        print("✓ gRPC server import successful")
        return True
    except ImportError as e:
        print(f"✗ Failed to import gRPC server: {e}")
        return False

def test_ik_solver():
    """Test if we can import the IK solver."""
    
    print("\nTesting IK solver access...")
    
    try:
        from daxie.src.teleop.solve_ik import solve_ik
        print("✓ IK solver import successful")
        return True
    except ImportError as e:
        print(f"✗ Failed to import IK solver: {e}")
        return False

def test_dependencies():
    """Test if all required dependencies are installed."""
    
    print("\nTesting dependencies...")
    
    dependencies = {
        'pyroki': 'pyroki',
        'viser': 'viser', 
        'yourdfpy': 'yourdfpy',
        'numpy': 'numpy',
        'scipy': 'scipy',
        'torch': 'torch'
    }
    
    missing = []
    for name, import_name in dependencies.items():
        try:
            __import__(import_name)
            print(f"✓ {name} available")
        except ImportError:
            print(f"✗ {name} missing")
            missing.append(name)
    
    if missing:
        print(f"\nInstall missing dependencies with:")
        print(f"pip install {' '.join(missing)}")
        return False
    
    return True

if __name__ == "__main__":
    print("=== Phone Teleoperation Setup Test ===\n")
    
    # Test dependencies
    deps_ok = test_dependencies()
    
    # Test daxie access
    urdf_path, mesh_path = test_daxie_access()
    
    # Test gRPC server
    grpc_ok = test_grpc_server()
    
    # Test IK solver
    ik_ok = test_ik_solver()
    
    print("\n=== Summary ===")
    if deps_ok and urdf_path and grpc_ok and ik_ok:
        print("✓ All tests passed! Ready to test phone teleoperation.")
        print(f"\nNext steps:")
        print(f"1. Update the example script with these paths:")
        print(f"   urdf_path='{urdf_path}'")
        print(f"   mesh_path='{mesh_path}'")
        print(f"2. Connect your SO100 robot")
        print(f"3. Run the phone teleoperation example")
    else:
        print("✗ Some tests failed. Fix the issues above before proceeding.") 