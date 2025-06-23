#!/usr/bin/env python3
"""
Identity Motor Converter

Based on ground truth analysis, the old system sends IK solutions
DIRECTLY to motors without any conversion. This converter does the same.
"""

class IdentityMotorConverter:
    """
    Identity converter that passes motor commands through unchanged.
    
    This is based on the discovery that the old system sends IK solutions
    directly to motors without any normalization conversion.
    """
    
    def __init__(self, calibration_file_path=None):
        """Initialize identity converter (calibration file not needed)."""
        print("🔄 IDENTITY CONVERTER: Initialized")
        print("   → Will pass all motor commands through unchanged")
        print("   → No calibration conversion needed")
    
    def convert_motor_dict(self, goal_pos):
        """
        Convert motor positions using identity function.
        
        Args:
            goal_pos: Dictionary of motor positions {motor_name: angle_degrees}
            
        Returns:
            Dictionary with identical values (no conversion)
        """
        # Identity function: output = input (no conversion needed)
        converted = goal_pos.copy()
        return converted
    
    def new_to_old_normalization(self, positions, motor_names):
        """
        Convert list of positions using identity function.
        
        Args:
            positions: List of motor positions in degrees
            motor_names: List of motor names
            
        Returns:
            List with identical values (no conversion)
        """
        # Identity function: output = input (no conversion needed)
        converted = positions.copy()
        return converted


def test_identity_converter():
    """Test the identity converter with ground truth data."""
    
    print("🧪 TESTING IDENTITY CONVERTER")
    print("=" * 60)
    
    # Ground truth data
    ik_solution = [0.0, 156.622482, 138.169525, 18.452971, -89.999779, 48.701412]
    expected_output = [0.0, 156.622482, 138.169525, 18.452971, -89.999779, 48.701412]
    motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    
    # Test converter
    converter = IdentityMotorConverter()
    result = converter.new_to_old_normalization(ik_solution, motor_names)
    
    # Check results
    differences = [r - e for r, e in zip(result, expected_output)]
    max_diff = max(abs(d) for d in differences)
    
    print(f"\n📊 RESULTS:")
    print(f"Input:      {ik_solution}")
    print(f"Expected:   {expected_output}")
    print(f"Output:     {result}")
    print(f"Differences: {differences}")
    print(f"Max Diff:   {max_diff:.6f}°")
    
    if max_diff < 0.001:
        print("✅ PERFECT MATCH!")
    else:
        print(f"❌ ERROR: {max_diff:.3f}° difference")
    
    # Test dict interface
    print(f"\n🧪 TESTING DICT INTERFACE:")
    goal_dict = {name: pos for name, pos in zip(motor_names, ik_solution)}
    result_dict = converter.convert_motor_dict(goal_dict)
    
    print(f"Input dict:  {goal_dict}")
    print(f"Output dict: {result_dict}")
    
    return converter


if __name__ == "__main__":
    test_identity_converter() 