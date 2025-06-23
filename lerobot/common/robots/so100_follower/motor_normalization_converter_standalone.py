#!/usr/bin/env python3
"""
Standalone Motor Normalization Converter

Converts between old and new motor normalization systems using only the new calibration file.
This version uses the ACTUAL homing offsets from the calibration file.
"""

import json
from typing import List, Dict, Any


class StandaloneMotorNormalizationConverter:
    """
    Converts motor positions between old and new normalization systems.
    
    Old system: degrees = (raw_value + homing_offset) / (resolution//2) * 180
    New system: degrees = (raw_value - mid) * 360 / (resolution-1)
    
    This version uses the actual homing offsets from the new calibration file.
    """
    
    def __init__(self, new_calib_path: str):
        """Initialize converter with new calibration file."""
        self.new_calib_path = new_calib_path
        self.resolution = 4096  # Standard for STS3215 motors
        
        # Load new calibration data
        with open(new_calib_path, 'r') as f:
            self.new_calib_data = json.load(f)
        
        # Derive old system parameters using ACTUAL homing offsets
        self.derived_old_parameters = self._derive_old_parameters()
        
        print("🔧 FIXED CONVERTER: Using actual homing offsets from calibration file")
        self._print_conversion_parameters()
    
    def _derive_old_parameters(self) -> Dict[str, Dict[str, Any]]:
        """Derive old system parameters using ACTUAL homing offsets."""
        derived_params = {}
        
        for motor_name, motor_data in self.new_calib_data.items():
            # New system parameters
            new_range_min = motor_data['range_min']
            new_range_max = motor_data['range_max']
            new_mid = (new_range_min + new_range_max) / 2
            new_max_res = self.resolution - 1
            
            # Use ACTUAL homing offset from calibration file
            actual_homing_offset = motor_data['homing_offset']
            
            derived_params[motor_name] = {
                'new_range_min': new_range_min,
                'new_range_max': new_range_max,
                'new_mid': new_mid,
                'new_max_res': new_max_res,
                'old_homing_offset': actual_homing_offset,  # Use actual value!
                'resolution': self.resolution
            }
        
        return derived_params
    
    def _print_conversion_parameters(self):
        """Print the conversion parameters for debugging."""
        print("\n📋 CONVERSION PARAMETERS (FIXED):")
        print("=" * 60)
        
        for motor_name, params in self.derived_old_parameters.items():
            print(f"\n🔧 {motor_name.upper()}:")
            print(f"  New system mid: {params['new_mid']:.1f}")
            print(f"  Actual homing offset: {params['old_homing_offset']}")
            print(f"  Range: {params['new_range_min']} to {params['new_range_max']}")
    
    def new_to_old_normalization(self, joint_angles: List[float], motor_names: List[str]) -> List[float]:
        """
        Convert joint angles from new normalization to old normalization.
        
        Args:
            joint_angles: List of joint angles in degrees (new system)
            motor_names: List of motor names corresponding to joint_angles
            
        Returns:
            List of joint angles in degrees (old system)
        """
        converted_angles = []
        
        for i, (angle, motor_name) in enumerate(zip(joint_angles, motor_names)):
            if motor_name in self.derived_old_parameters:
                params = self.derived_old_parameters[motor_name]
                
                # Step 1: Convert new system degrees to raw value
                # new_degrees = (raw - mid) * 360 / (resolution-1)
                # raw = (new_degrees * (resolution-1) / 360) + mid
                raw_value = (angle * params['new_max_res'] / 360) + params['new_mid']
                
                # Step 2: Convert raw value to old system degrees
                # old_degrees = (raw + homing_offset) / (resolution//2) * 180
                old_angle = (raw_value + params['old_homing_offset']) / (params['resolution'] // 2) * 180
                
                converted_angles.append(old_angle)
                
                # Debug output
                print(f"    🔧 {motor_name}: {angle:.2f}° -> raw {raw_value:.1f} -> {old_angle:.2f}°")
            else:
                # Motor not found in calibration, pass through unchanged
                converted_angles.append(angle)
                print(f"    ⚠️ {motor_name}: {angle:.2f}° (unchanged - not in calibration)")
        
        return converted_angles
    
    def old_to_new_normalization(self, joint_angles: List[float], motor_names: List[str]) -> List[float]:
        """
        Convert joint angles from old normalization to new normalization.
        
        Args:
            joint_angles: List of joint angles in degrees (old system)
            motor_names: List of motor names corresponding to joint_angles
            
        Returns:
            List of joint angles in degrees (new system)
        """
        converted_angles = []
        
        for i, (angle, motor_name) in enumerate(zip(joint_angles, motor_names)):
            if motor_name in self.derived_old_parameters:
                params = self.derived_old_parameters[motor_name]
                
                # Step 1: Convert old system degrees to raw value
                # old_degrees = (raw + homing_offset) / (resolution//2) * 180
                # raw = (old_degrees * (resolution//2) / 180) - homing_offset
                raw_value = (angle * (params['resolution'] // 2) / 180) - params['old_homing_offset']
                
                # Step 2: Convert raw value to new system degrees
                # new_degrees = (raw - mid) * 360 / (resolution-1)
                new_angle = (raw_value - params['new_mid']) * 360 / params['new_max_res']
                
                converted_angles.append(new_angle)
                
                # Debug output
                print(f"    🔧 {motor_name}: {angle:.2f}° -> raw {raw_value:.1f} -> {new_angle:.2f}°")
            else:
                # Motor not found in calibration, pass through unchanged
                converted_angles.append(angle)
                print(f"    ⚠️ {motor_name}: {angle:.2f}° (unchanged - not in calibration)")
        
        return converted_angles
    
    def convert_motor_dict(self, motor_dict: Dict[str, float]) -> Dict[str, float]:
        """
        Convert a dictionary of motor positions from new to old normalization.
        This is the main interface for the SO100 follower integration.
        
        Args:
            motor_dict: Dictionary with motor names as keys and angles as values
            
        Returns:
            Dictionary with converted angles
        """
        motor_names = list(motor_dict.keys())
        joint_angles = list(motor_dict.values())
        
        converted_angles = self.new_to_old_normalization(joint_angles, motor_names)
        
        return dict(zip(motor_names, converted_angles)) 