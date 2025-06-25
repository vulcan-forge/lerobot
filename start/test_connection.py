#!/usr/bin/env python3
"""
Simple test script to debug COM5 connection issues.
"""

import sys
sys.path.insert(0, '..')

import logging
logging.basicConfig(level=logging.DEBUG)

try:
    import scservo_sdk as scs
    print("scservo_sdk imported successfully")
    
    # Test basic port handler creation
    port_handler = scs.PortHandler("COM5")
    print(f"Port handler created for COM5")
    
    # Test opening the port
    result = port_handler.openPort()
    print(f"openPort() result: {result}")
    
    if result:
        print("Port opened successfully!")
        port_handler.closePort()
        print("Port closed successfully!")
    else:
        print("Failed to open port")
        
except ImportError as e:
    print(f"Failed to import scservo_sdk: {e}")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc() 