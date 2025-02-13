import time
import serial

try:
    # Open COM9 serial port
    ser = serial.Serial("COM9", baudrate=1000000, timeout=1)
    print("✅ COM9 opened successfully!")

    while True:  # Continuous monitoring loop
        try:
            # Send position read command - example for Dynamixel
            # Modify these commands based on your specific servo protocol
            command = bytes([
                0xFF, 0xFF,  # Header
                0xFE,        # Broadcast ID
                0x04,        # Length
                0x02,        # Instruction (Read)
                0x24,        # Position address
                0x02,        # Data length
                0xD9        # Checksum
            ])
            
            ser.write(command)
            time.sleep(0.1)  # Give servos time to respond
            
            # Read response
            if ser.in_waiting:
                response = ser.read(ser.in_waiting)
                positions = [int.from_bytes(response[i:i+2], byteorder='little') 
                           for i in range(5, len(response)-1, 2)]
                print(f"📡 Servo Positions: {positions}")
            else:
                print("⚠️ No response from servos")
            
            time.sleep(0.5)  # Wait before next read

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"⚠️ Error reading positions: {e}")
            time.sleep(1)

except serial.SerialException as e:
    print(f"❌ Error: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("🔌 COM9 closed.")
