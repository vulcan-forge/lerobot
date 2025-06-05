import cv2

def list_available_cameras(max_ports=10):
    print("Scanning for available camera ports...")
    available_ports = []

    for i in range(max_ports):
        cap = cv2.VideoCapture(i)
        if cap is None or not cap.isOpened():
            print(f"Port {i}: Not available")
        else:
            print(f"Port {i}: Available")
            available_ports.append(i)
            cap.release()

    return available_ports

if __name__ == "__main__":
    available = list_available_cameras()
    if available:
        print("\nAvailable camera ports:", available)
    else:
        print("\nNo available camera ports found.")
