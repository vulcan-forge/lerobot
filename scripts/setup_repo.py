import os
import subprocess
import sys
import shutil
from pathlib import Path

def install_uv():
    """Install UV if not present"""
    try:
        subprocess.run(['uv', '--version'], capture_output=True, check=True)
        print("UV already installed!")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("UV not found. Installing UV...")
        try:
            subprocess.run([sys.executable, '-m', 'pip', 'install', 'uv'], check=True)
            print("\nUV installed successfully!")
            print("\nPlease:")
            print("1. Close and reopen your terminal")
            print("2. Run this script again to complete setup")
            print("This is needed for UV to be available in your path.")
            sys.exit(0)
        except subprocess.CalledProcessError as e:
            print(f"Error installing UV: {e}")
            return False

def check_venv_exists():
    """Check if .venv directory exists"""
    return Path('.venv').exists()

def handle_existing_venv():
    """Handle existing .venv directory"""
    while True:
        response = input("\n.venv already exists. What would you like to do?\n"
                        "1: Skip venv creation (keep existing)\n"
                        "2: Delete and recreate venv\n"
                        "3: Exit\n"
                        "Choose (1-3): ").strip()

        if response == '1':
            print("Keeping existing venv...")
            return True
        elif response == '2':
            print("Removing existing venv...")
            try:
                shutil.rmtree('.venv')
                return False  # venv no longer exists
            except Exception as e:
                print(f"Error removing .venv: {e}")
                return True  # keep existing if can't remove
        elif response == '3':
            print("Exiting setup...")
            sys.exit(0)
        else:
            print("Invalid choice. Please choose 1, 2, or 3.")

def setup_venv():
    """Create and setup virtual environment using UV"""
    venv_exists = check_venv_exists()

    if venv_exists:
        venv_exists = handle_existing_venv()

    if not venv_exists:
        try:
            # Create venv
            subprocess.run(['uv', 'venv', '.venv'], check=True)

            # Install requirements
            subprocess.run(['uv', 'pip', 'install', '-e', '.'], check=True)

            print("Virtual environment created successfully!")
        except subprocess.CalledProcessError as e:
            print(f"Error setting up virtual environment: {e}")
            return False

    return True

def main():
    """Main setup function"""
    print("Starting repository setup...")

    # Install UV if needed
    if not install_uv():
        print("Failed to install UV. Please install manually with: pip install uv")
        sys.exit(1)

    # Setup virtual environment
    if not setup_venv():
        sys.exit(1)

    print("\nSetup complete!")
    print("\nTo finish setup, please:")
    print("1. Close and reopen your terminal")
    print("2. Navigate to this directory")
    print("The virtual environment will automatically activate!")

if __name__ == "__main__":
    main()
