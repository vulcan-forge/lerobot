import os
import subprocess
import sys
import shutil
from pathlib import Path
import json

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
    """Setup virtual environment using UV"""
    print("Setting up virtual environment...")
    try:
        # Create virtual environment with Python 3.10
        subprocess.run(["uv", "venv", "-p", "3.10"], check=True)

        # Install dependencies with feetech extras
        subprocess.run(["uv", "pip", "install", "-e", ".[feetech]"], check=True)

        print("Virtual environment created successfully!")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error setting up virtual environment: {e}")
        return False

def setup_vscode_settings():
    """Setup VS Code settings to automatically activate the Python environment"""
    vscode_dir = Path('.vscode')
    settings_file = vscode_dir / 'settings.json'

    # Create .vscode directory if it doesn't exist
    vscode_dir.mkdir(exist_ok=True)

    # Set the correct interpreter path based on OS
    # Unix-like systems (Linux, macOS)
    interpreter_path = "${workspaceFolder}/.venv/bin/python"
    if os.name == 'nt':  # Windows
        interpreter_path = "${workspaceFolder}\\.venv\\Scripts\\python.exe"

    # Default settings to add
    python_settings = {
        "python.defaultInterpreterPath": interpreter_path,
        # We don't need to activate automatically since python is using the .venv as a default interpreter
        # Commenting to showcase the command to automatically activate the environment if needed
        # "python.terminal.activateEnvironment": True
    }

    # Read existing settings if file exists
    if settings_file.exists():
        with open(settings_file, 'r') as f:
            try:
                settings = json.loads(f.read() or '{}')
            except json.JSONDecodeError:
                settings = {}
    else:
        settings = {}

    # Update settings
    settings.update(python_settings)

    # Write updated settings with proper JSON formatting
    with open(settings_file, 'w') as f:
        json.dump(settings, f, indent=4)

    print("VS Code settings updated successfully!")
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

    # Setup VS Code settings
    if not setup_vscode_settings():
        print("Warning: Failed to setup VS Code settings")

    print("\nSetup complete!")
    print("\nTo finish setup, please:")
    print("1. Close and reopen your terminal")
    print("2. Navigate to this directory")
    print("The virtual environment will automatically activate!")

if __name__ == "__main__":
    main()
