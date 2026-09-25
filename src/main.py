# main.py
import traceback
from app import SimulatorApp

def main():
    try:
        app = SimulatorApp()
        app.run()
    except Exception as e:
        print("\n[CRITICAL SIMULATOR ERROR]:")
        traceback.print_exc()
        input("\nPress Enter to close the window...")

if __name__ == "__main__":
    main()