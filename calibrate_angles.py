"""
calibrate_angles.py
SED 1115 Lab 8: Calibration Program
by: Zahra Ailan
Description:
This program guides the user through a calibration process for a two-servo
robot arm (shoulder and elbow). It sends a series of test angles to each servo
and instructs the user to measure and record the actual angles observed.
The data is used to create a calibration file for correcting servo nonlinearity.
"""

import time

# Unique ID for this test jig 
JIG_ID = "300389429"

# Output file name for calibration data
CALIBRATION_FILE = f"calibration_data_{JIG_ID}.txt"


def set_servo_angle(servo_pin, angle):
    
    #Simulates sending a PWM signal to a servo motor
    
    print(f"[HARDWARE SIM] Servo {servo_pin} → {angle:3d}°")
    time.sleep(1)  # Pause to allow movement to stabilize


def run_calibration_sequence():

    #Runs the calibration procedure for both servos.
    #Sends a set of angles and prompts the user to measure actual positions.
    
    # Servo names and identifiers 
    SERVO_MAPPING = {"Shoulder (Servo A)": "PIN_A",
        "Elbow (Servo B)": "PIN_B"}

    # Angles to test (0° to 180° in 10° steps)
    desired_angles = list(range(0, 181, 10))

    # Header
    print("\n=========================================================")
    print("      SERVO CALIBRATION PROGRAM – SED 1115 LAB 8")
    print("=========================================================")
    print(f"Jig ID: {JIG_ID}")
    print(f"Calibration Data File: {CALIBRATION_FILE}")
    print("Goal: Record actual measured angles for each servo position.")
    print("=========================================================\n")

    # Run calibration for each servo
    for servo_name, pin in SERVO_MAPPING.items():
        print(f"\n--- Calibrating {servo_name} (Pin: {pin}) ---")
        print("Desired Angle (°) | Action")
        print("-----------------|---------------------------------")

        for angle in desired_angles:
            set_servo_angle(pin, angle)
            print(f"{angle:17d} | Measure and record the actual angle now.")

    print("\n=========================================================")
    print("Calibration sequence complete.")
    print(f"Next step: Record your measured data in '{CALIBRATION_FILE}'.")
    print("=========================================================\n")


# Run the program
if __name__ == "__main__":
    run_calibration_sequence()