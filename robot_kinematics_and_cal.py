"""
SED 1115 – Lab 8: Inverse Kinematics & Servo Calibration
by Zahra Ailan
Description:
Calculates servo angles for a 2-link robotic arm, applies calibration
corrections from file, and sends compensated angles to the hardware.
"""

import math
import time

# ------------------ CONSTANTS ------------------
JIG_ID = "300389429"
CALIBRATION_FILE = f"calibration_data_3000389429.txt"

# Arm geometry (mm)
A_X, A_Y = -50.0, 139.5
L_A, L_B = 155.0, 155.0


# ------------------ HARDWARE MOCK ------------------
def set_servo_angle(servo_pin, angle):
    """Simulates sending a PWM signal to the servo (0–180°)."""
    angle = max(0, min(180, angle))
    print(f"[HARDWARE] {servo_pin} → {angle:.1f}°")
    time.sleep(0.05)


# ------------------ INVERSE KINEMATICS ------------------
def inverse_kinematics(Cx, Cy):
    """
    Computes servo angles (in degrees) for a 2-link arm.
    Returns (servoA, servoB), or (None, None) if unreachable.
    """
    try:
        # Distance between base (A) and end effector (C)
        AC = math.sqrt((Cx - A_X)**2 + (Cy - A_Y)**2)
        cos_BAC = (L_A**2 + AC**2 - L_B**2) / (2 * L_A * AC)
        cos_BAC = max(-1, min(1, cos_BAC))
        angle_BAC = math.acos(cos_BAC)

        # Angle from vertical axis
        cos_YAC = (A_Y**2 + AC**2 - (Cy)**2) / (2 * A_Y * AC)
        cos_YAC = max(-1, min(1, cos_YAC))
        angle_YAC = math.acos(cos_YAC)

        # Angle at elbow
        sin_ACB = (L_A * math.sin(angle_BAC)) / L_B
        sin_ACB = max(-1, min(1, sin_ACB))
        angle_ACB = math.asin(sin_ACB)

        # Convert to servo angles
        alpha = math.degrees(angle_BAC + angle_YAC)
        beta = math.degrees(angle_BAC + angle_ACB)
        return alpha - 75.0, 150.0 - beta
    except (ValueError, ZeroDivisionError):
        print(f"[IK ERROR] Target ({Cx}, {Cy}) unreachable.")
        return None, None


# ------------------ CALIBRATION ------------------
def load_calibration(filename):
    """Reads calibration file and returns {'A': [...], 'B': [...]}."""
    data = {'A': [], 'B': []}
    servo = None
    try:
        with open(filename) as f:
            for line in f:
                line = line.strip()
                if "SERVO A" in line: servo = 'A'
                elif "SERVO B" in line: servo = 'B'
                elif servo and "Desired" in line and "Error" in line:
                    try:
                        desired = float(line.split("Desired Angle:")[1].split(',')[0])
                        error = float(line.split("Error:")[1].split(',')[0])
                        data[servo].append((desired, error))
                    except Exception:
                        pass
        print(f"[OK] Calibration loaded from {filename}") #error handling
    except FileNotFoundError:
        print(f"[WARN] File '{filename}' not found.")
    return data


def interpolate(desired, pairs):
    """Linear interpolation for servo error."""
    if not pairs:
        return 0.0
    for i in range(len(pairs) - 1):
        x1, y1 = pairs[i]
        x2, y2 = pairs[i + 1]
        if x1 <= desired <= x2:
            return y1 + (desired - x1) * (y2 - y1) / (x2 - x1)
    return pairs[0][1] if desired < pairs[0][0] else pairs[-1][1]


def send_compensated(servo_name, desired, table):
    """Applies calibration and sends the corrected angle."""
    key = 'A' if 'shoulder' in servo_name.lower() else 'B'
    pin = f"PIN_{key}"
    error = interpolate(desired, table.get(key, []))
    final = desired + error
    print(f"{servo_name}: Desired={desired:.1f}, Error={error:.1f}, Final={final:.1f}")
    set_servo_angle(pin, final)
    return final


# ------------------ MAIN TEST ------------------
if __name__ == "__main__":
    print("\n--- SED 1115 LAB 8: INVERSE KINEMATICS & CALIBRATION ---\n")

    # 1. Load calibration data
    errors = load_calibration(CALIBRATION_FILE)

    # 2. Compute target servo angles
    target_x, target_y = 150.0, 100.0
    servoA, servoB = inverse_kinematics(target_x, target_y)

    if servoA is not None:
        # 3. Send compensated angles
        send_compensated("Shoulder Servo A", servoA, errors)
        send_compensated("Elbow Servo B", servoB, errors)

        # 4. Send uncalibrated for comparison
        print("\nUncalibrated Output:")
        set_servo_angle("PIN_A", servoA)
        set_servo_angle("PIN_B", servoB)
