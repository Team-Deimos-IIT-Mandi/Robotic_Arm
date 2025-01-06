import math
import time
import keyboard

initial_x, initial_y, initial_z = 100, 100, 100
x, y, z = initial_x, initial_y, initial_z
l1 = 450
l2 = 450
l3 = 10  # Home position

previous_angles = [120, 20, 45]  # Initial angles (assuming the arm starts at 0 degrees)


def calculate_angles(x, y, z, l1, l2, l3):
    l3 = (x ** 2 + y ** 2 + z ** 2) ** 0.5

    angle_beta = math.degrees(math.acos((l2 ** 2 + l3 ** 2 - l1 ** 2) / (2 * l2 * l3)))
    angle_alfa = math.degrees(math.acos((l1 ** 2 + l3 ** 2 - l2 ** 2) / (2 * l1 * l3)))
    angle_gama = math.degrees(math.acos((l1 ** 2 + l2 ** 2 - l3 ** 2) / (2 * l1 * l2)))

    angle_delta = math.degrees(math.atan2(y, x))
    theta1 = angle_delta + angle_alfa
    theta2 = angle_gama
    theta3 = math.degrees(math.atan2(z, x))

    return theta1, theta2, theta3


def move_arm_to_position(x, y, z, l1, l2, l3):
    global previous_angles

    print(f"Moving to position x={x}, y={y}, z={z}")
    new_angles = calculate_angles(x, y, z, l1, l2, l3)

    # Calculate the angle differences
    angle_differences = [abs(new_angle - prev_angle) for new_angle, prev_angle in zip(new_angles, previous_angles)]

    # Calculate the maximum angle difference (this will determine the movement time)
    max_angle_diff = max(angle_differences)

    print(
        f"Angle differences: θ1 = {angle_differences[0]:.2f}°, θ2 = {angle_differences[1]:.2f}°, θ3 = {angle_differences[2]:.2f}°")
    print(f"Max angle difference = {max_angle_diff:.2f}°")

    time.sleep(max_angle_diff)  # 1 second per degree of movement

    print("Movement complete. Ready for the next command.")

    # Update previous angles
    previous_angles = new_angles


def handle_key_press():
    global x, y, z
    while True:
        if keyboard.is_pressed('up'):
            x += 2
            move_arm_to_position(x, y, z, l1, l2, l3)
            while keyboard.is_pressed('up'):  # Wait for the key to be released
                pass

        elif keyboard.is_pressed('down'):
            x -= 2
            move_arm_to_position(x, y, z, l1, l2, l3)
            while keyboard.is_pressed('down'):  # Wait for the key to be released
                pass

        elif keyboard.is_pressed('left'):
            y -= 2
            move_arm_to_position(x, y, z, l1, l2, l3)
            while keyboard.is_pressed('left'):  # Wait for the key to be released
                pass

        elif keyboard.is_pressed('right'):
            y += 2
            move_arm_to_position(x, y, z, l1, l2, l3)
            while keyboard.is_pressed('right'):  # Wait for the key to be released
                pass

        # Optionally, you can add a small sleep to reduce CPU usage
        time.sleep(0.01)



handle_key_press()
