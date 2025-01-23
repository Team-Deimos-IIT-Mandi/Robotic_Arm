import pygame
import sys

# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() == 0:
    print("No gaming controller detected. Please connect a controller and try again.")
    sys.exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to {joystick.get_name()}")
print("Use the left joystick to move in 2D space.")

# Starting position in 2D space
x, y = 0, 0

# Movement speed multiplier
speed = 5  # Adjust as needed for smoother or faster movement

# Main loop
try:
    while True:
        # Process events
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Read joystick values
                axis_0 = joystick.get_axis(0)  # Horizontal axis (left joystick)
                axis_1 = joystick.get_axis(1)  # Vertical axis (left joystick)

                # Apply movement based on joystick values
                # Note: Axis 1 is typically inverted (+1 = down, -1 = up), so we invert it here
                dx = axis_0 * speed  # Horizontal movement
                dy = -axis_1 * speed  # Vertical movement (inverted)

                # Update the position
                x += dx
                y += dy

                # Print the updated position
                print(f"Position: x = {x:.2f}, y = {y:.2f}")

except KeyboardInterrupt:
    print("\nExiting...")

# Clean up
finally:
    pygame.joystick.quit()
    pygame.quit()
