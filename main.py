import cv2
import random

from robot_control import init_robot, init_camera, stop_robot
from lidar import init_lidar, shutdown_lidar, check_obstacle_and_scan
from imu_navigation import encircling
from vision import (
    get_color_range,
    find_largest_colored_object,
    bbox_area,
    bbox_center_x,
    draw_bounding_box,
    servo_angle_from_center,
)
from utils import get_randomized_colors, AREA_THRESHOLD, MAX_ITERATIONS


def main():
    bot = None
    cap = None
    laser = None
    scan = None

    try:
        # Initialize hardware
        bot = init_robot()
        cap = init_camera(0)
        laser, scan = init_lidar()

        # Get randomized color order
        colors = get_randomized_colors()
        print("Color order:", colors)

        while colors:
            color = colors[0]
            lower_color, upper_color = get_color_range(color)

            if lower_color is None or upper_color is None:
                print(f"Skipping unknown color: {color}")
                colors.remove(color)
                continue

            times = random.randint(1, 4)
            print(
                f"Starting {color.upper()} with {times} encirclements "
                f"(Colors remaining: {colors})"
            )

            iteration = 0

            while iteration < MAX_ITERATIONS:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read frame from camera.")
                    break

                frame_height, frame_width = frame.shape[:2]

                bbox = find_largest_colored_object(frame, lower_color, upper_color)

                if bbox:
                    area = bbox_area(bbox)
                    center_x = bbox_center_x(bbox)
                    servo_angle = servo_angle_from_center(frame_width, center_x)

                    # Aim camera/robot towards the cone and move forward
                    bot.set_pwm_servo(1, servo_angle)
                    bot.set_motor(30, 30, 30, 30)

                    draw_bounding_box(frame, bbox)

                    x, y, w, h = bbox
                    print(
                        f"{color.upper()} object at x={x}, y={y}, w={w}, h={h} "
                        f"-> Area: {area}"
                    )

                    # Trigger encircling when close enough
                    if area > AREA_THRESHOLD:
                        bot.set_pwm_servo(1, 105)

                        obstacle_detected = check_obstacle_and_scan(
                            laser, scan, 85, 90, 0.7
                        )
                        if not obstacle_detected:
                            # Try a second time as in the original logic
                            obstacle_detected = check_obstacle_and_scan(
                                laser, scan, 85, 90, 0.7
                            )

                        if obstacle_detected:
                            print(
                                f"Obstacle detected in the specified range! "
                                f"Performing {times} encirclements for {color.upper()}"
                            )
                            encircling(bot, laser, scan, times)
                            print(
                                f"Finished encircling for {color.upper()} "
                                f"with {times} encirclements."
                            )
                            colors.remove(color)
                            break
                        else:
                            print("No obstacle detected in the specified range.")
                else:
                    # No cone detected – stop movement for safety
                    bot.set_motor(0, 0, 0, 0)

                cv2.imshow("Color Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("Exit requested by user.")
                    return

                iteration += 1

            if iteration >= MAX_ITERATIONS:
                print(
                    f"Could not find {color.upper()} "
                    f"within {MAX_ITERATIONS} iterations. Skipping."
                )
                colors.remove(color)

    finally:
        # Cleanup resources
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()

        if laser is not None:
            shutdown_lidar(laser)

        if bot is not None:
            stop_robot(bot)

        print("Resources cleaned up. Program terminated.")


if __name__ == "__main__":
    main()
