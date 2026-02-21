import time

from lidar import do_a_scan


def convert_angle(angle: float) -> float:
    """
    Convert IMU yaw angle to [0, 360) with a flipped axis, matching the original logic.
    """
    return (360 - (angle % 360)) % 360


def calculate_rotation(prev_angle: float, curr_angle: float) -> float:
    """
    Calculate the signed shortest difference between two angles (degrees).
    """
    diff = curr_angle - prev_angle
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff


def encircling(bot, laser, scan, x_times: int):
    """
    Perform x_times encirclements around the target cone, using LiDAR and IMU data
    to follow the closest obstacle and track rotation.
    """
    init_roll, init_pitch, init_yaw = bot.get_imu_attitude_data()
    converted_init_yaw = convert_angle(init_yaw)

    # Ensure we get a valid initial yaw (non-zero in this logic)
    while converted_init_yaw == 0:
        print("Waiting for valid initial yaw...")
        init_roll, init_pitch, init_yaw = bot.get_imu_attitude_data()
        converted_init_yaw = convert_angle(init_yaw)

    last_encirclement_time = time.time()
    cooldown_seconds = 2

    while x_times > 0:
        scan_dict = do_a_scan(laser, scan)

        # Filter out readings at front/back sectors; keep side readings only
        filtered_items = [
            (angle, value)
            for angle, value in scan_dict.items()
            if value > 0 and not (0 <= angle <= 40 or 190 <= angle <= 360)
        ]

        if not filtered_items:
            # No valid points, skip this loop
            continue

        # Find the closest point and align servo towards it
        min_pair = min(filtered_items, key=lambda x: x[1])
        min_angle, min_range = min_pair

        bot.set_pwm_servo(1, (min_angle - 90))
        bot.set_motor(30, 30, 30, 30)
        bot.set_pwm_servo(1, 0)
        time.sleep(0.1)
        bot.set_motor(30, 30, 30, 30)

        # Read current yaw and compute rotation relative to initial yaw
        current_roll, current_pitch, current_yaw = bot.get_imu_attitude_data()
        converted_current_yaw = convert_angle(current_yaw)
        diff = calculate_rotation(converted_init_yaw, converted_current_yaw)

        print("x_times remaining:", x_times, "| yaw diff:", diff)

        # Detect roughly one full loop (based on original 160°–164° heuristic)
        if 160 < abs(diff) < 164:
            current_time = time.time()
            if current_time - last_encirclement_time > cooldown_seconds:
                x_times -= 1
                last_encirclement_time = current_time
                print(f"Completed an encirclement! Remaining: {x_times}")

    bot.set_pwm_servo(1, 110)
    bot.set_motor(0, 0, 0, 0)
    print("Encircling complete.")
