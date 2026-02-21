import numpy as np
import ydlidar


def init_lidar():
    """
    Initialize the YDLiDAR and return (laser, scan).
    """
    ports = ydlidar.lidarPortList()
    port = "/dev/ydlidar"
    for _, value in ports.items():
        port = value  # use detected port if available

    laser = ydlidar.CYdLidar()
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 512000)
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TOF)
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 20)
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 32.0)
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01)

    scan = ydlidar.LaserScan()

    ret = laser.initialize()
    if not ret:
        raise RuntimeError("Failed to initialize LiDAR.")

    ret = laser.turnOn()
    if not ret:
        raise RuntimeError("Failed to start LiDAR scan.")

    print("LiDAR initialized on port:", port)
    return laser, scan


def shutdown_lidar(laser):
    """
    Safely shut down the LiDAR.
    """
    if laser:
        laser.turnOff()
        laser.disconnecting()
        print("LiDAR shut down.")


def do_a_scan(laser, scan):
    """
    Perform a scan and return a dict: angle(deg) -> range(m).
    Missing readings are simply omitted.
    """
    angle_range_dict = {}
    r = laser.doProcessSimple(scan)
    if r:
        for point in scan.points:
            angle_degrees = int(np.degrees(point.angle) % 360)
            angle_range_dict[angle_degrees] = point.range

    return angle_range_dict


def check_obstacle_and_scan(laser, scan, angle_min, angle_max, range_threshold) -> bool:
    """
    Check if there is an obstacle within [angle_min, angle_max] whose distance
    is <= range_threshold. Returns True if an obstacle is detected.
    """
    r = laser.doProcessSimple(scan)
    if not r:
        print("Failed to process LiDAR scan.")
        return False

    obstacle_detected = False

    for point in scan.points:
        angle_degrees = np.degrees(point.angle) % 360
        range_val = point.range

        if angle_min <= angle_degrees <= angle_max:
            if (range_val <= range_threshold) and (range_val != 0):
                print(
                    f"Obstacle detected at {angle_degrees:.2f}° "
                    f"with distance {range_val:.2f} m"
                )
                obstacle_detected = True

    return obstacle_detected
