# Changelog

## [1.0.0] - 2024-XX-XX
### Added
- Created `start_simulation.sh` script to streamline the launch process for Gazebo, ArduPilot SITL, MAVProxy, and Python controllers.
- Added comprehensive connection architecture documentation in `docs/README_CONNECTION.md`.

### Changed
- Replaced the placeholder Gazebo Husky model marker texture with a standardized 5x5 ArUco marker (ID: 72, Dictionary: `DICT_5X5_250`) and a white border to guarantee robust detection.
- Updated `drone_pose_controller.py` to match the ArUco marker changes (`id_to_find = 72` and `aruco_dict = aruco.DICT_5X5_250`).
- Switched MAVProxy to run in daemon mode (`--daemon` and `--non-interactive`) to prevent crashes in headless/background execution on WSL.

### Fixed
- Fixed critical ROS Noetic OpenCV 4.2.0 compatibility issues (`AttributeError: module 'cv2.aruco' has no attribute 'ArucoDetector'`). The pose estimation now natively uses standard OpenCV 4.2 syntax to prevent silent callback crashes.
- Fixed the `[Errno 114] Operation already in progress` connection bug by adding a 5-second initialization delay between SITL and MAVProxy.
- Prevented infinite loops in ROS Publisher by separating input (`/webcam/image_raw`) and output (`/webcam/image_marked`) topics.
