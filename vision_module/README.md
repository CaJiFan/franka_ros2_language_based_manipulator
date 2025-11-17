# Vision Module

This project implements a ROS 2 package for object detection using the YOLO (You Only Look Once) algorithm. The package processes images, detects objects, and publishes their information via ROS 2 topics.

## Project Structure

```
vision_module
├── src
│   └── vision_module
│       ├── __init__.py
│       ├── vision_node.py        # Main ROS 2 node for object detection
│       ├── yolo_detector.py      # YOLO model handling and object detection
│       ├── publishers
│       │   └── object_publisher.py # Publisher for detected object information
│       ├── utils
│       │   └── transforms.py      # Utility functions for image transformations
│       └── types
│           └── __init__.py        # Custom data types for object detection
├── package.xml                    # ROS 2 package configuration
├── setup.cfg                      # Package configuration settings
├── setup.py                       # Build script for the package
├── resource
│   └── vision_module              # Additional resources (e.g., model weights)
├── launch
│   └── vision_launch.py           # Launch configuration for the ROS 2 node
├── config
│   └── yolo.yaml                  # YOLO model configuration settings
├── requirements.txt               # Python dependencies for the project
├── tests
│   └── test_detection.py          # Unit tests for object detection functionality
└── README.md                      # Project documentation
```

## Installation

1. Clone the repository:
   ```
   git clone <repository-url>
   cd vision_module
   ```

2. Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

3. Build the package:
   ```
   colcon build
   ```

4. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage

To run the object detection node, use the following command:
```
ros2 run vision_module vision_node
```

Ensure that you have a camera or image source publishing images to the appropriate topic.

## YOLO Detection Process

The YOLO detector processes incoming images, detects objects, and retrieves their positions. Detected object information, including labels and confidence scores, is published to a specified ROS 2 topic for further processing or visualization.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for details.