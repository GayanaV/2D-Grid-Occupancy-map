 Develop a 2D Occupancy Grid Map of a Room using Overhead Cameras 

The primary objective of this project is to develop a 2D occupancy grid map of a room using overhead cameras, similar to the map created by a ROS2-based SLAM algorithm typically used by autonomous mobile robots (AMRs).


# Steps

# Phase 1: Static Environment Mapping :
### Setting up the ROS2 environment :

+ Ubuntu 20.04
+ Install ROS2 foxy, packages – [3.1.2-3.1.4](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
+ Check `“ros2 topic list”`
+ export TURTLEBOT3_MODEL=waffle_pi
+ Steps to create ros2 workspace
    + `“mkdir –p ~/turtlebot3_ws/src”`
    + Install turtlebot3 packages – [6.1.1,6.1.2](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)
    + Run the house simulation 
+ Check `“ros2 topic list”`
+ Locations Overhead cameras
[C1: (-5,-2,8) C2:(-5,3,8) C3:(1,-2,8) C4:(1,3,8)]

### Overhead Cameras :
+ Download the infraCam.zip
+ Extract the folder `“turtlebot3_camera_house”` 
and place in the `“……/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models”` directory.
+  Replace the `waffle_pi.model` in 
`“…../turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds/turtlebot3_houses/”` with the downloaded version.
+  Change path in lines 6,11,16,21
+  Source and build the workspace - `"$ source install/setup.bash"`.
+  Run house simulation: 
`“ros2 launch turtlebot3_gazebo 
turtlebot3_house.launch.py”`
+  Topics related overhead camera are now 
visible.

###  to acquire data from camera : image_listener.py

+ ROS works as a pub-sub model 
+ The images are being streamed on the topics.
+ The images can be accessed through the python script.
+ Always `“source /opt/ros/foxy/setup.bash”` before running the python script.
+ 1 – Initiates the nodes (script will be given unique name) and waits for user to interrupt the program.
+ 2 – Created subscription to camera topic.
+ 3 – Every time the camera image is published, listener_callback function is executed.

###  to evaluate the generated map?

Evaluation_of_gen_map
+ Run command below (replace path in yaml file)

```bash
$ ros2 launch turtlebot3_navigation2 
$ navigation2.launch.py use_sim_time:=True 
$ map:=$HOME/map_house.yaml
```

+ Rviz: a visualization tool for ROS
+ We have 8 key point to point measurements as benchmarks.
+ A map generated with cartographer will be provided for reference.

<img src="./img/refmap.png" width=100%>

#### How to evaluate the generated map?
#### Using Rviz to find distance between key points

### + `Step 2`: Capture and Stitch Images

+ Capture Images from Cameras (Image Stitching Node):

Write a ROS2 node to capture images from the overhead cameras.
```python

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageStitchingNode(Node):
    def __init__(self):
        super().__init__('image_stitching_node')
        
        self.bridge = CvBridge()
        self.images = {}
        self.subscribers = []

        camera_topics = ["/camera1/image_raw", "/camera2/image_raw", "/camera3/image_raw", "/camera4/image_raw"]

        for i, topic in enumerate(camera_topics):
            self.subscribers.append(self.create_subscription(Image, topic, lambda msg, idx=i: self.image_callback(msg, idx), 10))
        
        self.stitched_image_pub = self.create_publisher(Image, '/stitched_image', 10)

    def image_callback(self, msg, camera_index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.images[camera_index] = cv_image
            
            if len(self.images) == len(self.subscribers):  # Check if all cameras have sent images
                self.stitch_images()
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def stitch_images(self):
        images = [self.images[i] for i in sorted(self.images.keys())]
        
        stitcher = cv2.Stitcher_create()
        status, stitched = stitcher.stitch(images)

        if status == cv2.Stitcher_OK:
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
            self.stitched_image_pub.publish(stitched_msg)
        else:
            self.get_logger().error(f"Image stitching failed with status {status}")

def main(args=None):
    rclpy.init(args=args)
    image_stitching_node = ImageStitchingNode()
    rclpy.spin(image_stitching_node)
    image_stitching_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### + `Step 3`: Generate 2D Occupancy Grid Map (Create the Occupancy Grid Map Node)

+ Convert Image to Grayscale and Threshold:

Convert the stitched image to a binary occupancy grid.

+ Generate Occupancy Grid:

Create an occupancy grid from the binary image.

```python
#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

class OccupancyGridMapNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_map_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/stitched_image',
            self.image_callback,
            10
        )
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
            
            occupancy_grid = self.create_occupancy_grid(binary_image)
            self.occupancy_grid_pub.publish(occupancy_grid)
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def create_occupancy_grid(self, binary_image):
        occupancy_grid = OccupancyGrid()
        
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = "map"
        
        occupancy_grid.info.resolution = 0.05
        occupancy_grid.info.width = binary_image.shape[1]
        occupancy_grid.info.height = binary_image.shape[0]
        occupancy_grid.info.origin.position.x = 0
        occupancy_grid.info.origin.position.y = 0
        occupancy_grid.info.origin.position.z = 0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        data = []
        for i in range(binary_image.shape[0]):
            for j in range(binary_image.shape[1]):
                if binary_image[i, j] == 255:
                    data.append(0)  # Free space
                else:
                    data.append(100)  # Occupied space
        
        occupancy_grid.data = data
        
        return occupancy_grid

def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_node = OccupancyGridMapNode()
    rclpy.spin(occupancy_grid_node)
    occupancy_grid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 try:
        OccupancyGridMapNode()
    except rclpy.ROSInterruptException:
        pass
```

Make the script executable : `$ chmod +x src/image_processing/src/occupancy_grid_map.py`.

### + `Step 4`: Dynamic environment code

+ stitch image dynamic environment 
```python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ImageStitchingNode(Node):
    def __init__(self):
        super().__init__('image_stitching_node')
        self.bridge = CvBridge()
        self.images = [None] * 4  # Initialize list to store images from 4 cameras
        self.camera_topics = [
            '/overhead_camera/overhead_camera1/image_raw',
            '/overhead_camera/overhead_camera2/image_raw',
            '/overhead_camera/overhead_camera3/image_raw',
            '/overhead_camera/overhead_camera4/image_raw'
        ]
        self.subscribers = []

        for i, topic in enumerate(self.camera_topics):
            self.subscribers.append(self.create_subscription(
                Image,
                topic,
                lambda msg, index=i: self.image_callback(msg, index),
                10
            ))

        self.stitched_image_pub = self.create_publisher(Image, '/stitched_image', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('ImageStitchingNode has been started.')

    def image_callback(self, msg, index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.images[index] = cv_image
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def timer_callback(self):
        if all(image is not None for image in self.images):
            try:
                self.stitch_images()
            except Exception as e:
                self.get_logger().error(f'Error in timer_callback: {e}')

    def combine_images(self, images):
        resized_images = [cv2.resize(img, (320, 240)) for img in images]
        top_row = np.hstack((resized_images[3], resized_images[2]))
        bottom_row = np.hstack((resized_images[1], resized_images[0]))
        combined_image = np.vstack((top_row, bottom_row))
        return combined_image

    def stitch_images(self):
        combined_image = self.combine_images(self.images)

        try:
            # Save the stitched image to a file
            output_path = os.path.expanduser('~/stitched_image.jpg')
            cv2.imwrite(output_path, combined_image)
            self.get_logger().info(f'Stitched image saved to {output_path}')

            # Publish the stitched image
            stitched_msg = self.bridge.cv2_to_imgmsg(combined_image, 'bgr8')
            self.stitched_image_pub.publish(stitched_msg)
            self.get_logger().info('Image stitching succeeded.')
        except Exception as e:
            self.get_logger().error(f'Error in stitch_images: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_stitching_node = ImageStitchingNode()
    rclpy.spin(image_stitching_node)
    image_stitching_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### + `Step 5`: Simulation in Gazebo and Validate

Created a launch file to run both nodes.

Created a launch file image_processing.launch: 

```xml
<launch>
    <node name="image_stitching_node" pkg="image_processing" type="image_stitching.py" output="screen"/>
    <node name="occupancy_grid_map_node" pkg="image_processing" type="occupancy_grid_map.py" output="screen"/>
</launch>
```

Run the launch file : ` $ roslaunch image_processing image_processing.launch`.



+ Implement Object Detection:

Use a pre-trained model (e.g., YOLO, SSD) to detect objects and also to update the occupancy grid.

```python
import cv2
import numpy as np

def detect_objects(image):
    # Use a pre-trained model to detect objects
    net = cv2.dnn.readNet('yolov3.weights', 'yolov3.cfg')
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    blob = cv2.dnn.blobFromImage(image, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
    net.setInput(blob)
    detections = net.forward(output_layers)
    return detections

detections = detect_objects(stitched_image)
```

+ Update Occupancy Grid:

Adjust the occupancy grid based on detected objects.

```python
def update_occupancy_grid(grid, detections):
    for detection in detections:
        x, y, w, h = detection
        grid[y:y+h, x:x+w] = 1  # Mark detected object area as occupied
    return grid

updated_grid = update_occupancy_grid(occupancy_grid, detections)
```

+ Publish Updated Occupancy Grid:

Continuously update and publish the new occupancy grid.

```python
while True:
    detections = detect_objects(stitched_image)
    updated_grid = update_occupancy_grid(occupancy_grid, detections)
    publish_occupancy_grid(updated_grid)
```

### + `Step 6`: Add Semantic Labels

+ Add Semantic Information:
Label detected objects with semantic information.

```python
def add_semantic_labels(grid, detections):
    labels = ["table", "chair", "stool", "box"]
    for detection, label in zip(detections, labels):
        x, y, w, h = detection
        grid[y:y+h, x:x+w] = label  # Annotate grid with labels
    return grid

labeled_grid = add_semantic_labels(occupancy_grid, detections)
```

### + `Step 7`: Validate and Compare with AMR-generated Map
+ Simulation of AMR with SLAM:
  
# Object Detection:

+ Use computer vision techniques (e.g., deep learning models like YOLO or Faster R-CNN) to detect objects such as tables, chairs, plant pots, and traffic cones in the 2D image.
3D Reconstruction:

+ Convert the 2D image into a 3D representation of the scene using methods like structure-from-motion (SfM) or multi-view stereo (MVS). Estimate the 3D positions of objects based on their appearances across multiple images or viewpoints.
Spatial Scene Graph Construction:

+ Construct a spatial scene graph to represent the relationships between objects and their positions in 3D space.
Each node in the graph corresponds to an object, and edges depict spatial relationships such as adjacency or containment.
Dynamic Updates:

+ Implement mechanisms to dynamically update the scene graph when objects are removed from the image:
Re-detect objects in the updated image.
Refine the 3D representation of the scene.
Modify the spatial scene graph to reflect changes in object positions or relationships.
# Integration and Comparison:
+ Develop a 2D occupancy grid map using overhead camera inputs.
+ Compare this map with maps autonomously generated by AMRs using SLAM techniques.
+ Evaluate differences and similarities in map accuracy, completeness, and suitability for navigation and path-planning tasks.

By following these steps, you can effectively develop and compare 2D occupancy grid maps generated from overhead cameras with those produced by AMRs using SLAM, enhancing capabilities for navigation and situational awareness in simulated environments.

  
Here's a high-level Python pseudo-code example to illustrate the process:

```python
# Step 1: Object Detection
detected_objects = detect_objects(image)

# Step 2: 3D Reconstruction
point_cloud = reconstruct_3d_scene(image)

# Step 3: Spatial Scene Graph Construction
scene_graph = construct_scene_graph(detected_objects, point_cloud)

# Step 4: Dynamic Updates
def remove_object(object_id):
    # Remove object from detected_objects list
    detected_objects.remove(object_id)
    
    # Reconstruct 3D scene
    updated_point_cloud = reconstruct_3d_scene(updated_image)
    
    # Update scene graph
    update_scene_graph(scene_graph, detected_objects, updated_point_cloud)

# Example usage
remove_object("traffic_cone")

```
<hr />
