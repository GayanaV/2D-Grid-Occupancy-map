 Develop a 2D Occupancy Grid Map of a Room using Overhead Cameras [INT 2]

The primary objective of this project is to develop a 2D occupancy grid map of a room using overhead cameras, similar to the map created by a ROS2-based SLAM algorithm typically used by autonomous mobile robots (AMRs).

<img src="./img/cam.png" width=47%> <img src="./img/dsg.png" width=51%>
<hr />

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
+ Download the [infraCam.zip](./infraCam/)
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

### How to acquire data from camera : [[image_listener.py](./image_listener.py)]

+ ROS works as a pub-sub model 
+ The images are being streamed on the topics.
+ The images can be accessed through the python script.
+ Always `“source /opt/ros/foxy/setup.bash”` before running the python script.
+ 1 – Initiates the nodes (script will be given unique name) and waits for user to interrupt the program.
+ 2 – Created subscription to camera topic.
+ 3 – Every time the camera image is published, listener_callback function is executed.

### How to evaluate the generated map?

[[Evaluation_of_gen_map](./Evaluation_of_gen_map/)]
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

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageStitchingNode:
    def __init__(self):
        rospy.init_node('image_stitching_node', anonymous=True)
        
        self.bridge = CvBridge()
        self.images = {}
        self.subscribers = []
        
        camera_topics = ["/camera1/image_raw", "/camera2/image_raw", "/camera3/image_raw", "/camera4/image_raw"]
        
        for i, topic in enumerate(camera_topics):
            self.subscribers.append(rospy.Subscriber(topic, Image, self.image_callback, callback_args=i))
        
        self.stitched_image_pub = rospy.Publisher('/stitched_image', Image, queue_size=10)
        
        rospy.spin()

    def image_callback(self, msg, camera_index):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.images[camera_index] = cv_image
            
            if len(self.images) == 4:
                self.stitch_images()
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def stitch_images(self):
        images = [self.images[i] for i in sorted(self.images.keys())]
        
        stitcher = cv2.Stitcher.create()
        (status, stitched) = stitcher.stitch(images)
        
        if status == cv2.Stitcher_OK:
            stitched_msg = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
            self.stitched_image_pub.publish(stitched_msg)
        else:
            rospy.logerr(f"Image stitching failed with status {status}")

if __name__ == '__main__':
    try:
        ImageStitchingNode()
    except rospy.ROSInterruptException:
        pass
```
Make the script executable : `$ chmod +x src/image_processing/src/image_stitching.py`.

### + `Step 3`: Generate 2D Occupancy Grid Map (Create the Occupancy Grid Map Node)

+ Convert Image to Grayscale and Threshold:

Convert the stitched image to a binary occupancy grid.

+ Generate Occupancy Grid:

Create an occupancy grid from the binary image.

```python
#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge

class OccupancyGridMapNode:
    def __init__(self):
        rospy.init_node('occupancy_grid_map_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        rospy.Subscriber("/stitched_image", Image, self.image_callback)
        self.occupancy_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=10)
        
        rospy.spin()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
            
            occupancy_grid = self.create_occupancy_grid(binary_image)
            self.occupancy_grid_pub.publish(occupancy_grid)
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

    def create_occupancy_grid(self, binary_image):
        occupancy_grid = OccupancyGrid()
        
        occupancy_grid.header.stamp = rospy.Time.now()
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
                    data.append(0)
                else:
                    data.append(100)
        
        occupancy_grid.data = data
        
        return occupancy_grid

if __name__ == '__main__':
    try:
        OccupancyGridMapNode()
    except rospy.ROSInterruptException:
        pass
```

Make the script executable : `$ chmod +x src/image_processing/src/occupancy_grid_map.py`.

### + `Step 4`: Simulate in Gazebo and Validate

Create a launch file to run both nodes.

Create a launch file image_processing.launch: 

```xml
<launch>
    <node name="image_stitching_node" pkg="image_processing" type="image_stitching.py" output="screen"/>
    <node name="occupancy_grid_map_node" pkg="image_processing" type="occupancy_grid_map.py" output="screen"/>
</launch>
```

Run the launch file : ` $ roslaunch image_processing image_processing.launch`.



+ Implement Object Detection:

Use a pre-trained model (e.g., YOLO, SSD) to detect objects and update the occupancy grid.

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
+ Simulate AMR with SLAM:
  
Use the ROS2 navigation stack to simulate an AMR generating its own map.
Compare the AMR-generated map with the overhead camera map.

By following these steps, we will be able to develop a 2D occupancy grid map using overhead cameras, dynamically update it with object detection, and add semantic labels for improved navigation and path-planning in a simulated environment.

Creating a dynamic 3D spatial scene graph from a 2D image requires a combination of computer vision techniques and 3D rendering. Here's a simplified outline of how you could approach this:

+ `Object Detection`: Use computer vision techniques (e.g., deep learning-based object detection models like YOLO or Faster R-CNN) to detect objects in the image. This step involves identifying the table, chair, plant pot, and traffic cone.

+ `3D Reconstruction`: Convert the 2D image into a 3D representation of the scene. This can be done using structure-from-motion (SfM) or multi-view stereo (MVS) techniques to estimate the 3D positions of objects based on their appearance in multiple images or viewpoints.

+ `Spatial Scene Graph`: Once you have the 3D representation of the scene, construct a spatial scene graph to represent the relationships between objects and their positions in 3D space. Each node in the graph corresponds to an object, and edges represent spatial relationships (e.g., adjacency, containment).

+ `Dynamic Updates`: Implement a mechanism to dynamically update the scene graph when objects are removed from the image. This involves re-detecting objects in the updated image, updating the 3D representation of the scene, and modifying the spatial scene graph accordingly.
  
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