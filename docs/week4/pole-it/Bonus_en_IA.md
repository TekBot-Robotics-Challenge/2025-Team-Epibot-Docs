# AI Documentation

## Object Detection and Color Identification using Artificial Intelligence

This computer vision module is designed to integrate with our smart sorting solution. Its goal is to **replace the color sensor** by using **artificial intelligence** to analyze objects on a conveyor belt.

The process is simple:

1. A **camera** captures an image of an object on the conveyor belt.
2. The image is analyzed by the pre-trained **YOLOv8x** model to identify the presence of objects.
3. If an object is detected, the **dominant color** of the object is extracted.
4. The returned result includes:

   * RGB color code
   * Approximate color name
   * Object detection confidence percentage
   * Bounding box coordinates

### Advantages over traditional color sensors

This method offers several significant advantages over traditional spot sensors:

* **Increased accuracy**: Analyzes the entire object instead of a single point
* **Flexibility**: Adapts to different shapes and sizes of objects
* **Contextual intelligence**: Differentiates between the object and the conveyor background

## Technical Details

### I - Detection Model: YOLOv8x

**YOLO (You Only Look Once)** is a pre-trained model developed by **Ultralytics** that enables real-time object detection. Our module uses the **YOLOv8x** model, the **largest and most accurate** version of the YOLOv8 family.

* YOLOv8x can recognize up to **80 object classes**
* It offers an excellent trade-off between **speed** and **accuracy**, making it particularly suited for industrial environments.
* **YOLOv8x** was chosen for its **ability to maximize detection precision**, even in low-contrast images or when detecting small objects.

### II - Dominant Color Identification

A key feature of this module is its **ability to automatically and robustly identify the dominant color** of the detected object.

### Process Steps:

1. **Object detection in the image**

   The image captured by the camera is analyzed by **YOLOv8x**. Once an object is detected, its **bounding box coordinates** are extracted, precisely locating the object's position in the image and ignoring the rest (background, other objects, visual noise).

2.  **Image cropping around the object**

   Using the bounding box coordinates, a **sub-image**, called the **region of interest**, is extracted. This keeps only the detected object, improving the reliability of color detection.

3. **Color analysis using KMeans**

   Pixels from the **region of interest** are analyzed using the **KMeans clustering algorithm** (via the `scikit-learn` library). This groups pixels by color similarity to identify the **dominant color**.

4. **Color interpretation**

   The extracted dominant color is an **RGB** value (e.g. `[0, 255, 0]`). This value is then compared to a dictionary of standard colors (red, green, blue, etc.) to deduce an **approximate and understandable color name** (e.g. `"green"`).

### Data Flow

`Image → YOLO Detection → Color Analysis → Result`

### 🛠 Technologies Used

* `OpenCV`: image reading
* `Ultralytics`: loading the YOLOv8x model
* `scikit-learn`: dominant color extraction using KMeans
* `NumPy`: pixel data manipulation

## Output Results

The module returns the following information:

* **RGB values**: Digital color components
* **Color name**: (red, blue, green, etc.)
* **Confidence level**: Detection reliability (percentage)
* **Detection coordinates**: Object location in the image

### Images 

[![IA images](/week4/images/Images_IA/Red_box.jpeg)](/week4/images/Images_IA/Red_box.jpeg)
[![IA images](/week4/images/Images_IA/Red_box_analyse.png)](/week4/images/Images_IA/Red_box_analyse.png)
[![IA images](/week4/images/Images_IA/Red_box_result.png)](/week4/images/Images_IA/Red_box_result.png)
[![IA images](/week4/images/Images_IA/Blue_box.jpeg)](/week4/images/Images_IA/Blue_box.jpeg)
[![IA images](/week4/images/Images_IA/Blue_box_analyse.png)](/week4/images/Images_IA/Blue_box_analyse.png)
[![IA images](/week4/images/Images_IA/Blue_box_result.png)](/week4/images/Images_IA/Blue_box_result.png)

## Code Documentation

### 1. Imports and Dependencies

```python
import cv2
import numpy as np
from ultralytics import YOLO
from PIL import Image
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
```

**Role:** Working environment configuration

* `cv2`: Image processing (OpenCV)
* `numpy`: Pixel-level numerical calculations
* `ultralytics`: YOLO object detection model
* `sklearn.cluster`: KMeans algorithm for color analysis
* `matplotlib`: Result visualization

### 2. Function `load_image()`

```python
def load_image(image_path):
    """Load an image from a file path"""
    image = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image_rgb
```

**Role:** Image loading and preparation

* Reads image from a given path
* Converts from BGR (default in OpenCV) to RGB (standard), as YOLO expects RGB
* Returns the processed image

### 3. Function `detect_objects()`

```python
def detect_objects(image, model_path='yolov8x.pt'):
    """Detect objects with YOLO"""
    model = YOLO(model_path)
    results = model(image)
    return results[0]
```

**Role:** YOLOv8x object detection

* Loads the pre-trained YOLOv8x model
* Applies it to the input image
* Returns detection results (bounding boxes, classes, confidences)

### 4. Function `filter_objects_by_area()`

```python
def filter_objects_by_area(results, min_area_ratio=0.001, max_area_ratio=0.3):
    """Filter objects by bounding box area"""
    image_area = results.orig_shape[0] * results.orig_shape[1]
    filtered_boxes = []

    if results.boxes is not None:
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            box_area = (x2 - x1) * (y2 - y1)
            area_ratio = box_area / image_area

            if min_area_ratio <= area_ratio <= max_area_ratio:
                filtered_boxes.append({
                    'box': [int(x1), int(y1), int(x2), int(y2)],
                    'area': box_area,
                    'confidence': float(box.conf[0]),
                    'class': int(box.cls[0])
                })

    filtered_boxes.sort(key=lambda x: x['area'])
    return filtered_boxes
```

**Role:** Intelligent object selection

* Calculates the area of each bounding box
* Filters out too-small or too-large detections
* Sorts objects by ascending area
* Returns the most relevant objects

### 5. Function `extract_dominant_color()`

```python
def extract_dominant_color(image, bbox, n_colors=1):
    """Extract the dominant color from a region of the image"""
    x1, y1, x2, y2 = bbox
    roi = image[y1:y2, x1:x2]
    roi_resized = cv2.resize(roi, (50, 50))
    pixels = roi_resized.reshape(-1, 3)
    kmeans = KMeans(n_clusters=n_colors, random_state=42, n_init=10)
    kmeans.fit(pixels)
    dominant_color = kmeans.cluster_centers_[0].astype(int)
    return dominant_color
```

**Role:** Object color analysis

* Crops the detected object
* Optimizes by resizing to 50x50 pixels
* Applies KMeans to find the main color
* Returns RGB value of the dominant color

### 6. Function `rgb_to_name()`

```python
def rgb_to_name(rgb):
    """Convert RGB color to approximate color name"""
    r, g, b = rgb
    colors = {
        'red': [255, 0, 0],
        'green': [0, 255, 0],
        'blue': [0, 0, 255],
        'yellow': [255, 255, 0],
        'magenta': [255, 0, 255],
        'white': [255, 255, 255],
        'black': [0, 0, 0],
        'gray': [128, 128, 128],
        'orange': [255, 165, 0],
        'purple': [128, 0, 128],
        'pink': [255, 192, 203],
        'brown': [165, 42, 42]
    }

    min_distance = float('inf')
    closest_color = 'unknown'

    for color_name, color_rgb in colors.items():
        distance = np.sqrt(sum([(a - b) ** 2 for a, b in zip(rgb, color_rgb)]))
        if distance < min_distance:
            min_distance = distance
            closest_color = color_name

    return closest_color
```

**Role:** Approximate color naming

* Compares RGB value to reference colors
* Calculates Euclidean distance
* Returns closest match name

### 7. Function `visualize_result()`

```python
def visualize_result(image, bbox, color, color_name):
    """Visualize the result with bounding box"""
    plt.figure(figsize=(8, 6))
    image_copy = image.copy()
    x1, y1, x2, y2 = bbox
    cv2.rectangle(image_copy, (x1, y1), (x2, y2), (0, 255, 0), 7)
    text = "OBJECT DETECTED"
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 2
    thickness = 7
    (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    cv2.rectangle(image_copy,
                  (x1, y1 - text_height - baseline - 10),
                  (x1 + text_width + 10, y1 - 5),
                  (0, 255, 0), -1)
    cv2.putText(
        image_copy,
        text,
        (x1 + 5, y1 - 10),
        font,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_AA
    )
    plt.imshow(image_copy)
    plt.title(f'Detected Object - Color: {color_name}')
    plt.axis('off')
    plt.show()
```

**Role:** Display detection results visually

* Draws bounding box and label
* Displays annotated image
* Helps validate detection quality

copy)

&#x20;   plt.title(f'Detected Object - Color: {color\_name}')

&#x20;   plt.axis('off')

&#x20;   plt.show()
