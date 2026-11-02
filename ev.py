import cv2
import numpy as np

# 1. 9 Starting Locations in meters [X, Y] for calibration
ref_meters = np.array([
    [0.000, 0.000],    # S1
    [0.000, 26.427],   # S2
    [10.709, 6.918],   # S3
    [11.210, 17.881],  # S4
    [13.223, 25.658],  # S5
    [-9.428, 15.824],  # S6
    [-8.970, 22.831],  # S7
    [-15.676, 4.991],  # S8
    [7.160, -4.762]    # S9
], dtype=np.float32)

# 2. All remaining points to project [X, Y]
project_points = {
    # Landmarks
    "L1": [3.183, 8.012],   "L2": [7.269, 9.482],   "L3": [7.878, 17.583],
    "L4": [9.225, 22.389],  "L5": [3.518, 23.990],  "L6": [0.882, 16.870],
    "L7": [-3.944, 21.415], "L8": [-5.491, 16.334], "L9": [-7.695, 13.528],
    "L10": [-1.610, 12.602],"L11": [-7.715, 9.721], "L12": [-4.311, 4.442],
    "L13": [-5.720, 28.118],"L14": [-11.438, 5.230],"L15": [6.483, 1.102],
    # Waypoints
    "W1": [1.951, 9.759],   "W2": [4.170, 19.499],  "W3": [7.013, 23.149],
    "W4": [-1.701, 18.665], "W5": [-5.339, 19.034], "W6": [-7.675, 8.197],
    "W7": [-2.108, 7.682],  "W8": [5.939, 6.790],   "W9": [5.088, 13.092],
    # Deep Sampling
    "P1": [-5.997, 25.901]
}

# 3. Color mapping dictionary (OpenCV uses BGR format)
colors = {
    'S': (255, 220, 150),  # Light Blue for starting points (recorded clicks)
    'L': (180, 180, 180),  # Grey for Landmarks
    'W': (150, 180, 255),  # Peach/Orange for Waypoints
    'P': (150, 230, 150)   # Light Green for Deep Sampling
}

ref_pixels = []
x = 18  # Target circle radius

def get_color(name):
    # Match the first letter of the point name to the color dictionary
    return colors.get(name[0], (255, 255, 255))

def calibrate_and_project(event, px, py, flags, param):
    global ref_pixels, img
    
    if event == cv2.EVENT_LBUTTONDOWN and len(ref_pixels) < 9:
        ref_pixels.append([px, py])
        name = f"S{len(ref_pixels)}"
        color = get_color(name)
        
        # Draw the clicked 'S' point
        cv2.circle(img, (px, py), x, color, -1)
        # Use (0, 0, 0) for black text
        cv2.putText(img, name, (px + 5, py - 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        cv2.imshow("Map Projection", img)
        
        # Once all 9 are clicked, project the rest
        if len(ref_pixels) == 9:
            pts_dst = np.array(ref_pixels, dtype=np.float32)
            matrix, _ = cv2.findHomography(ref_meters, pts_dst, cv2.RANSAC)
            
            for pt_name, coords in project_points.items():
                pt_color = get_color(pt_name)
                pt_meters = np.array([[coords]], dtype=np.float32)
                pt_pixels = cv2.perspectiveTransform(pt_meters, matrix)
                
                proj_x, proj_y = int(pt_pixels[0][0][0]), int(pt_pixels[0][0][1])
                
                if 0 <= proj_x < img.shape[1] and 0 <= proj_y < img.shape[0]:
                    cv2.circle(img, (proj_x, proj_y), x, pt_color, -1)
                    # Use (0, 0, 0) for black text
                    cv2.putText(img, pt_name, (proj_x + 5, proj_y - 5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
            
            cv2.imshow("Map Projection", img)
            print("Successfully projected all Landmarks, Waypoints, and Deep Sampling points.")

img = cv2.imread('image_c.png')

print("Click ALL 9 Starting Locations (S1-S9) in exact sequential order:")
print("S1 -> S2 -> S3 -> S4 -> S5 -> S6 -> S7 -> S8 -> S9")

cv2.imshow("Map Projection", img)
cv2.setMouseCallback("Map Projection", calibrate_and_project)
cv2.waitKey(0)
cv2.destroyAllWindows()