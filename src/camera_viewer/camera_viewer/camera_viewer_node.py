#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class Detection:
    contour: list[np.ndarray]
    box: np.ndarray
    dimensions: tuple[int, int]
    area: float
    score: float


@dataclass
class Dimensions:
    width: float
    height: float
    distance: float
    confidence: float

    def __str__(self):
        return f'Width: {self.width:.3f}m, Height: {self.height:.3f}m, Distance: {self.distance:.2f}m'


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        
        # Subscriptions
        self.rgb_subscription = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_subscription = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        
        # Detection parameters
        self.min_area_ratio = 0.03
        self.score_threshold = 0.02
        
        # Temporal filtering
        self.stable_detection: Detection = None
        self.stable_count = 0
        self.stability_threshold = 5
        self.dimension_tolerance = 0.15
        
        # State
        self.current_depth_map = None
        self.stable_dimensions: Dimensions = None
        self.dimension_stable_count = 0
        
        # Camera intrinsics
        self.fx = self.fy = 615.0
        self.cx = self.cy = 320.0
        self.intrinsics_received = False
        
        # Remove A6-specific dimensions - now general purpose

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Detect and filter rectangles
        detection = self.detect_rectangle(cv_image)
        detection = self.apply_temporal_filter(detection)
        
        # Estimate dimensions if detection is valid
        dimensions = None
        if detection and detection.score > self.score_threshold and self.current_depth_map is not None:
            dimensions = self.estimate_dimensions_from_depth(detection, self.current_depth_map)
            if dimensions:
                self.track_stable_dimensions(dimensions)
        
        # Display results
        display_frame = self.draw_overlay(cv_image, detection, dimensions)
        cv2.imshow('Rectangle Detection', display_frame)
        cv2.waitKey(1)
    
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_depth_map = depth_image.astype(np.float32) / 1000.0  # Convert mm to meters
        except Exception as e:
            self.get_logger().warn(f'Failed to process depth image: {e}')
    
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        if not self.intrinsics_received:
            self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}')
            self.intrinsics_received = True

    def detect_rectangle(self, frame: np.ndarray) -> Detection:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Edge detection with noise reduction
        filtered = cv2.bilateralFilter(gray, 9, 75, 75)
        mean_val = np.mean(filtered)
        low_thresh = max(30, int(mean_val * 0.4))
        high_thresh = min(180, int(mean_val * 1.3))
        edges = cv2.Canny(filtered, low_thresh, high_thresh)
        
        # Morphological operations to connect edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        edges = cv2.dilate(edges, kernel, iterations=1)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = int(w * h * self.min_area_ratio)
        
        best_contour = None
        best_score = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            # Approximate contour
            epsilon = 0.015 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) < 4 or len(approx) > 10:
                continue
            
            # Calculate metrics
            min_rect = cv2.minAreaRect(contour)
            rect_w, rect_h = min_rect[1]
            if rect_w * rect_h == 0:
                continue
                
            area_ratio = area / (w * h)
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
            rectangularity = area / (rect_w * rect_h)
            
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            # Filter for rectangular shapes
            if (area_ratio >= self.min_area_ratio and 
                1.0 <= aspect_ratio <= 4.0 and
                rectangularity > 0.6 and 
                solidity > 0.7):
                
                score = area_ratio * rectangularity * solidity
                
                if score > best_score:
                    best_score = score
                    best_contour = contour
        
        if best_contour is None:
            return None
        
        # Create detection result
        min_rect = cv2.minAreaRect(best_contour)
        box = cv2.boxPoints(min_rect)
        box = np.array(box, dtype=np.int32)
        rect_w, rect_h = min_rect[1]
        
        if rect_w < rect_h:
            rect_w, rect_h = rect_h, rect_w
        
        return Detection(best_contour, box, (rect_w, rect_h), cv2.contourArea(best_contour), best_score)
    
    def apply_temporal_filter(self, detection: Detection) -> Detection:
        if detection is None or detection.score < self.score_threshold:
            self.stable_count = 0
            return detection
        
        current_dims = detection.dimensions
        
        if self.stable_detection is not None:
            stable_dims = self.stable_detection.dimensions
            
            # Check both orientations to prevent flickering
            w_diff1 = abs(current_dims[0] - stable_dims[0]) / stable_dims[0]
            h_diff1 = abs(current_dims[1] - stable_dims[1]) / stable_dims[1]
            w_diff2 = abs(current_dims[0] - stable_dims[1]) / stable_dims[1]
            h_diff2 = abs(current_dims[1] - stable_dims[0]) / stable_dims[0]
            
            # Use the orientation that matches better
            if (w_diff1 + h_diff1) <= (w_diff2 + h_diff2):
                if w_diff1 < self.dimension_tolerance and h_diff1 < self.dimension_tolerance:
                    self.stable_count += 1
                else:
                    self.stable_count = 0
                    self.stable_detection = detection
            else:
                # Flip to match stable orientation
                flipped_detection = Detection(
                    detection.contour, detection.box, (stable_dims[0], stable_dims[1]),
                    detection.area, detection.score)
                if w_diff2 < self.dimension_tolerance and h_diff2 < self.dimension_tolerance:
                    self.stable_count += 1
                    return flipped_detection
                else:
                    self.stable_count = 0
                    self.stable_detection = detection
        else:
            self.stable_detection = detection
            self.stable_count = 1
        
        return detection
    
    def estimate_dimensions_from_depth(self, detection: Detection, depth_map: np.ndarray) -> Dimensions:
        try:
            depth_h, depth_w = depth_map.shape[:2]
            box = detection.box.astype(int)
            
            # Calculate real distances between adjacent corners
            real_distances = []
            corner_depths = []
            
            for i in range(4):
                p1, p2 = box[i], box[(i+1) % 4]
                
                # Get depth at corners with bounds checking
                x1 = int(np.clip(p1[0], 0, depth_w-1))
                y1 = int(np.clip(p1[1], 0, depth_h-1))
                x2 = int(np.clip(p2[0], 0, depth_w-1))
                y2 = int(np.clip(p2[1], 0, depth_h-1))
                
                d1, d2 = depth_map[y1, x1], depth_map[y2, x2]
                
                # Skip if depths are invalid
                if d1 <= 0 or d2 <= 0 or d1 > 10.0 or d2 > 10.0:
                    continue
                    
                corner_depths.extend([d1, d2])
                
                # Convert pixel distance to real-world distance
                pixel_dist = np.linalg.norm(p1.astype(float) - p2.astype(float))
                avg_edge_depth = (d1 + d2) / 2.0
                focal_avg = (self.fx + self.fy) / 2.0
                real_dist = (pixel_dist * avg_edge_depth) / focal_avg
                real_distances.append(real_dist)
            
            if len(real_distances) < 2:
                return None
            
            # Group opposite edges and assign dimensions
            if len(real_distances) >= 4:
                # Group opposite edges (0,2 and 1,3)
                dim1 = (real_distances[0] + real_distances[2]) / 2.0
                dim2 = (real_distances[1] + real_distances[3]) / 2.0
                
                # Assign dimensions (smaller as width, larger as height)
                width_m = min(dim1, dim2)
                height_m = max(dim1, dim2)
            else:
                width_m = min(real_distances[0], real_distances[1])
                height_m = max(real_distances[0], real_distances[1])
            
            avg_distance = np.median(corner_depths) if corner_depths else 0
            
            result = Dimensions(
                round(float(width_m), 3),
                round(float(height_m), 3),
                round(float(avg_distance), 3),
                detection.score
            )
            
            # Log results
            self.get_logger().info(f'Dimensions: {result}')
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error estimating dimensions: {e}')
            return None
    
    def track_stable_dimensions(self, dimensions: Dimensions):
        if self.stable_dimensions is None:
            self.stable_dimensions = dimensions
            self.dimension_stable_count = 1
            return
        
        # Check if dimensions are similar
        w_diff = abs(dimensions.width - self.stable_dimensions.width) / self.stable_dimensions.width
        h_diff = abs(dimensions.height - self.stable_dimensions.height) / self.stable_dimensions.height
        
        if w_diff < 0.1 and h_diff < 0.1:  # 10% tolerance
            self.dimension_stable_count += 1
            if self.dimension_stable_count == self.stability_threshold:
                self.get_logger().info(f"🎯 STABLE DIMENSIONS: {self.stable_dimensions}")
        else:
            self.stable_dimensions = dimensions
            self.dimension_stable_count = 1
    
    def draw_overlay(self, frame: np.ndarray, detection: Detection, dimensions: Dimensions = None) -> np.ndarray:
        overlay = frame.copy()
        
        if detection is None:
            cv2.putText(overlay, "Searching for rectangles...", (20, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            return overlay
        
        # Draw detection
        cv2.drawContours(overlay, [detection.contour], -1, (0, 255, 0), 2)
        cv2.drawContours(overlay, [detection.box], -1, (255, 0, 0), 3)
        
        # Draw corner numbers
        for i, corner in enumerate(detection.box):
            cv2.circle(overlay, tuple(corner), 8, (0, 0, 255), -1)
            cv2.putText(overlay, str(i+1), tuple(corner + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Info display
        info_texts = [
            f"Score: {detection.score:.3f}",
            f"Pixels: {detection.dimensions[0]:.0f} x {detection.dimensions[1]:.0f}",
            f"Stable: {self.stable_count}/{self.stability_threshold}"
        ]
        
        if dimensions:
            info_texts.extend([
                f"Width: {dimensions.width*1000:.0f}mm",
                f"Height: {dimensions.height*1000:.0f}mm", 
                f"Distance: {dimensions.distance:.2f}m",
                f"Dims Stable: {self.dimension_stable_count}/{self.stability_threshold}"
            ])
            
            # Stability indicator
            if self.dimension_stable_count >= self.stability_threshold:
                cv2.putText(overlay, "DIMENSIONS STABLE!", (20, overlay.shape[0] - 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
        
        # Draw info text
        for i, text in enumerate(info_texts):
            cv2.putText(overlay, text, (20, 30 + i * 25), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        return overlay


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()