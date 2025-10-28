#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from dataclasses import dataclass


@dataclass
class Detection():
    contour: list[np.ndarray]
    box: np.ndarray
    dimensions: tuple[int, int]
    area: float
    score: float

    def __str__(self):
        return f'Contour: {self.contour}, Box: {self.box}, Dimensions: {self.dimensions}, \
                Area: {self.area}, Score: {self.score}'


@dataclass
class Dimensions():
    width: float
    height: float
    distance: float
    confidence: float

    def __str__(self):
        return f'Width: {self.width:.2f}m, Length: {self.height:.2f}m, \
                Distance: {self.distance:.2f}m, Confidence: {self.confidence:.2f}'


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.bridge = CvBridge()
        
        # RGB image subscription
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/realsense2_camera/color/image_raw',
            self.image_callback,
            10)
        
        # Depth image subscription
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/realsense2_camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        # Camera info subscription for intrinsics
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/realsense2_camera/color/camera_info',
            self.camera_info_callback,
            10)
        
        # Rectangle detection parameters
        self.min_area_ratio = 0.03
        self.score_threshold = 0.05
        
        # Temporal filtering
        self.stable_detection: Detection = None
        self.stable_count = 0
        self.stability_threshold = 5
        self.dimension_tolerance = 0.15
        
        # Depth data and dimension tracking
        self.current_depth_map = None
        self.stable_dimensions: Dimensions = None
        self.dimension_stable_count = 0
        
        # Camera intrinsics (will be updated from camera_info)
        self.fx = 615.0  # Default fallback
        self.fy = 615.0  # Default fallback
        self.cx = 320.0  # Default fallback
        self.cy = 240.0  # Default fallback
        self.depth_scale = 0.001
        self.intrinsics_received = False

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Only log occasionally to avoid spam
        if hasattr(self, '_rgb_count'):
            self._rgb_count += 1
        else:
            self._rgb_count = 1
            self.get_logger().info('RGB stream active')
        
        # Detect rectangles
        detection = self.detect_rectangle(cv_image)
        detection = self.apply_temporal_filter(detection)
        
        # Estimate dimensions using depth if available
        dimensions = None
        if detection and detection.score > self.score_threshold and self.current_depth_map is not None:
            dimensions = self.estimate_dimensions_from_depth(detection, self.current_depth_map)
            # Track stable dimensions
            if dimensions:
                self.track_stable_dimensions(dimensions)
        
        # Draw overlay
        display_frame = self.draw_overlay(cv_image, detection, dimensions)
        
        cv2.imshow('RealSense RGB with Rectangle Detection', display_frame)
        cv2.waitKey(1)
    
    def depth_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.current_depth_map = depth_image.astype(np.float32) * self.depth_scale
            # Only log occasionally to avoid spam
            if hasattr(self, '_depth_count'):
                self._depth_count += 1
                if self._depth_count % 30 == 0:  # Log every 30 frames
                    depth_stats = f'min={np.min(self.current_depth_map):.3f}, max={np.max(self.current_depth_map):.3f}, mean={np.mean(self.current_depth_map):.3f}'
                    self.get_logger().info(f'Depth stats: {depth_stats}')
            else:
                self._depth_count = 1
                self.get_logger().info(f'Depth stream active - shape: {self.current_depth_map.shape}')
        except Exception as e:
            self.get_logger().warn(f'Failed to process depth image: {e}')
    
    def camera_info_callback(self, msg):
        """Get camera intrinsics from camera info"""
        if not self.intrinsics_received:
            self.fx = msg.k[0]  # K[0] = fx
            self.fy = msg.k[4]  # K[4] = fy
            self.cx = msg.k[2]  # K[2] = cx
            self.cy = msg.k[5]  # K[5] = cy
            self.intrinsics_received = True
            self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def detect_rectangle(self, frame: np.ndarray) -> Detection:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # More aggressive smoothing to reduce noise
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        mean_val = np.mean(blurred)
        low_thresh = max(30, int(mean_val * 0.4))
        high_thresh = min(120, int(mean_val * 1.0))
        
        edges = cv2.Canny(blurred, low_thresh, high_thresh)
        # Stronger morphological operations to connect edges
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        edges_final = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=3)
        
        contours, _ = cv2.findContours(edges_final, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = int(w * h * self.min_area_ratio)
        
        best_contour = None
        best_score = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            
            x, y, w_rect, h_rect = cv2.boundingRect(contour)
            area_ratio = area / (w * h)
            aspect_ratio = max(w_rect, h_rect) / min(w_rect, h_rect) if min(w_rect, h_rect) > 0 else 0
            
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            solidity = area / hull_area if hull_area > 0 else 0
            
            if (area_ratio >= self.min_area_ratio and 
                0.4 <= aspect_ratio <= 4.0 and
                solidity > 0.5):
                
                score = area_ratio * solidity
                
                if score > best_score:
                    best_score = score
                    best_contour = contour
        
        if best_contour is None:
            return None
        
        # Always use minimum area rectangle for smoother, less noisy detection
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
            
            w_diff = abs(current_dims[0] - stable_dims[0]) / stable_dims[0]
            h_diff = abs(current_dims[1] - stable_dims[1]) / stable_dims[1]
            
            if w_diff < self.dimension_tolerance and h_diff < self.dimension_tolerance:
                self.stable_count += 1
                # Don't freeze the rectangle, just track stability for dimensions
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
            box = detection.box.astype(float)
            
            # Use average of fx and fy as focal length (they are usually very close)
            focal_length_pixels = (self.fx + self.fy) / 2.0
            
            if focal_length_pixels == 0:
                return None
            
            real_distances = []
            
            # Calculate real distances between adjacent corners using depth
            for i in range(4):
                p1, p2 = box[i], box[(i+1) % 4]
                
                # Sample multiple points along the edge to find valid depth
                edge_depths = []
                num_samples = 10
                
                for t in np.linspace(0, 1, num_samples):
                    x = int(np.clip(p1[0] + t * (p2[0] - p1[0]), 0, depth_w - 1))
                    y = int(np.clip(p1[1] + t * (p2[1] - p1[1]), 0, depth_h - 1))
                    depth_val = depth_map[y, x]
                    if 0.1 < depth_val < 10.0:  # Valid depth range
                        edge_depths.append(depth_val)
                
                # Skip if not enough valid depth samples along this edge
                if len(edge_depths) < 3:
                    self.get_logger().info(f'Edge {i}: insufficient depth samples ({len(edge_depths)}/10)')
                    continue
                    
                # Use median depth for this edge (more robust than mean)
                avg_edge_depth = np.median(edge_depths)
                
                # Calculate pixel distance between corners
                pixel_dist = np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                
                # Convert to real-world distance using the correct formula
                real_dist = (pixel_dist * avg_edge_depth) / focal_length_pixels
                real_distances.append(real_dist)
                
                self.get_logger().info(f'Edge {i}: pixel_dist={pixel_dist:.1f}, avg_depth={avg_edge_depth:.3f}, real_dist={real_dist:.3f} (from {len(edge_depths)} samples)')
            
            if len(real_distances) < 2:
                self.get_logger().warn(f'Not enough valid distances: {len(real_distances)}/4')
                return None
            
            # Handle cases where we don't have all 4 edges
            if len(real_distances) == 4:
                # Ideal case - opposite edges should be similar, average them
                width_m = (real_distances[0] + real_distances[2]) / 2.0
                height_m = (real_distances[1] + real_distances[3]) / 2.0
            elif len(real_distances) == 3:
                # Use available measurements
                width_m = real_distances[0] if 0 in range(len(real_distances)) else real_distances[2] if 2 < len(real_distances) else (real_distances[0] + real_distances[2]) / 2.0
                height_m = real_distances[1] if 1 in range(len(real_distances)) else real_distances[3] if 3 < len(real_distances) else (real_distances[1] + real_distances[3]) / 2.0
            else:
                # Use what we have - assume rectangular
                width_m = real_distances[0]
                height_m = real_distances[1] if len(real_distances) > 1 else real_distances[0]
            
            # Find median depth of corners for distance measurement
            corner_depths = []
            for corner in box:
                x = int(np.clip(corner[0], 0, depth_w - 1))
                y = int(np.clip(corner[1], 0, depth_h - 1))
                depth_val = depth_map[y, x]
                if depth_val > 0:
                    corner_depths.append(depth_val)
            
            if not corner_depths:
                return None
                
            median_depth = np.median(corner_depths)
            
            result = Dimensions(round(float(width_m), 3), round(float(height_m), 3), 
                                round(float(median_depth), 3), detection.score)
            
            self.get_logger().info(f'Calculated dimensions: {result} (focal_length={focal_length_pixels:.1f})')
            return result
            
        except Exception as e:
            self.get_logger().warn(f'Error estimating dimensions: {e}')
            return None
    
    def track_stable_dimensions(self, dimensions: Dimensions):
        """Track when dimensions become stable for output"""
        if self.stable_dimensions is None:
            self.stable_dimensions = dimensions
            self.dimension_stable_count = 1
            return
        
        # Check if dimensions are similar to stable ones
        w_diff = abs(dimensions.width - self.stable_dimensions.width) / self.stable_dimensions.width
        h_diff = abs(dimensions.height - self.stable_dimensions.height) / self.stable_dimensions.height
        
        if w_diff < 0.1 and h_diff < 0.1:  # 10% tolerance
            self.dimension_stable_count += 1
            if self.dimension_stable_count == self.stability_threshold:
                # Output stable dimensions
                self.get_logger().info(f"STABLE DIMENSIONS: {self.stable_dimensions}")
        else:
            self.stable_dimensions = dimensions
            self.dimension_stable_count = 1
    
    def draw_overlay(self, frame: np.ndarray, detection: Detection, dimensions: Dimensions = None) -> np.ndarray:
        overlay = frame.copy()
        
        if detection is None:
            cv2.putText(overlay, "Searching for rectangles...", (20, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            depth_status = "Depth: Available" if self.current_depth_map is not None else "Depth: Waiting..."
            cv2.putText(overlay, depth_status, (20, overlay.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            return overlay
        
        cv2.drawContours(overlay, [detection.contour], -1, (0, 255, 0), 2)
        cv2.drawContours(overlay, [detection.box], -1, (255, 0, 0), 3)
        
        for i, corner in enumerate(detection.box):
            cv2.circle(overlay, tuple(corner), 8, (0, 0, 255), -1)
            cv2.putText(overlay, str(i+1), tuple(corner + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_texts = [
            f"Score: {detection.score:.3f}",
            f"Pixels: {detection.dimensions[0]:.0f} x {detection.dimensions[1]:.0f}",
            f"Rect Stable: {self.stable_count}/{self.stability_threshold}"
        ]
        
        if dimensions:
            info_texts.extend([
                f"Width: {dimensions.width:.3f}m",
                f"Height: {dimensions.height:.3f}m",
                f"Distance: {dimensions.distance:.2f}m",
                f"Dims Stable: {self.dimension_stable_count}/{self.stability_threshold}"
            ])
            # Show if dimensions are stable
            if self.dimension_stable_count >= self.stability_threshold:
                cv2.putText(overlay, "DIMENSIONS STABLE!", (20, overlay.shape[0] - 60), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
        else:
            depth_available = "YES" if self.current_depth_map is not None else "NO"
            intrinsics_available = "YES" if self.intrinsics_received else "NO"
            info_texts.extend([
                f"Depth Available: {depth_available}",
                f"Intrinsics: {intrinsics_available}"
            ])
            if not self.intrinsics_received:
                info_texts.append("Waiting for camera info...")
            elif self.current_depth_map is None:
                info_texts.append("Waiting for depth data...")
        
        for i, text in enumerate(info_texts):
            cv2.putText(overlay, text, (20, 30 + i * 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Status indicators
        depth_status = "Depth: Available" if self.current_depth_map is not None else "Depth: Waiting..."
        intrinsics_status = f"Intrinsics: fx={self.fx:.0f}" if self.intrinsics_received else "Intrinsics: Waiting..."
        cv2.putText(overlay, depth_status, (20, overlay.shape[0] - 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(overlay, intrinsics_status, (20, overlay.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
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
