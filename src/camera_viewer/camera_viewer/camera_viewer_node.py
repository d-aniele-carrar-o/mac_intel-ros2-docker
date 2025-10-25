#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Dict, Any

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
        
        # Rectangle detection parameters
        self.min_area_ratio = 0.03
        self.score_threshold = 0.05
        
        # Temporal filtering
        self.stable_detection = None
        self.stable_count = 0
        self.stability_threshold = 5
        self.dimension_tolerance = 0.15
        
        # Depth data
        self.current_depth_map = None
        
        # RealSense camera parameters
        self.focal_length_pixels = 615.0
        self.depth_scale = 0.001

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Detect rectangles
        detection = self.detect_rectangle(cv_image)
        detection = self.apply_temporal_filter(detection)
        
        # Estimate dimensions using depth if available
        dimensions = None
        if detection and detection['score'] > self.score_threshold and self.current_depth_map is not None:
            dimensions = self.estimate_dimensions_from_depth(detection, self.current_depth_map)
        
        # Draw overlay
        display_frame = self.draw_overlay(cv_image, detection, dimensions)
        
        cv2.imshow('RealSense RGB with Rectangle Detection', display_frame)
        cv2.waitKey(1)
    
    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.current_depth_map = depth_image.astype(np.float32) * self.depth_scale

    def detect_rectangle(self, frame: np.ndarray) -> Optional[Dict[str, Any]]:
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        mean_val = np.mean(blurred)
        low_thresh = max(50, int(mean_val * 0.6))
        high_thresh = min(180, int(mean_val * 1.5))
        
        edges = cv2.Canny(blurred, low_thresh, high_thresh)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        edges_final = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
        
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
        
        fitted_corners = self.fit_rectangle_to_contour(best_contour)
        
        if fitted_corners is not None:
            box = fitted_corners.astype(np.int32)
            rect_w = max(
                np.linalg.norm(fitted_corners[1] - fitted_corners[0]),
                np.linalg.norm(fitted_corners[2] - fitted_corners[3])
            )
            rect_h = max(
                np.linalg.norm(fitted_corners[3] - fitted_corners[0]),
                np.linalg.norm(fitted_corners[2] - fitted_corners[1])
            )
        else:
            min_rect = cv2.minAreaRect(best_contour)
            box = cv2.boxPoints(min_rect)
            box = np.array(box, dtype=np.int32)
            rect_w, rect_h = min_rect[1]
        
        if rect_w < rect_h:
            rect_w, rect_h = rect_h, rect_w
        
        return {
            'contour': best_contour,
            'box': box,
            'dimensions': (rect_w, rect_h),
            'area': cv2.contourArea(best_contour),
            'score': best_score
        }
    
    def fit_rectangle_to_contour(self, contour: np.ndarray) -> Optional[np.ndarray]:
        try:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            if len(approx) == 4:
                return self.order_corners(approx.reshape(4, 2))
            
            hull = cv2.convexHull(contour)
            hull_approx = cv2.approxPolyDP(hull, epsilon, True)
            
            if len(hull_approx) == 4:
                return self.order_corners(hull_approx.reshape(4, 2))
                
        except Exception:
            pass
        
        return None
    
    def order_corners(self, pts: np.ndarray) -> np.ndarray:
        pts = pts[np.argsort(pts[:, 1])]
        top = pts[:2][np.argsort(pts[:2, 0])]
        bottom = pts[2:][np.argsort(pts[2:, 0])]
        return np.array([top[0], top[1], bottom[1], bottom[0]], dtype=np.float32)
    
    def apply_temporal_filter(self, detection: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if detection is None or detection['score'] < self.score_threshold:
            self.stable_count = 0
            return detection
        
        current_dims = detection['dimensions']
        
        if self.stable_detection is not None:
            stable_dims = self.stable_detection['dimensions']
            
            w_diff = abs(current_dims[0] - stable_dims[0]) / stable_dims[0]
            h_diff = abs(current_dims[1] - stable_dims[1]) / stable_dims[1]
            
            if w_diff < self.dimension_tolerance and h_diff < self.dimension_tolerance:
                self.stable_count += 1
                if self.stable_count >= self.stability_threshold:
                    return self.stable_detection
            else:
                self.stable_count = 0
                self.stable_detection = None
        
        if detection['score'] > self.score_threshold * 1.5:
            if self.stable_detection is None:
                self.stable_detection = detection
                self.stable_count = 1
        
        return detection
    
    def estimate_dimensions_from_depth(self, detection: Dict[str, Any], depth_map: np.ndarray) -> Optional[Dict[str, float]]:
        try:
            h, w = depth_map.shape[:2]
            box = detection['box']
            
            real_distances = []
            corner_depths = []
            
            for i in range(4):
                p1, p2 = box[i], box[(i+1)%4]
                
                x1 = int(np.clip(p1[0], 0, w-1))
                y1 = int(np.clip(p1[1], 0, h-1))
                x2 = int(np.clip(p2[0], 0, w-1))
                y2 = int(np.clip(p2[1], 0, h-1))
                
                d1, d2 = depth_map[y1, x1], depth_map[y2, x2]
                
                if d1 <= 0 or d2 <= 0 or d1 > 10 or d2 > 10:
                    continue
                    
                corner_depths.extend([d1, d2])
                
                avg_edge_depth = (d1 + d2) / 2
                pixel_dist = np.linalg.norm(p1 - p2)
                real_dist = (pixel_dist * avg_edge_depth) / self.focal_length_pixels
                real_distances.append(real_dist)
            
            if len(real_distances) < 4:
                return None
            
            width_m = (real_distances[0] + real_distances[2]) / 2
            height_m = (real_distances[1] + real_distances[3]) / 2
            
            valid_depths = [d for d in corner_depths if d > 0 and d < 10]
            if not valid_depths:
                return None
                
            avg_distance = np.median(valid_depths)
            
            return {
                'width_m': round(float(width_m), 3),
                'height_m': round(float(height_m), 3),
                'distance_m': round(float(avg_distance), 3),
                'confidence': detection['score']
            }
            
        except Exception as e:
            self.get_logger().warn(f'Error estimating dimensions: {e}')
            return None
    
    def draw_overlay(self, frame: np.ndarray, detection: Optional[Dict[str, Any]], dimensions: Optional[Dict[str, float]] = None) -> np.ndarray:
        overlay = frame.copy()
        
        if detection is None:
            cv2.putText(overlay, "Searching for rectangles...", (20, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            depth_status = "Depth: Available" if self.current_depth_map is not None else "Depth: Waiting..."
            cv2.putText(overlay, depth_status, (20, overlay.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            return overlay
        
        cv2.drawContours(overlay, [detection['contour']], -1, (0, 255, 0), 2)
        cv2.drawContours(overlay, [detection['box']], -1, (255, 0, 0), 3)
        
        for i, corner in enumerate(detection['box']):
            cv2.circle(overlay, tuple(corner), 8, (0, 0, 255), -1)
            cv2.putText(overlay, str(i+1), tuple(corner + 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        info_texts = [
            f"Score: {detection['score']:.3f}",
            f"Pixels: {detection['dimensions'][0]:.0f} x {detection['dimensions'][1]:.0f}",
            f"Stable: {self.stable_count}/{self.stability_threshold}"
        ]
        
        if dimensions:
            info_texts.extend([
                f"Width: {dimensions['width_m']:.3f}m",
                f"Height: {dimensions['height_m']:.3f}m",
                f"Distance: {dimensions['distance_m']:.2f}m"
            ])
        else:
            info_texts.append("Dimensions: Need depth data")
        
        for i, text in enumerate(info_texts):
            cv2.putText(overlay, text, (20, 30 + i * 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        depth_status = "Depth: Available" if self.current_depth_map is not None else "Depth: Waiting..."
        cv2.putText(overlay, depth_status, (20, overlay.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
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