"""Edge detection algorithms for die scanner production package."""

import numpy as np
import time
from typing import List, Optional, Tuple
import logging

from ..models.data_models import ScanPoint, EdgePoint, SafetyParameters
from ..exceptions.custom_exceptions import EdgeDetectionError, ValidationError
from ..utils.geometric_calculations import calculate_distance


class EdgeDetector:
    """Production edge detection with enhanced safety validation."""
    
    def __init__(self, safety_params: SafetyParameters, base_threshold: float = 2.0):
        """
        Initialize edge detector.
        
        Args:
            safety_params: Safety validation parameters
            base_threshold: Base height threshold for edge detection (mm)
        """
        self.safety_params = safety_params
        self.base_threshold = base_threshold
        self.logger = logging.getLogger(__name__)
        
        # Edge detection parameters
        self.adaptive_factor = 1.5
        self.confirmation_points = 3
        self.max_height_buffer = 10
        
        # Detection state
        self.recent_heights: List[float] = []
        self.pending_edges: List[dict] = []
        
    def detect_edge(self, current_point: ScanPoint, previous_height: Optional[float]) -> Optional[EdgePoint]:
        """
        Detect edge at current scan point.
        
        Args:
            current_point: Current scan measurement
            previous_height: Previous valid height measurement
            
        Returns:
            EdgePoint if edge detected, None otherwise
        """
        if not current_point.is_valid or current_point.height <= -999:
            return None
            
        if previous_height is None or previous_height <= -999:
            self._update_height_buffer(current_point.height)
            return None
        
        # Calculate height difference
        height_diff = current_point.height - previous_height
        
        # Update rolling height buffer
        self._update_height_buffer(current_point.height)
        
        # Calculate adaptive threshold
        adaptive_threshold = self._calculate_adaptive_threshold()
        
        # Check for significant height change
        if abs(height_diff) >= adaptive_threshold:
            # Calculate edge quality
            quality = self._calculate_edge_quality(
                current_point.x, current_point.y, 
                current_point.height, previous_height, height_diff
            )
            
            # Determine edge type
            edge_type = "drop_off" if height_diff < 0 else "rise"
            
            # Create potential edge point
            edge_point = EdgePoint(
                x=current_point.x,
                y=current_point.y,
                edge_type=edge_type,
                confidence=quality
            )
            
            # Apply quality filter
            if quality >= self.safety_params.min_edge_quality:
                # Multi-point confirmation
                if self._confirm_edge_point(edge_point):
                    return edge_point
        
        return None
    
    def _update_height_buffer(self, height: float) -> None:
        """Update rolling height buffer for variance calculation."""
        self.recent_heights.append(height)
        if len(self.recent_heights) > self.max_height_buffer:
            self.recent_heights.pop(0)
    
    def _calculate_adaptive_threshold(self) -> float:
        """Calculate adaptive threshold based on local variance."""
        if len(self.recent_heights) < 3:
            return self.base_threshold
        
        local_variance = np.var(self.recent_heights)
        adaptive_threshold = self.base_threshold + (local_variance * self.adaptive_factor)
        
        # Clamp threshold to reasonable bounds
        min_threshold = self.base_threshold * 0.5
        max_threshold = self.base_threshold * 3.0
        
        return max(min_threshold, min(max_threshold, adaptive_threshold))
    
    def _calculate_edge_quality(self, x: float, y: float, height: float, 
                              previous_height: float, height_diff: float) -> float:
        """
        Calculate edge quality score.
        
        Returns:
            Quality score between 0.0 and 1.0
        """
        quality = 0.0
        
        # Factor 1: Height difference magnitude (40% weight)
        height_factor = min(abs(height_diff) / (self.base_threshold * 2), 1.0)
        quality += height_factor * 0.4
        
        # Factor 2: Gradient strength (30% weight)
        gradient_strength = self._calculate_gradient_strength()
        gradient_factor = min(gradient_strength / 2.0, 1.0)
        quality += gradient_factor * 0.3
        
        # Factor 3: Noise assessment (20% weight)
        if len(self.recent_heights) >= 5:
            recent_std = np.std(self.recent_heights[-5:])
            noise_factor = max(0, 1.0 - (recent_std / 1.0))
            quality += noise_factor * 0.2
        else:
            quality += 0.1
        
        # Factor 4: Consistency check (10% weight)
        quality += 0.1  # Baseline consistency score
        
        return min(quality, 1.0)
    
    def _calculate_gradient_strength(self) -> float:
        """Calculate gradient strength from recent measurements."""
        if len(self.recent_heights) < 3:
            return 0.0
        
        heights = np.array(self.recent_heights[-3:])
        gradient = np.gradient(heights)
        return np.mean(np.abs(gradient))
    
    def _confirm_edge_point(self, edge_point: EdgePoint) -> bool:
        """
        Multi-point edge confirmation.
        
        Args:
            edge_point: Potential edge point
            
        Returns:
            True if edge is confirmed
        """
        # Add to pending confirmation list
        edge_data = {
            'point': edge_point,
            'timestamp': time.time(),
            'confirmed_count': 1
        }
        
        # Check for nearby pending edges
        confirmation_distance = 5.0  # mm
        
        for pending in self.pending_edges:
            distance = calculate_distance(
                edge_point.x, edge_point.y,
                pending['point'].x, pending['point'].y
            )
            
            if (distance <= confirmation_distance and 
                edge_point.edge_type == pending['point'].edge_type):
                pending['confirmed_count'] += 1
                
                if pending['confirmed_count'] >= self.confirmation_points:
                    # Remove confirmed edge from pending
                    self.pending_edges.remove(pending)
                    return True
        
        # Add to pending list if not confirmed
        self.pending_edges.append(edge_data)
        
        # Clean up old pending edges
        current_time = time.time()
        self.pending_edges = [
            p for p in self.pending_edges 
            if current_time - p['timestamp'] < 10.0
        ]
        
        # Accept high-confidence edges immediately
        if edge_point.confidence > 0.9:
            return True
        
        return False
    
    def filter_edges_by_radius(self, edge_points: List[EdgePoint], 
                              center_x: float, center_y: float) -> Tuple[List[EdgePoint], List[EdgePoint]]:
        """
        Filter edges by radius to separate outer perimeter from interior edges.
        
        Args:
            edge_points: List of detected edges
            center_x, center_y: Estimated center coordinates
            
        Returns:
            Tuple of (outer_edges, interior_edges)
        """
        outer_edges = []
        interior_edges = []
        
        for edge in edge_points:
            distance = calculate_distance(edge.x, edge.y, center_x, center_y)
            
            if (distance >= self.safety_params.min_outer_radius and 
                distance <= self.safety_params.max_outer_radius):
                outer_edges.append(edge)
            else:
                interior_edges.append(edge)
        
        return outer_edges, interior_edges
    
    def validate_edge_distribution(self, edge_points: List[EdgePoint], 
                                 center_x: float, center_y: float) -> None:
        """
        Validate edge point distribution for safety.
        
        Raises ValidationError if distribution is unsafe.
        """
        if len(edge_points) < self.safety_params.min_perimeter_edges:
            raise ValidationError(
                f"Insufficient edges: {len(edge_points)} < {self.safety_params.min_perimeter_edges}"
            )
        
        # Check angular distribution
        angles = []
        for edge in edge_points:
            angle = np.arctan2(edge.y - center_y, edge.x - center_x)
            angles.append(angle)
        
        angles = sorted(angles)
        
        # Find largest gap
        gaps = []
        for i in range(len(angles)):
            next_i = (i + 1) % len(angles)
            gap = angles[next_i] - angles[i]
            if gap < 0:
                gap += 2 * np.pi
            gaps.append(gap)
        
        largest_gap = max(gaps)
        coverage = (2 * np.pi - largest_gap) * 180 / np.pi
        
        if coverage < self.safety_params.min_angular_coverage:
            raise ValidationError(
                f"Insufficient angular coverage: {coverage:.1f}° < {self.safety_params.min_angular_coverage}°"
            )