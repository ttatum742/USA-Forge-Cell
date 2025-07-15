"""Advanced scanning methods for production die scanner package."""

import numpy as np
import time
import logging
from typing import List, Tuple, Optional, Dict, Any

from ..models.data_models import ScanPoint, EdgePoint, SafetyParameters
from ..exceptions.custom_exceptions import ScanningError, SafetyError, ValidationError
from ..utils.geometric_calculations import calculate_distance, calculate_angular_coverage
from ..services.robot_service import RobotService
from ..services.sensor_service import SensorService


class AdvancedScanningMethods:
    """
    Advanced scanning methods for production die scanner.
    
    Implements safety-critical scanning techniques:
    - Mini-spiral edge refinement
    - Coverage gap analysis and filling
    - Statistical outlier removal
    - Enhanced edge classification
    """
    
    def __init__(self, robot: RobotService, sensor: SensorService, safety_params: SafetyParameters):
        """Initialize advanced scanning methods."""
        self.robot = robot
        self.sensor = sensor
        self.safety_params = safety_params
        self.logger = logging.getLogger(__name__)
        
        # Advanced scanning parameters
        self.spiral_radius = 5.0  # mm
        self.spiral_step = 0.5    # mm high precision step
        self.spiral_turns = 3     # number of spiral turns
        self.points_per_turn = 16 # points per spiral turn
        
        # Gap analysis parameters
        self.gap_threshold = 30.0  # degrees minimum gap to fill
        self.radial_scan_distance = 25.4  # mm (1 inch)
        self.radial_step_size = 0.5  # mm
        
        # Statistical parameters
        self.outlier_std_threshold = 2.0  # standard deviations
        self.quality_threshold = 0.3      # minimum quality score
        
    def perform_spiral_edge_refinement(self, edge_points: List[EdgePoint], 
                                     scan_points: List[ScanPoint]) -> Tuple[List[EdgePoint], List[ScanPoint]]:
        """
        Phase 2: Mini-spiral scanning around detected edges for high-precision refinement.
        
        Args:
            edge_points: Initial edge points to refine
            scan_points: Current scan points list (will be modified)
            
        Returns:
            Tuple of (refined_edge_points, updated_scan_points)
            
        Raises:
            ScanningError: If spiral refinement fails
        """
        if not edge_points:
            self.logger.warning("No edges found for spiral refinement")
            return edge_points, scan_points
        
        self.logger.info("=== Phase 2: Mini-Spiral Edge Refinement ===")
        self.logger.info(f"Starting spiral refinement around {len(edge_points)} detected edges")
        self.logger.info(f"Spiral params: {self.spiral_radius}mm radius, {self.spiral_step}mm step, {self.spiral_turns} turns")
        
        refined_edge_points = list(edge_points)  # Copy original edges
        updated_scan_points = list(scan_points)  # Copy scan points
        refined_edges_added = 0
        
        # Refine each edge with mini-spiral scanning
        for edge_idx, edge_point in enumerate(edge_points):
            try:
                self.logger.debug(f"Refining edge {edge_idx+1}/{len(edge_points)}: ({edge_point.x:.1f}, {edge_point.y:.1f})")
                
                # Perform mini-spiral around this edge
                spiral_edges, spiral_points = self._perform_mini_spiral_scan(
                    center_x=edge_point.x,
                    center_y=edge_point.y,
                    current_scan_points=updated_scan_points
                )
                
                refined_edge_points.extend(spiral_edges)
                updated_scan_points.extend(spiral_points)
                refined_edges_added += len(spiral_edges)
                
                # Log progress every 5 edges
                if (edge_idx + 1) % 5 == 0:
                    self.logger.info(f"Refined {edge_idx+1}/{len(edge_points)} edges, +{len(spiral_edges)} edges around edge {edge_idx+1}")
                    
            except Exception as e:
                self.logger.error(f"Error refining edge {edge_idx+1}: {e}")
                continue
        
        self.logger.info(f"Spiral refinement complete: {len(edge_points)} → {len(refined_edge_points)} edges (+{refined_edges_added})")
        
        return refined_edge_points, updated_scan_points
    
    def _perform_mini_spiral_scan(self, center_x: float, center_y: float, 
                                current_scan_points: List[ScanPoint]) -> Tuple[List[EdgePoint], List[ScanPoint]]:
        """Perform mini-spiral scan around a detected edge point."""
        edges_found = []
        scan_points_added = []
        total_points = int(self.spiral_turns * self.points_per_turn)
        
        try:
            current_pos = self.robot.read_position()
            current_z = current_pos[2]
        except Exception as e:
            raise ScanningError(f"Failed to read robot position for spiral scan: {e}")
        
        last_height = None
        
        self.logger.debug(f"Mini-spiral: center=({center_x:.1f}, {center_y:.1f}), {total_points} points")
        
        for point in range(total_points):
            try:
                # Calculate spiral position
                progress = point / self.points_per_turn  # How many turns completed
                angle = progress * 2 * np.pi  # Angle in radians
                radius = (progress / self.spiral_turns) * self.spiral_radius  # Growing radius
                
                # Calculate actual X,Y position
                scan_x = center_x + radius * np.cos(angle)
                scan_y = center_y + radius * np.sin(angle)
                
                # Move to position
                self.robot.send_move_command(scan_x, scan_y, current_z)
                
                # Wait for move completion (short timeout for small moves)
                if not self.robot.wait_for_status(10, timeout=3.0):
                    self.logger.debug(f"Move timeout in spiral point {point}")
                    continue
                
                # Take measurement
                height = self.sensor.read_height()
                
                if height > -999:
                    # Create scan point
                    scan_point = ScanPoint(
                        x=scan_x, y=scan_y, z=current_z,
                        height=height, is_valid=True, timestamp=time.time()
                    )
                    scan_points_added.append(scan_point)
                    
                    # Detect edges with existing edge detector logic
                    if last_height is not None and abs(height - last_height) >= 2.0:
                        edge_type = "drop_off" if height < last_height else "rise"
                        quality = min(abs(height - last_height) / 4.0, 1.0)  # Simple quality score
                        
                        if quality >= self.safety_params.min_edge_quality:
                            edge_point = EdgePoint(
                                x=scan_x, y=scan_y,
                                edge_type=edge_type,
                                confidence=quality
                            )
                            edges_found.append(edge_point)
                            self.logger.debug(f"Spiral edge found at ({scan_x:.1f}, {scan_y:.1f})")
                    
                    last_height = height
                
            except Exception as e:
                self.logger.error(f"Error in spiral point {point}: {e}")
                continue
        
        return edges_found, scan_points_added
    
    def perform_coverage_gap_analysis(self, edge_points: List[EdgePoint], 
                                    scan_points: List[ScanPoint]) -> Tuple[List[EdgePoint], List[ScanPoint]]:
        """
        Phase 3: Analyze angular coverage and fill gaps with targeted radial scans.
        
        Args:
            edge_points: Current edge points
            scan_points: Current scan points (will be modified)
            
        Returns:
            Tuple of (updated_edge_points, updated_scan_points)
            
        Raises:
            SafetyError: If insufficient coverage for safety requirements
        """
        if len(edge_points) < 3:
            raise SafetyError("Insufficient edges for gap analysis")
        
        self.logger.info("=== Phase 3: Coverage Gap Analysis ===")
        
        # Calculate preliminary center for gap analysis
        prelim_center_x, prelim_center_y = self._estimate_preliminary_center(edge_points)
        self.logger.info(f"Using preliminary center ({prelim_center_x:.1f}, {prelim_center_y:.1f}) for gap analysis")
        
        # Calculate angles of all edges relative to preliminary center
        edge_angles = []
        for edge in edge_points:
            angle = np.arctan2(edge.y - prelim_center_y, edge.x - prelim_center_x)
            angle_deg = np.degrees(angle)
            if angle_deg < 0:
                angle_deg += 360  # Normalize to 0-360
            edge_angles.append(angle_deg)
        
        # Sort angles to find gaps
        edge_angles.sort()
        
        # Find gaps larger than threshold
        gaps_to_fill = []
        
        for i in range(len(edge_angles)):
            next_idx = (i + 1) % len(edge_angles)
            current_angle = edge_angles[i]
            next_angle = edge_angles[next_idx]
            
            # Calculate gap size (handle wrap-around at 360°)
            if next_angle > current_angle:
                gap_size = next_angle - current_angle
            else:
                gap_size = (360 - current_angle) + next_angle
            
            if gap_size > self.gap_threshold:
                gap_center = (current_angle + gap_size/2) % 360
                gaps_to_fill.append((gap_center, gap_size))
        
        self.logger.info(f"Found {len(gaps_to_fill)} coverage gaps > {self.gap_threshold}°")
        
        # Fill each gap with targeted radial scan
        updated_edge_points = list(edge_points)
        updated_scan_points = list(scan_points)
        gaps_filled = 0
        scan_radius = self.safety_params.expected_die_diameter/2 + 10.0  # Start outside expected edge
        
        for gap_center_deg, gap_size in gaps_to_fill:
            self.logger.info(f"Filling gap at {gap_center_deg:.1f}° (size: {gap_size:.1f}°)")
            
            try:
                # Calculate start position for radial scan
                gap_angle_rad = np.radians(gap_center_deg)
                start_x = prelim_center_x + scan_radius * np.cos(gap_angle_rad)
                start_y = prelim_center_y + scan_radius * np.sin(gap_angle_rad)
                
                # Perform targeted radial scan
                radial_edges, radial_points = self._perform_targeted_radial_scan(
                    start_x, start_y, prelim_center_x, prelim_center_y
                )
                
                updated_edge_points.extend(radial_edges)
                updated_scan_points.extend(radial_points)
                
                if radial_edges:
                    gaps_filled += 1
                    self.logger.info(f"Gap filled: +{len(radial_edges)} edges")
                else:
                    self.logger.warning(f"No edges found in gap at {gap_center_deg:.1f}°")
                
            except Exception as e:
                self.logger.error(f"Error filling gap at {gap_center_deg:.1f}°: {e}")
                continue
        
        # Validate final coverage meets safety requirements
        final_coverage = calculate_angular_coverage(updated_edge_points, prelim_center_x, prelim_center_y)
        
        if final_coverage < self.safety_params.min_angular_coverage:
            raise SafetyError(
                f"Insufficient angular coverage after gap filling: {final_coverage:.1f}° < {self.safety_params.min_angular_coverage}° required"
            )
        
        self.logger.info(f"Coverage gap analysis complete: {gaps_filled}/{len(gaps_to_fill)} gaps filled")
        self.logger.info(f"Final angular coverage: {final_coverage:.1f}°")
        
        return updated_edge_points, updated_scan_points
    
    def _perform_targeted_radial_scan(self, start_x: float, start_y: float,
                                    center_x: float, center_y: float) -> Tuple[List[EdgePoint], List[ScanPoint]]:
        """Perform targeted radial scan from outside toward center."""
        edges_found = []
        scan_points_added = []
        
        try:
            current_pos = self.robot.read_position()
            current_z = current_pos[2]
        except Exception as e:
            raise ScanningError(f"Failed to read position for radial scan: {e}")
        
        # Calculate direction vector toward center
        dx = center_x - start_x
        dy = center_y - start_y
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            return edges_found, scan_points_added
            
        # Normalize direction
        dx_norm = dx / distance
        dy_norm = dy / distance
        
        # Scan inward toward center
        steps = int(self.radial_scan_distance / self.radial_step_size)
        last_height = None
        
        for step in range(steps):
            scan_x = start_x + (step * self.radial_step_size * dx_norm)
            scan_y = start_y + (step * self.radial_step_size * dy_norm)
            
            try:
                # Move and measure
                self.robot.send_move_command(scan_x, scan_y, current_z)
                
                # Wait for move
                if not self.robot.wait_for_status(10, timeout=3.0):
                    continue
                
                # Measure
                height = self.sensor.read_height()
                if height > -999:
                    scan_point = ScanPoint(
                        x=scan_x, y=scan_y, z=current_z,
                        height=height, is_valid=True, timestamp=time.time()
                    )
                    scan_points_added.append(scan_point)
                    
                    # Detect edges
                    if last_height is not None and abs(height - last_height) >= 2.0:
                        edge_type = "drop_off" if height < last_height else "rise"
                        quality = min(abs(height - last_height) / 4.0, 1.0)
                        
                        if quality >= self.safety_params.min_edge_quality:
                            edge_point = EdgePoint(
                                x=scan_x, y=scan_y,
                                edge_type=edge_type,
                                confidence=quality
                            )
                            edges_found.append(edge_point)
                    
                    last_height = height
                    
            except Exception as e:
                self.logger.error(f"Error in targeted scan step {step}: {e}")
                continue
        
        return edges_found, scan_points_added
    
    def remove_outlier_edges(self, edge_points: List[EdgePoint]) -> List[EdgePoint]:
        """
        Phase 4: Quality validation and outlier removal using statistical analysis.
        
        Args:
            edge_points: Edge points to analyze
            
        Returns:
            Filtered edge points with outliers removed
            
        Raises:
            SafetyError: If too many edges removed or insufficient remaining
        """
        if len(edge_points) < 4:
            raise SafetyError("Insufficient edges for outlier analysis")
        
        self.logger.info("=== Phase 4: Quality Validation and Outlier Removal ===")
        
        # Get preliminary center for distance calculations
        prelim_center_x, prelim_center_y = self._estimate_preliminary_center(edge_points)
        
        # Calculate distances and qualities
        edge_distances = []
        edge_qualities = []
        
        for edge in edge_points:
            distance = calculate_distance(edge.x, edge.y, prelim_center_x, prelim_center_y)
            edge_distances.append(distance)
            edge_qualities.append(edge.confidence)
        
        edge_distances = np.array(edge_distances)
        edge_qualities = np.array(edge_qualities)
        
        # Statistical outlier detection based on distance from center
        distance_mean = np.mean(edge_distances)
        distance_std = np.std(edge_distances)
        
        # Identify outliers
        outlier_indices = []
        
        for i, (dist, quality) in enumerate(zip(edge_distances, edge_qualities)):
            is_distance_outlier = abs(dist - distance_mean) > (self.outlier_std_threshold * distance_std)
            is_quality_outlier = quality < self.quality_threshold
            
            if is_distance_outlier or is_quality_outlier:
                outlier_indices.append(i)
                self.logger.debug(f"Outlier edge {i}: dist={dist:.1f}mm (mean±{self.outlier_std_threshold}σ: {distance_mean:.1f}±{self.outlier_std_threshold*distance_std:.1f}), quality={quality:.2f}")
        
        # Safety check: Don't remove too many edges
        outlier_ratio = len(outlier_indices) / len(edge_points)
        if outlier_ratio > 0.5:  # More than 50% outliers
            raise SafetyError(f"Too many outlier edges detected: {len(outlier_indices)}/{len(edge_points)} ({outlier_ratio:.1%})")
        
        # Remove outliers (in reverse order to preserve indices)
        filtered_edges = []
        outliers_removed = 0
        
        for i, edge in enumerate(edge_points):
            if i not in outlier_indices:
                filtered_edges.append(edge)
            else:
                outliers_removed += 1
                self.logger.debug(f"Removed outlier edge: ({edge.x:.1f}, {edge.y:.1f})")
        
        # Safety validation of remaining edges
        if len(filtered_edges) < self.safety_params.min_perimeter_edges:
            raise SafetyError(
                f"Too few edges after outlier removal: {len(filtered_edges)} < {self.safety_params.min_perimeter_edges} required"
            )
        
        self.logger.info(f"Outlier removal complete: {len(edge_points)} → {len(filtered_edges)} edges (-{outliers_removed})")
        
        return filtered_edges
    
    def _estimate_preliminary_center(self, edge_points: List[EdgePoint]) -> Tuple[float, float]:
        """Estimate preliminary center from edge points."""
        if len(edge_points) < 3:
            raise ValidationError("Need at least 3 edge points for preliminary center estimation")
        
        # Simple centroid calculation
        x_coords = [ep.x for ep in edge_points]
        y_coords = [ep.y for ep in edge_points]
        
        center_x = np.mean(x_coords)
        center_y = np.mean(y_coords)
        
        return center_x, center_y
    
    def analyze_scanning_coverage(self, scan_points: List[ScanPoint], 
                                edge_points: List[EdgePoint]) -> Dict[str, Any]:
        """
        Analyze comprehensive scanning coverage metrics.
        
        Args:
            scan_points: All scan points
            edge_points: All edge points
            
        Returns:
            Dictionary of coverage metrics
        """
        coverage_data = {
            'x_range': 0.0,
            'y_range': 0.0,
            'total_area': 0.0,
            'point_density': 0.0,
            'edge_coverage': 0.0,
            'efficiency': 0.0
        }
        
        valid_points = [sp for sp in scan_points if sp.is_valid and sp.height > -999]
        if not valid_points:
            return coverage_data
        
        # Basic coverage metrics
        x_coords = [sp.x for sp in valid_points]
        y_coords = [sp.y for sp in valid_points]
        
        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        total_area = x_range * y_range
        
        # Point density (points per square mm)
        point_density = len(valid_points) / max(1, total_area)
        
        # Edge coverage analysis
        if edge_points and len(edge_points) > 2:
            center_x, center_y = self._estimate_preliminary_center(edge_points)
            edge_coverage = calculate_angular_coverage(edge_points, center_x, center_y)
        else:
            edge_coverage = 0.0
        
        # Scanning efficiency
        efficiency = (len(valid_points) / max(1, len(scan_points))) * 100
        
        coverage_data.update({
            'x_range': x_range,
            'y_range': y_range,
            'total_area': total_area,
            'point_density': point_density,
            'edge_coverage': edge_coverage,
            'efficiency': efficiency
        })
        
        return coverage_data
    
    def perform_enhanced_edge_classification(self, edge_points: List[EdgePoint]) -> Tuple[List[EdgePoint], List[EdgePoint]]:
        """
        Phase 5: Enhanced edge classification using geometric validation and iterative refinement.
        
        Args:
            edge_points: All detected edge points
            
        Returns:
            Tuple of (outer_edges, interior_edges)
            
        Raises:
            SafetyError: If classification fails or insufficient edges remain
        """
        if len(edge_points) < 4:
            raise SafetyError("Insufficient edges for enhanced classification")
        
        self.logger.info("=== Phase 5: Enhanced Edge Classification ===")
        
        # Phase 1: Analyze current edge distribution
        edge_analysis = self._analyze_edge_distribution(edge_points)
        
        # Phase 2: Geometric validation using RANSAC
        validated_edges = self._geometric_edge_validation(edge_analysis)
        
        # Phase 3: Iterative center refinement
        refined_center, refined_edges = self._iterative_center_refinement(validated_edges)
        
        # Phase 4: Quality-based final classification
        outer_edges, interior_edges = self._quality_based_classification(refined_center, refined_edges, edge_points)
        
        # Safety validation
        if len(outer_edges) < self.safety_params.min_perimeter_edges:
            raise SafetyError(
                f"Enhanced classification resulted in too few outer edges: {len(outer_edges)} < {self.safety_params.min_perimeter_edges} required"
            )
        
        self.logger.info(f"Enhanced classification complete: {len(outer_edges)} outer edges, {len(interior_edges)} interior edges")
        
        return outer_edges, interior_edges
    
    def _analyze_edge_distribution(self, edge_points: List[EdgePoint]) -> Dict[str, Any]:
        """Analyze current edge distribution to identify classification issues."""
        if not edge_points:
            return {}
        
        # Calculate distances from preliminary center
        prelim_center_x, prelim_center_y = self._estimate_preliminary_center(edge_points)
        
        edge_data = []
        for edge in edge_points:
            distance = calculate_distance(edge.x, edge.y, prelim_center_x, prelim_center_y)
            angle = np.arctan2(edge.y - prelim_center_y, edge.x - prelim_center_x)
            
            edge_data.append({
                'edge': edge,
                'distance': distance,
                'angle': angle,
                'angle_deg': np.degrees(angle) % 360
            })
        
        # Group edges by distance ranges
        distance_groups = {
            'very_close': [e for e in edge_data if e['distance'] < 35],
            'close': [e for e in edge_data if 35 <= e['distance'] < 45],
            'target_range': [e for e in edge_data if 45 <= e['distance'] <= 70],
            'far': [e for e in edge_data if 70 < e['distance'] < 80],
            'very_far': [e for e in edge_data if e['distance'] >= 80]
        }
        
        # Analyze angular distribution
        angles = [e['angle_deg'] for e in edge_data]
        angular_gaps = self._find_angular_gaps(angles)
        
        analysis = {
            'prelim_center': (prelim_center_x, prelim_center_y),
            'edge_data': edge_data,
            'distance_groups': distance_groups,
            'angular_gaps': angular_gaps,
            'total_edges': len(edge_data)
        }
        
        # Log analysis results
        self.logger.info(f"Edge distribution analysis:")
        for group_name, edges in distance_groups.items():
            if edges:
                self.logger.info(f"  {group_name}: {len(edges)} edges")
        
        return analysis
    
    def _geometric_edge_validation(self, edge_analysis: Dict[str, Any]) -> List[EdgePoint]:
        """Validate edges using RANSAC circle fitting."""
        edge_data = edge_analysis['edge_data']
        
        if len(edge_data) < 4:
            return [e['edge'] for e in edge_data]
        
        # Extract coordinates for RANSAC
        points = np.array([[e['edge'].x, e['edge'].y] for e in edge_data])
        x, y = points[:, 0], points[:, 1]
        
        # Use simple RANSAC implementation
        center_x, center_y, radius, inliers = self._ransac_circle_fit(x, y)
        
        self.logger.info(f"RANSAC circle fit: center=({center_x:.1f}, {center_y:.1f}), radius={radius:.1f}mm")
        self.logger.info(f"RANSAC inliers: {len(inliers)}/{len(edge_data)} edges")
        
        # Validate circle parameters
        if radius < 30 or radius > 80:
            self.logger.warning(f"RANSAC radius {radius:.1f}mm outside expected range, using all edges")
            return [e['edge'] for e in edge_data]
        
        # Return edges that are inliers to the best circle
        validated_edges = [edge_data[i]['edge'] for i in inliers]
        
        # Add nearby edges that might have been missed
        for i, edge_info in enumerate(edge_data):
            if i not in inliers:
                distance_to_circle = abs(calculate_distance(edge_info['edge'].x, edge_info['edge'].y, center_x, center_y) - radius)
                if distance_to_circle < 5.0:  # Within 5mm of circle
                    validated_edges.append(edge_info['edge'])
                    self.logger.debug(f"Added nearby edge: ({edge_info['edge'].x:.1f}, {edge_info['edge'].y:.1f}) - {distance_to_circle:.1f}mm from circle")
        
        return validated_edges
    
    def _iterative_center_refinement(self, validated_edges: List[EdgePoint], max_iterations: int = 3) -> Tuple[Tuple[float, float], List[EdgePoint]]:
        """Iteratively refine center estimate and reclassify edges."""
        if len(validated_edges) < 4:
            center = self._estimate_preliminary_center(validated_edges)
            return center, validated_edges
        
        current_edges = validated_edges.copy()
        
        for iteration in range(max_iterations):
            # Calculate current center using simple least squares
            x_coords = [e.x for e in current_edges]
            y_coords = [e.y for e in current_edges]
            center_x = np.mean(x_coords)
            center_y = np.mean(y_coords)
            
            # Calculate distances and identify outliers
            distances = []
            for edge in current_edges:
                dist = calculate_distance(edge.x, edge.y, center_x, center_y)
                distances.append(dist)
            
            distances = np.array(distances)
            median_distance = np.median(distances)
            mad = np.median(np.abs(distances - median_distance))
            
            # Remove outliers (edges too far from median distance)
            outlier_threshold = median_distance + 3 * mad
            inlier_threshold = median_distance - 3 * mad
            
            refined_edges = []
            for i, edge in enumerate(current_edges):
                if inlier_threshold <= distances[i] <= outlier_threshold:
                    refined_edges.append(edge)
                else:
                    self.logger.debug(f"Iteration {iteration+1}: Removed outlier edge ({edge.x:.1f}, {edge.y:.1f}) - dist={distances[i]:.1f}mm")
            
            if len(refined_edges) < 4:
                break
            
            # Check for convergence
            if len(refined_edges) == len(current_edges):
                self.logger.info(f"Center refinement converged after {iteration+1} iterations")
                break
            
            current_edges = refined_edges
            self.logger.info(f"Iteration {iteration+1}: {len(current_edges)} edges, center=({center_x:.1f}, {center_y:.1f})")
        
        return (center_x, center_y), current_edges
    
    def _quality_based_classification(self, refined_center: Tuple[float, float], refined_edges: List[EdgePoint], 
                                    all_edge_points: List[EdgePoint]) -> Tuple[List[EdgePoint], List[EdgePoint]]:
        """Final classification based on enhanced edge quality scores."""
        center_x, center_y = refined_center
        
        outer_edges = []
        interior_edges = []
        
        # Calculate quality scores for all original edges
        edge_scores = []
        for edge in all_edge_points:
            quality_score = self._calculate_enhanced_edge_quality(edge, center_x, center_y, refined_edges)
            edge_scores.append((edge, quality_score))
        
        # Sort by quality score
        edge_scores.sort(key=lambda x: x[1], reverse=True)
        
        # Classify edges based on quality and geometric criteria
        for edge, quality in edge_scores:
            distance = calculate_distance(edge.x, edge.y, center_x, center_y)
            
            # Enhanced classification criteria
            is_outer = self._is_outer_edge_enhanced(edge, center_x, center_y, quality, distance, refined_edges, outer_edges)
            
            if is_outer:
                outer_edges.append(edge)
                self.logger.debug(f"OUTER edge: ({edge.x:.1f}, {edge.y:.1f}) - dist={distance:.1f}mm, quality={quality:.3f}")
            else:
                interior_edges.append(edge)
                self.logger.debug(f"INTERIOR edge: ({edge.x:.1f}, {edge.y:.1f}) - dist={distance:.1f}mm, quality={quality:.3f}")
        
        return outer_edges, interior_edges
    
    def _calculate_enhanced_edge_quality(self, edge: EdgePoint, center_x: float, center_y: float, 
                                       reference_edges: List[EdgePoint]) -> float:
        """Calculate enhanced quality score for edge classification."""
        distance = calculate_distance(edge.x, edge.y, center_x, center_y)
        
        quality = 0.0
        
        # Factor 1: Base confidence from original detection
        quality += edge.confidence * 0.3
        
        # Factor 2: Distance from expected radius
        if reference_edges:
            ref_distances = [calculate_distance(e.x, e.y, center_x, center_y) for e in reference_edges]
            expected_radius = np.median(ref_distances)
            distance_error = abs(distance - expected_radius)
            distance_factor = max(0, 1.0 - distance_error / (expected_radius * 0.3))
            quality += distance_factor * 0.4
        else:
            # Fallback to original distance criteria
            if 45 <= distance <= 70:
                quality += 0.4
        
        # Factor 3: Consistency with nearby edges
        consistency_score = self._calculate_local_consistency(edge, center_x, center_y, reference_edges)
        quality += consistency_score * 0.2
        
        # Factor 4: Angular coverage contribution
        angular_score = self._calculate_angular_contribution(edge, center_x, center_y, reference_edges)
        quality += angular_score * 0.1
        
        return min(quality, 1.0)
    
    def _is_outer_edge_enhanced(self, edge: EdgePoint, center_x: float, center_y: float, 
                              quality: float, distance: float, reference_edges: List[EdgePoint],
                              current_outer_edges: List[EdgePoint]) -> bool:
        """Enhanced criteria for determining if edge is outer perimeter edge."""
        
        # Minimum quality threshold
        if quality < self.safety_params.min_edge_quality:
            return False
        
        # Distance criteria - more flexible based on reference edges
        if reference_edges:
            ref_distances = [calculate_distance(e.x, e.y, center_x, center_y) for e in reference_edges]
            if ref_distances:
                median_radius = np.median(ref_distances)
                std_radius = np.std(ref_distances)
                
                # Allow edges within 2 standard deviations of median
                min_dist = max(35, median_radius - 2 * std_radius)
                max_dist = min(80, median_radius + 2 * std_radius)
                
                if not (min_dist <= distance <= max_dist):
                    return False
        else:
            # Fallback to safety parameter criteria
            if not (self.safety_params.min_outer_radius <= distance <= self.safety_params.max_outer_radius):
                return False
        
        # Angular spacing - avoid clustering
        angle = np.arctan2(edge.y - center_y, edge.x - center_x)
        min_angular_separation = np.radians(20)  # 20 degrees
        
        for existing_edge in current_outer_edges:
            existing_angle = np.arctan2(existing_edge.y - center_y, existing_edge.x - center_x)
            angular_diff = abs(angle - existing_angle)
            angular_diff = min(angular_diff, 2 * np.pi - angular_diff)  # Handle wrap-around
            
            if angular_diff < min_angular_separation:
                # Keep the higher quality edge
                existing_quality = self._calculate_enhanced_edge_quality(existing_edge, center_x, center_y, reference_edges)
                if quality <= existing_quality:
                    return False
                else:
                    # Remove the lower quality edge would require modifying the list during iteration
                    # For safety, we'll reject this edge instead
                    return False
        
        return True
    
    def _calculate_local_consistency(self, edge: EdgePoint, center_x: float, center_y: float, 
                                   reference_edges: List[EdgePoint]) -> float:
        """Calculate how consistent edge is with nearby reference edges."""
        if not reference_edges:
            return 0.5
        
        edge_angle = np.arctan2(edge.y - center_y, edge.x - center_x)
        edge_distance = calculate_distance(edge.x, edge.y, center_x, center_y)
        
        nearby_edges = []
        for ref_edge in reference_edges:
            ref_angle = np.arctan2(ref_edge.y - center_y, ref_edge.x - center_x)
            angular_diff = abs(edge_angle - ref_angle)
            angular_diff = min(angular_diff, 2 * np.pi - angular_diff)
            
            if angular_diff < np.radians(45):  # Within 45 degrees
                nearby_edges.append(ref_edge)
        
        if not nearby_edges:
            return 0.3
        
        # Calculate distance consistency
        nearby_distances = [calculate_distance(e.x, e.y, center_x, center_y) for e in nearby_edges]
        avg_distance = np.mean(nearby_distances)
        distance_consistency = max(0, 1.0 - abs(edge_distance - avg_distance) / (avg_distance * 0.2))
        
        return distance_consistency
    
    def _calculate_angular_contribution(self, edge: EdgePoint, center_x: float, center_y: float, 
                                      reference_edges: List[EdgePoint]) -> float:
        """Calculate how much this edge contributes to angular coverage."""
        if not reference_edges:
            return 0.5
        
        edge_angle = np.arctan2(edge.y - center_y, edge.x - center_x)
        ref_angles = [np.arctan2(e.y - center_y, e.x - center_x) for e in reference_edges]
        
        # Find closest reference angles
        angle_diffs = [abs(edge_angle - ref_angle) for ref_angle in ref_angles]
        angle_diffs = [min(diff, 2 * np.pi - diff) for diff in angle_diffs]
        
        min_separation = min(angle_diffs) if angle_diffs else np.pi
        
        # Reward edges that fill gaps
        if min_separation > np.radians(30):  # 30 degrees
            return 1.0
        elif min_separation > np.radians(15):  # 15 degrees
            return 0.7
        else:
            return 0.3
    
    def _find_angular_gaps(self, angles: List[float]) -> List[float]:
        """Find gaps in angular coverage."""
        if len(angles) < 2:
            return []
        
        sorted_angles = sorted(angles)
        gaps = []
        
        for i in range(len(sorted_angles)):
            next_i = (i + 1) % len(sorted_angles)
            gap = sorted_angles[next_i] - sorted_angles[i]
            if gap < 0:
                gap += 360
            gaps.append(gap)
        
        return gaps
    
    def _ransac_circle_fit(self, x: np.ndarray, y: np.ndarray, max_iterations: int = 50) -> Tuple[float, float, float, List[int]]:
        """Simple RANSAC circle fitting implementation."""
        best_inliers = []
        best_center_x = 0.0
        best_center_y = 0.0
        best_radius = 0.0
        
        if len(x) < 3:
            return best_center_x, best_center_y, best_radius, best_inliers
        
        threshold = 3.0  # mm tolerance
        
        for _ in range(max_iterations):
            # Randomly select 3 points
            if len(x) < 3:
                break
                
            indices = np.random.choice(len(x), 3, replace=False)
            x_sample = x[indices]
            y_sample = y[indices]
            
            # Fit circle to 3 points
            try:
                center_x, center_y, radius = self._fit_circle_to_three_points(x_sample, y_sample)
                
                if radius < 20 or radius > 100:  # Unrealistic radius
                    continue
                
                # Count inliers
                inliers = []
                for i in range(len(x)):
                    dist_to_center = np.sqrt((x[i] - center_x)**2 + (y[i] - center_y)**2)
                    error = abs(dist_to_center - radius)
                    if error < threshold:
                        inliers.append(i)
                
                if len(inliers) > len(best_inliers):
                    best_inliers = inliers
                    best_center_x = center_x
                    best_center_y = center_y
                    best_radius = radius
                    
            except Exception:
                continue
        
        return best_center_x, best_center_y, best_radius, best_inliers
    
    def _fit_circle_to_three_points(self, x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
        """Fit circle to exactly 3 points."""
        if len(x) != 3:
            raise ValueError("Need exactly 3 points")
        
        # Calculate circle center using perpendicular bisectors
        ax, ay = x[0], y[0]
        bx, by = x[1], y[1]
        cx, cy = x[2], y[2]
        
        # Midpoints
        m1x, m1y = (ax + bx) / 2, (ay + by) / 2
        m2x, m2y = (bx + cx) / 2, (by + cy) / 2
        
        # Slopes
        if abs(bx - ax) < 1e-6:  # Vertical line AB
            if abs(cx - bx) < 1e-6:  # Vertical line BC
                raise ValueError("Points are collinear")
            slope2 = (cy - by) / (cx - bx)
            perp_slope2 = -1 / slope2
            center_x = m1x
            center_y = m2y + perp_slope2 * (center_x - m2x)
        elif abs(cx - bx) < 1e-6:  # Vertical line BC
            slope1 = (by - ay) / (bx - ax)
            perp_slope1 = -1 / slope1
            center_x = m2x
            center_y = m1y + perp_slope1 * (center_x - m1x)
        else:
            slope1 = (by - ay) / (bx - ax)
            slope2 = (cy - by) / (cx - bx)
            
            if abs(slope1 - slope2) < 1e-6:
                raise ValueError("Points are collinear")
            
            perp_slope1 = -1 / slope1
            perp_slope2 = -1 / slope2
            
            # Find intersection of perpendicular bisectors
            center_x = (perp_slope2 * m2x - perp_slope1 * m1x + m1y - m2y) / (perp_slope2 - perp_slope1)
            center_y = m1y + perp_slope1 * (center_x - m1x)
        
        # Calculate radius
        radius = np.sqrt((center_x - ax)**2 + (center_y - ay)**2)
        
        return center_x, center_y, radius