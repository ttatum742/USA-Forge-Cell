"""Center calculation algorithms for die scanner production package."""

import numpy as np
from typing import List, Tuple
import logging

from ..models.data_models import EdgePoint, DieCenter, SafetyParameters
from ..exceptions.custom_exceptions import CenterCalculationError, SafetyError
from ..utils.geometric_calculations import (
    kasa_circle_fit, taubin_circle_fit, ransac_circle_fit,
    weighted_least_squares_fit, calculate_center_consensus
)
from ..utils.validation import validate_die_center


class CenterCalculator:
    """Production center calculation with multi-method consensus and safety validation."""
    
    def __init__(self, safety_params: SafetyParameters):
        """
        Initialize center calculator.
        
        Args:
            safety_params: Safety validation parameters
        """
        self.safety_params = safety_params
        self.logger = logging.getLogger(__name__)
    
    def calculate_center(self, edge_points: List[EdgePoint], scan_points: List) -> DieCenter:
        """
        Calculate die center using multi-method consensus with safety validation.
        
        Args:
            edge_points: List of validated edge points
            scan_points: List of scan points for height calculation
            
        Returns:
            DieCenter with validated results
            
        Raises:
            CenterCalculationError: If calculation fails safety validation
            SafetyError: If results are unsafe for operation
        """
        if len(edge_points) < self.safety_params.min_perimeter_edges:
            raise CenterCalculationError(
                f"Insufficient edge points: {len(edge_points)} < {self.safety_params.min_perimeter_edges}"
            )
        
        try:
            # Prepare coordinate arrays
            coordinates = np.array([[ep.x, ep.y] for ep in edge_points])
            x = coordinates[:, 0]
            y = coordinates[:, 1]
            weights = np.array([ep.confidence for ep in edge_points])
            
            self.logger.info(f"Calculating center from {len(edge_points)} edge points")
            
            # Method 1: RANSAC circle fitting (most robust)
            try:
                ransac_x, ransac_y, ransac_radius, inliers = ransac_circle_fit(x, y)
                ransac_weight = 0.4
                self.logger.debug(f"RANSAC: ({ransac_x:.1f}, {ransac_y:.1f}) - {np.sum(inliers)}/{len(x)} inliers")
            except Exception as e:
                self.logger.warning(f"RANSAC fitting failed: {e}")
                raise CenterCalculationError("RANSAC circle fitting failed")
            
            # Method 2: Weighted least squares
            try:
                weighted_x, weighted_y = weighted_least_squares_fit(x, y, weights)
                weighted_weight = 0.3
                self.logger.debug(f"Weighted LS: ({weighted_x:.1f}, {weighted_y:.1f})")
            except Exception as e:
                self.logger.warning(f"Weighted LS fitting failed: {e}")
                raise CenterCalculationError("Weighted least squares fitting failed")
            
            # Method 3: Taubin circle fitting (geometric accuracy)
            try:
                taubin_x, taubin_y, taubin_radius = taubin_circle_fit(x, y)
                taubin_weight = 0.2
                self.logger.debug(f"Taubin: ({taubin_x:.1f}, {taubin_y:.1f})")
            except Exception as e:
                self.logger.warning(f"Taubin fitting failed: {e}")
                raise CenterCalculationError("Taubin circle fitting failed")
            
            # Method 4: Kasa least squares (backup)
            try:
                kasa_x, kasa_y, kasa_radius = kasa_circle_fit(x, y)
                kasa_weight = 0.1
                self.logger.debug(f"Kasa LS: ({kasa_x:.1f}, {kasa_y:.1f})")
            except Exception as e:
                self.logger.warning(f"Kasa fitting failed: {e}")
                raise CenterCalculationError("Kasa circle fitting failed")
            
            # Calculate consensus center
            center_estimates = [
                (ransac_x, ransac_y, ransac_weight),
                (weighted_x, weighted_y, weighted_weight),
                (taubin_x, taubin_y, taubin_weight),
                (kasa_x, kasa_y, kasa_weight)
            ]
            
            final_x, final_y, confidence = calculate_center_consensus(center_estimates)
            
            # Calculate diameter from RANSAC result (most robust)
            diameter = ransac_radius * 2
            
            # Calculate average height from valid scan points
            valid_heights = [sp.height for sp in scan_points if sp.is_valid and sp.height > -999]
            avg_height = np.mean(valid_heights) if valid_heights else 0.0
            
            # Create die center result
            die_center = DieCenter(
                center_x=final_x,
                center_y=final_y,
                diameter=diameter,
                avg_height=avg_height,
                confidence=confidence,
                edge_points=edge_points,
                calculation_method="multi_method_consensus"
            )
            
            self.logger.info(f"Center calculated: ({final_x:.1f}, {final_y:.1f}) confidence={confidence:.1f}%")
            
            # CRITICAL: Validate result for safety
            self._validate_center_safety(die_center)
            
            return die_center
            
        except CenterCalculationError:
            raise  # Re-raise center calculation errors
        except Exception as e:
            raise CenterCalculationError(f"Center calculation failed: {e}")
    
    def _validate_center_safety(self, die_center: DieCenter) -> None:
        """
        Validate center calculation for safety - NO FALLBACKS.
        
        Args:
            die_center: Calculated die center
            
        Raises:
            SafetyError: If center calculation is unsafe
        """
        try:
            # Use comprehensive validation from utils
            validate_die_center(die_center, self.safety_params)
            
            # Additional production-specific checks
            self._validate_method_agreement(die_center)
            self._validate_edge_consistency(die_center)
            
            self.logger.info("Center calculation passed all safety validations")
            
        except Exception as e:
            # ANY validation failure is a safety error
            raise SafetyError(f"Center calculation failed safety validation: {e}")
    
    def _validate_method_agreement(self, die_center: DieCenter) -> None:
        """
        Validate that multiple calculation methods agree.
        
        Raises:
            SafetyError: If methods disagree significantly
        """
        if die_center.confidence < self.safety_params.min_confidence_score:
            raise SafetyError(
                f"Method disagreement too high: confidence {die_center.confidence:.1f}% "
                f"< {self.safety_params.min_confidence_score}% required"
            )
    
    def _validate_edge_consistency(self, die_center: DieCenter) -> None:
        """
        Validate edge point consistency with calculated center.
        
        Raises:
            SafetyError: If edges are inconsistent with center
        """
        # Calculate distances from center to each edge
        distances = []
        for edge in die_center.edge_points:
            distance = np.sqrt(
                (edge.x - die_center.center_x)**2 + 
                (edge.y - die_center.center_y)**2
            )
            distances.append(distance)
        
        distances = np.array(distances)
        
        # Check for reasonable consistency
        mean_distance = np.mean(distances)
        std_distance = np.std(distances)
        
        # Require standard deviation to be less than 10% of mean radius (tighter for production)
        if std_distance > mean_distance * 0.1:
            raise SafetyError(
                f"Edge points too inconsistent with center: std={std_distance:.1f}mm, "
                f"mean={mean_distance:.1f}mm (>10% variation)"
            )
        
        # Check for outliers (distances > 2 standard deviations from mean)
        outliers = np.abs(distances - mean_distance) > 2 * std_distance
        outlier_count = np.sum(outliers)
        
        if outlier_count > len(die_center.edge_points) * 0.1:  # More than 10% outliers
            raise SafetyError(
                f"Too many edge outliers: {outlier_count}/{len(die_center.edge_points)} (>10%)"
            )
    
    def estimate_preliminary_center(self, edge_points: List[EdgePoint]) -> Tuple[float, float]:
        """
        Quick preliminary center estimation for filtering.
        
        Args:
            edge_points: List of edge points
            
        Returns:
            Tuple of (center_x, center_y)
            
        Raises:
            CenterCalculationError: If estimation fails
        """
        if len(edge_points) < 3:
            raise CenterCalculationError("Need at least 3 edge points for preliminary estimation")
        
        try:
            # Simple centroid calculation
            x_coords = [ep.x for ep in edge_points]
            y_coords = [ep.y for ep in edge_points]
            
            center_x = np.mean(x_coords)
            center_y = np.mean(y_coords)
            
            return center_x, center_y
            
        except Exception as e:
            raise CenterCalculationError(f"Preliminary center estimation failed: {e}")