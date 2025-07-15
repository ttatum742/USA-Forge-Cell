"""Validation utilities for die scanner package - NO FALLBACKS."""

import numpy as np
from typing import List
from ..models.data_models import ScanPoint, EdgePoint, DieCenter, SafetyParameters
from ..exceptions.custom_exceptions import (
    ValidationError, SafetyError, InsufficientDataError,
    AngularCoverageError, DiameterValidationError, ConfidenceError
)
from .geometric_calculations import calculate_angular_coverage, calculate_distance


def validate_scan_data(scan_points: List[ScanPoint]) -> None:
    """
    Validate scan data meets minimum requirements.
    
    Raises ValidationError if data is insufficient.
    """
    if not scan_points:
        raise InsufficientDataError("No scan points provided")
    
    valid_points = [sp for sp in scan_points if sp.is_valid and sp.height > -999]
    
    if len(valid_points) < 50:
        raise InsufficientDataError(
            f"Insufficient valid scan points: {len(valid_points)} < 50 required"
        )
    
    # Check for reasonable data distribution
    if len(valid_points) < len(scan_points) * 0.7:
        invalid_ratio = (len(scan_points) - len(valid_points)) / len(scan_points)
        raise ValidationError(
            f"Too many invalid readings: {invalid_ratio:.1%} invalid data"
        )


def validate_edge_points(edge_points: List[EdgePoint], safety_params: SafetyParameters) -> None:
    """
    Validate edge points meet safety requirements.
    
    Raises SafetyError if edges are insufficient.
    """
    if not edge_points:
        raise InsufficientDataError("No edge points detected")
    
    if len(edge_points) < safety_params.min_perimeter_edges:
        raise InsufficientDataError(
            f"Insufficient edge points: {len(edge_points)} < {safety_params.min_perimeter_edges} required"
        )
    
    # Validate edge quality
    low_quality_edges = [ep for ep in edge_points if ep.confidence < safety_params.min_edge_quality]
    if len(low_quality_edges) > len(edge_points) * 0.3:
        raise ValidationError(
            f"Too many low-quality edges: {len(low_quality_edges)}/{len(edge_points)}"
        )


def validate_angular_coverage(edge_points: List[EdgePoint], center_x: float, center_y: float,
                             safety_params: SafetyParameters) -> None:
    """
    Validate angular coverage meets safety requirements.
    
    Raises AngularCoverageError if coverage is insufficient.
    """
    coverage = calculate_angular_coverage(edge_points, center_x, center_y)
    
    if coverage < safety_params.min_angular_coverage:
        raise AngularCoverageError(
            f"Insufficient angular coverage: {coverage:.1f}° < {safety_params.min_angular_coverage}° required"
        )


def validate_die_center(die_center: DieCenter, safety_params: SafetyParameters) -> None:
    """
    Comprehensive validation of die center calculation.
    
    Raises SafetyError if center calculation is unsafe.
    """
    # Validate confidence score
    if die_center.confidence < safety_params.min_confidence_score:
        raise ConfidenceError(
            f"Confidence too low: {die_center.confidence:.1f}% < {safety_params.min_confidence_score}% required"
        )
    
    # Validate diameter
    diameter_error = abs(die_center.diameter - safety_params.expected_die_diameter)
    if diameter_error > safety_params.die_diameter_tolerance:
        raise DiameterValidationError(
            f"Diameter outside tolerance: {die_center.diameter:.1f}mm "
            f"(expected: {safety_params.expected_die_diameter:.1f}mm ± {safety_params.die_diameter_tolerance:.1f}mm)"
        )
    
    # Validate center position (if known reference)
    known_center_x, known_center_y = 398.0, 809.0  # Known approximate center
    center_deviation = calculate_distance(
        die_center.center_x, die_center.center_y,
        known_center_x, known_center_y
    )
    
    if center_deviation > safety_params.max_center_deviation:
        raise SafetyError(
            f"Center deviation too large: {center_deviation:.1f}mm > {safety_params.max_center_deviation:.1f}mm allowed"
        )
    
    # Validate edge points used in calculation
    validate_edge_points(die_center.edge_points, safety_params)
    
    # Validate angular coverage
    validate_angular_coverage(
        die_center.edge_points, 
        die_center.center_x, 
        die_center.center_y,
        safety_params
    )


def validate_edge_consistency(edge_points: List[EdgePoint], center_x: float, center_y: float) -> None:
    """
    Validate that edge points form a consistent circular pattern.
    
    Raises ValidationError if edges are inconsistent.
    """
    if len(edge_points) < 4:
        raise ValidationError("Need at least 4 edge points for consistency check")
    
    # Calculate distances from center to each edge
    distances = []
    for edge in edge_points:
        distance = calculate_distance(edge.x, edge.y, center_x, center_y)
        distances.append(distance)
    
    distances = np.array(distances)
    
    # Check for reasonable consistency
    mean_distance = np.mean(distances)
    std_distance = np.std(distances)
    
    # Require standard deviation to be less than 15% of mean radius
    if std_distance > mean_distance * 0.15:
        raise ValidationError(
            f"Edge points too inconsistent: std={std_distance:.1f}mm, mean={mean_distance:.1f}mm"
        )
    
    # Check for outliers (distances > 2 standard deviations from mean)
    outliers = np.abs(distances - mean_distance) > 2 * std_distance
    outlier_count = np.sum(outliers)
    
    if outlier_count > len(edge_points) * 0.2:  # More than 20% outliers
        raise ValidationError(
            f"Too many edge outliers: {outlier_count}/{len(edge_points)}"
        )


def validate_calibration_result(measured_height: float, reference_height: float, tolerance: float) -> None:
    """
    Validate sensor calibration result.
    
    Raises CalibrationError if calibration is outside tolerance.
    """
    from ..exceptions.custom_exceptions import CalibrationError
    
    error = abs(measured_height - reference_height)
    
    if error > tolerance:
        raise CalibrationError(
            f"Calibration error too large: {error:.2f}mm > {tolerance:.2f}mm tolerance"
        )


def validate_scan_coverage(scan_points: List[ScanPoint], min_area: float = 10000.0) -> None:
    """
    Validate that scan covers sufficient area.
    
    Raises ValidationError if coverage is insufficient.
    """
    if not scan_points:
        raise ValidationError("No scan points for coverage validation")
    
    valid_points = [sp for sp in scan_points if sp.is_valid]
    
    if len(valid_points) < 10:
        raise ValidationError("Insufficient points for coverage analysis")
    
    x_coords = [sp.x for sp in valid_points]
    y_coords = [sp.y for sp in valid_points]
    
    x_range = max(x_coords) - min(x_coords)
    y_range = max(y_coords) - min(y_coords)
    scan_area = x_range * y_range
    
    if scan_area < min_area:
        raise ValidationError(
            f"Scan area too small: {scan_area:.0f}mm² < {min_area:.0f}mm² required"
        )


def enforce_no_fallbacks() -> None:
    """
    Explicit function to document that NO fallback values are used.
    
    This function serves as documentation that the production package
    will NEVER use hardcoded fallback values for safety reasons.
    """
    pass  # Intentionally empty - serves as documentation