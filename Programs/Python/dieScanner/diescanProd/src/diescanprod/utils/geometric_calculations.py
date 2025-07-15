"""Geometric calculation utilities for die scanner package."""

import numpy as np
from typing import Tuple, List
from ..models.data_models import EdgePoint
from ..exceptions.custom_exceptions import ValidationError


def calculate_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Calculate Euclidean distance between two points."""
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def kasa_circle_fit(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """
    Kasa circle fitting algorithm (least squares).
    
    Returns:
        Tuple of (center_x, center_y, radius)
    """
    if len(x) < 3:
        raise ValidationError("Need at least 3 points for circle fitting")
    
    A = np.vstack([x, y, np.ones(len(x))]).T
    b = x**2 + y**2
    
    try:
        c = np.linalg.lstsq(A, b, rcond=None)[0]
        center_x = c[0] / 2
        center_y = c[1] / 2
        radius = np.sqrt(c[2] + center_x**2 + center_y**2)
        return center_x, center_y, radius
    except np.linalg.LinAlgError:
        raise ValidationError("Circle fitting failed - points may be collinear")


def taubin_circle_fit(x: np.ndarray, y: np.ndarray) -> Tuple[float, float, float]:
    """
    Taubin circle fitting algorithm (geometric fitting).
    More accurate than Kasa method.
    
    Returns:
        Tuple of (center_x, center_y, radius)
    """
    if len(x) < 3:
        raise ValidationError("Need at least 3 points for circle fitting")
    
    n = len(x)
    
    # Center data
    x_mean = np.mean(x)
    y_mean = np.mean(y)
    x_centered = x - x_mean
    y_centered = y - y_mean
    
    # Calculate moments
    Mxx = np.sum(x_centered**2) / n
    Myy = np.sum(y_centered**2) / n
    Mxy = np.sum(x_centered * y_centered) / n
    Mxz = np.sum(x_centered * (x_centered**2 + y_centered**2)) / n
    Myz = np.sum(y_centered * (x_centered**2 + y_centered**2)) / n
    Mzz = np.sum((x_centered**2 + y_centered**2)**2) / n
    
    # Solve the eigenvector problem
    M = np.array([[Mxx, Mxy, Mxz], [Mxy, Myy, Myz], [Mxz, Myz, Mzz]])
    
    try:
        eigenvals, eigenvecs = np.linalg.eig(M)
        min_idx = np.argmin(eigenvals)
        A, B, C = eigenvecs[:, min_idx]
        
        center_x = -A / (2 * C) + x_mean
        center_y = -B / (2 * C) + y_mean
        radius = np.sqrt((A**2 + B**2) / (4 * C**2) - 1/C)
        
        return center_x, center_y, radius
    except (np.linalg.LinAlgError, ValueError):
        raise ValidationError("Taubin circle fitting failed")


def ransac_circle_fit(x: np.ndarray, y: np.ndarray, 
                     threshold: float = 2.0, max_iterations: int = 1000) -> Tuple[float, float, float, np.ndarray]:
    """
    RANSAC circle fitting for robust estimation.
    
    Returns:
        Tuple of (center_x, center_y, radius, inlier_mask)
    """
    if len(x) < 3:
        raise ValidationError("Need at least 3 points for RANSAC circle fitting")
    
    best_inliers = np.array([], dtype=bool)
    best_center_x, best_center_y, best_radius = 0, 0, 0
    
    for _ in range(max_iterations):
        # Randomly sample 3 points
        sample_indices = np.random.choice(len(x), 3, replace=False)
        sample_x = x[sample_indices]
        sample_y = y[sample_indices]
        
        try:
            center_x, center_y, radius = kasa_circle_fit(sample_x, sample_y)
            
            # Calculate distances from all points to the circle
            distances = np.abs(np.sqrt((x - center_x)**2 + (y - center_y)**2) - radius)
            inliers = distances < threshold
            
            # Keep the best model
            if np.sum(inliers) > np.sum(best_inliers):
                best_inliers = inliers
                best_center_x, best_center_y, best_radius = center_x, center_y, radius
        
        except ValidationError:
            continue
    
    if np.sum(best_inliers) < 3:
        raise ValidationError("RANSAC failed to find sufficient inliers")
    
    return best_center_x, best_center_y, best_radius, best_inliers


def weighted_least_squares_fit(x: np.ndarray, y: np.ndarray, weights: np.ndarray = None) -> Tuple[float, float]:
    """
    Weighted least squares circle center fitting.
    
    Returns:
        Tuple of (center_x, center_y)
    """
    if len(x) < 3:
        raise ValidationError("Need at least 3 points for weighted fitting")
    
    if weights is None:
        weights = np.ones(len(x))
    
    # Weighted centroid calculation
    total_weight = np.sum(weights)
    center_x = np.sum(weights * x) / total_weight
    center_y = np.sum(weights * y) / total_weight
    
    return center_x, center_y


def calculate_angular_coverage(edge_points: List[EdgePoint], center_x: float, center_y: float) -> float:
    """
    Calculate angular coverage of edge points around center.
    
    Returns:
        Angular coverage in degrees
    """
    if len(edge_points) < 2:
        return 0.0
    
    # Calculate angles for each edge point
    angles = []
    for edge in edge_points:
        angle = np.arctan2(edge.y - center_y, edge.x - center_x)
        angles.append(angle)
    
    # Sort angles
    angles = sorted(angles)
    
    # Calculate gaps between consecutive angles
    gaps = []
    for i in range(len(angles)):
        next_i = (i + 1) % len(angles)
        gap = angles[next_i] - angles[i]
        
        # Handle wrap-around
        if gap < 0:
            gap += 2 * np.pi
        gaps.append(gap)
    
    # Find the largest gap (uncovered area)
    largest_gap = max(gaps)
    
    # Coverage is 360° minus the largest gap
    coverage_radians = 2 * np.pi - largest_gap
    coverage_degrees = np.degrees(coverage_radians)
    
    return coverage_degrees


def validate_circular_pattern(edge_points: List[EdgePoint], center_x: float, center_y: float, 
                            expected_radius: float, tolerance: float = 10.0) -> bool:
    """
    Validate that edge points form a reasonable circular pattern.
    
    Returns:
        True if pattern is valid
    """
    if len(edge_points) < 3:
        return False
    
    distances = []
    for edge in edge_points:
        distance = calculate_distance(edge.x, edge.y, center_x, center_y)
        distances.append(distance)
    
    distances = np.array(distances)
    
    # Check if most points are within tolerance of expected radius
    within_tolerance = np.abs(distances - expected_radius) <= tolerance
    valid_fraction = np.sum(within_tolerance) / len(distances)
    
    # Require at least 70% of points to be within tolerance
    return valid_fraction >= 0.7


def calculate_center_consensus(center_estimates: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
    """
    Calculate consensus center from multiple estimation methods.
    
    Args:
        center_estimates: List of (center_x, center_y, weight) tuples
    
    Returns:
        Tuple of (consensus_x, consensus_y, confidence)
    """
    if not center_estimates:
        raise ValidationError("No center estimates provided")
    
    total_weight = sum(weight for _, _, weight in center_estimates)
    
    weighted_x = sum(x * weight for x, _, weight in center_estimates) / total_weight
    weighted_y = sum(y * weight for _, y, weight in center_estimates) / total_weight
    
    # Calculate confidence based on agreement between methods
    distances = []
    for x, y, weight in center_estimates:
        distance = calculate_distance(x, y, weighted_x, weighted_y)
        distances.append(distance)
    
    max_distance = max(distances)
    
    # Confidence decreases with disagreement between methods
    if max_distance <= 1.0:
        confidence = 100.0
    elif max_distance <= 2.0:
        confidence = 90.0
    elif max_distance <= 5.0:
        confidence = 80.0
    else:
        confidence = max(50.0, 100.0 - max_distance * 10)
    
    return weighted_x, weighted_y, confidence