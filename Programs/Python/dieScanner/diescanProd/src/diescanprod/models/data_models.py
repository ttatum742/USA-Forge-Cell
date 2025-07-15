"""Data models for die scanner production package."""

from dataclasses import dataclass
from typing import List
from datetime import datetime


@dataclass
class ScanPoint:
    """Individual scan measurement point."""
    x: float
    y: float 
    z: float
    height: float
    is_valid: bool
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()


@dataclass
class EdgePoint:
    """Die edge detection point."""
    x: float
    y: float
    edge_type: str  # 'drop_off', 'rise', 'stable'
    confidence: float
    
    def __post_init__(self):
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Confidence must be between 0.0 and 1.0")
        
        valid_types = ['drop_off', 'rise', 'stable']
        if self.edge_type not in valid_types:
            raise ValueError(f"Edge type must be one of {valid_types}")


@dataclass
class DieCenter:
    """Final die center calculation result."""
    center_x: float
    center_y: float
    diameter: float
    avg_height: float
    confidence: float
    edge_points: List[EdgePoint]
    calculation_method: str = "multi_method_consensus"
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
            
        if not 0.0 <= self.confidence <= 100.0:
            raise ValueError("Confidence must be between 0.0 and 100.0")
            
        if self.diameter <= 0:
            raise ValueError("Diameter must be positive")


@dataclass
class ScanConfiguration:
    """Scanning configuration parameters."""
    y_scan_distance: float = 120.0  # 5 inches in mm
    scan_step: float = 1.2  # mm
    x_move_small: float = 6.35  # 0.25" in mm
    x_move_large: float = 57.15  # 2.25" in mm
    start_offset_x: float = 125.0  # X offset from calibration
    start_offset_y: float = 110.0  # Y offset from calibration
    refinement_step: float = 0.5  # mm for refinement scanning


@dataclass
class SafetyParameters:
    """Safety validation parameters - NO FALLBACKS ALLOWED."""
    expected_die_diameter: float = 114.3  # 4.5 inches in mm
    die_diameter_tolerance: float = 10.0  # mm
    min_outer_radius: float = 45.0  # mm
    max_outer_radius: float = 70.0  # mm
    min_perimeter_edges: int = 8  # Minimum edges required (PRODUCTION)
    min_angular_coverage: float = 270.0  # Degrees (PRODUCTION)
    min_confidence_score: float = 85.0  # Minimum confidence % (PRODUCTION)
    max_center_deviation: float = 5.0  # Max deviation from expected position
    min_edge_quality: float = 0.7  # Minimum individual edge quality