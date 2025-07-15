"""Safety parameter definitions and validation."""

from ..models.data_models import SafetyParameters as SafetyParams
from ..exceptions.custom_exceptions import SafetyError


class SafetyParameters:
    """
    Safety parameter manager for production die scanning.
    
    Critical: These parameters are designed to prevent operator injury
    in 2000°F forging operations. DO NOT lower safety thresholds.
    """
    
    @staticmethod
    def get_production_parameters() -> SafetyParams:
        """Get production-grade safety parameters."""
        return SafetyParams(
            expected_die_diameter=114.3,  # 4.5 inches in mm
            die_diameter_tolerance=8.0,   # Reduced tolerance for production
            min_outer_radius=45.0,
            max_outer_radius=70.0,
            min_perimeter_edges=8,       # Increased for production safety
            min_angular_coverage=180.0,   # Increased for production safety
            min_confidence_score=80.0,    # Increased for production safety
            max_center_deviation=12.7,     # Reduced for production safety
            min_edge_quality=0.8          # Increased for production safety
        )
    
    @staticmethod
    def get_testing_parameters() -> SafetyParams:
        """Get testing parameters (still safe but less restrictive)."""
        return SafetyParams(
            expected_die_diameter=114.3,
            die_diameter_tolerance=10.0,
            min_outer_radius=45.0,
            max_outer_radius=70.0,
            min_perimeter_edges=5,
            min_angular_coverage=120.0,
            min_confidence_score=75.0,
            max_center_deviation=12.7,
            min_edge_quality=0.7
        )
    
    @staticmethod
    def validate_parameters(params: SafetyParams) -> None:
        """
        Validate safety parameters meet minimum requirements.
        
        Raises SafetyError if parameters are unsafe.
        """
        min_safe_edges = 5
        min_safe_coverage = 120.0
        min_safe_confidence = 75.0
        max_safe_deviation = 12.7
        
        if params.min_perimeter_edges < min_safe_edges:
            raise SafetyError(
                f"Minimum perimeter edges ({params.min_perimeter_edges}) "
                f"below safety minimum ({min_safe_edges})"
            )
        
        if params.min_angular_coverage < min_safe_coverage:
            raise SafetyError(
                f"Minimum angular coverage ({params.min_angular_coverage}°) "
                f"below safety minimum ({min_safe_coverage}°)"
            )
        
        if params.min_confidence_score < min_safe_confidence:
            raise SafetyError(
                f"Minimum confidence score ({params.min_confidence_score}%) "
                f"below safety minimum ({min_safe_confidence}%)"
            )
        
        if params.max_center_deviation > max_safe_deviation:
            raise SafetyError(
                f"Maximum center deviation ({params.max_center_deviation}mm) "
                f"above safety maximum ({max_safe_deviation}mm)"
            )
        
        if params.die_diameter_tolerance <= 0:
            raise SafetyError("Die diameter tolerance must be positive")
        
        if params.expected_die_diameter <= 0:
            raise SafetyError("Expected die diameter must be positive")
    
    @staticmethod
    def get_recommended_parameters() -> SafetyParams:
        """Get recommended safety parameters for production use."""
        return SafetyParameters.get_production_parameters()