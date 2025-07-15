"""Command-line interface for die scanner production package."""

import click
import logging
from typing import Optional

from ..core.scanner import DieScannerProd
from ..config.settings import Settings
from ..exceptions.custom_exceptions import SafetyError, DieScannerError
from ..utils.logging_utils import setup_logger


@click.group()
@click.version_option()
@click.option('--log-level', default='INFO', help='Logging level (DEBUG, INFO, WARNING, ERROR)')
@click.pass_context
def main(ctx, log_level):
    """DieScanner Production CLI - Safety-critical die center detection."""
    ctx.ensure_object(dict)
    ctx.obj['log_level'] = log_level
    
    # Setup logging
    setup_logger('diescanprod', log_level)


@main.command()
@click.option('--config-file', '-c', help='Configuration file path')
@click.option('--production/--testing', default=True, help='Use production or testing safety parameters')
@click.pass_context
def scan(ctx, config_file: Optional[str], production: bool):
    """Execute die scanning sequence."""
    try:
        click.echo("🎯 DieScanner Production - Starting Scan Sequence")
        click.echo("⚠️  SAFETY CRITICAL SYSTEM - 2000°F Forging Operations")
        
        if not production:
            click.echo("⚠️  WARNING: Using TESTING parameters - not for production use!")
        
        # Initialize settings
        settings = Settings()
        
        # Override production mode if specified
        import os
        if not production:
            os.environ['PRODUCTION_MODE'] = 'false'
        
        # Initialize scanner
        scanner = DieScannerProd(settings)
        
        # Execute scan
        click.echo("\n🔄 Executing scan sequence...")
        result = scanner.scan()
        
        # Display results
        click.echo("\n✅ SCAN SUCCESSFUL")
        click.echo(f"📍 Die Center: [{result.center_x:.1f}, {result.center_y:.1f}] mm")
        click.echo(f"📏 Diameter: {result.diameter:.1f} mm")
        click.echo(f"🎯 Confidence: {result.confidence:.1f}%")
        click.echo(f"📊 Edge Points: {len(result.edge_points)}")
        
        if result.confidence >= 95.0:
            click.echo("🏆 EXCELLENT confidence - Safe for operation")
        elif result.confidence >= 90.0:
            click.echo("✅ GOOD confidence - Safe for operation")
        else:
            click.echo("⚠️  MARGINAL confidence - Review results")
        
    except SafetyError as e:
        click.echo(f"\n❌ SAFETY VIOLATION: {e}", err=True)
        click.echo("🛑 SCAN ABORTED - Do NOT proceed with robot operation", err=True)
        raise click.Abort()
    except DieScannerError as e:
        click.echo(f"\n❌ SCAN FAILED: {e}", err=True)
        raise click.Abort()
    except Exception as e:
        click.echo(f"\n💥 UNEXPECTED ERROR: {e}", err=True)
        raise click.Abort()


@main.command()
@click.pass_context 
def test_comms(ctx):
    """Test robot and sensor communications."""
    try:
        click.echo("🔍 Testing Communications...")
        
        settings = Settings()
        scanner = DieScannerProd(settings)
        
        success = scanner.test_communications()
        
        if success:
            click.echo("✅ All communications OK")
        else:
            click.echo("❌ Communication test failed", err=True)
            raise click.Abort()
            
    except Exception as e:
        click.echo(f"❌ Communication test error: {e}", err=True)
        raise click.Abort()


@main.command()
@click.pass_context
def config(ctx):
    """Show current configuration."""
    try:
        click.echo("⚙️  DieScanner Production Configuration")
        click.echo("=" * 50)
        
        settings = Settings()
        
        click.echo(f"Robot IP: {settings.robot_ip}")
        click.echo(f"Robot Port: {settings.robot_port}")
        click.echo(f"Arduino Port: {settings.arduino_port}")
        click.echo(f"Arduino Baud: {settings.arduino_baud}")
        click.echo(f"Production Mode: {settings.is_production_mode()}")
        click.echo(f"Log Level: {settings.log_level}")
        
        click.echo("\n🛡️  Safety Parameters:")
        safety = settings.safety_params
        click.echo(f"Min Perimeter Edges: {safety.min_perimeter_edges}")
        click.echo(f"Min Angular Coverage: {safety.min_angular_coverage}°")
        click.echo(f"Min Confidence Score: {safety.min_confidence_score}%")
        click.echo(f"Max Center Deviation: {safety.max_center_deviation}mm")
        click.echo(f"Expected Die Diameter: {safety.expected_die_diameter}mm")
        
        click.echo("\n📏 Scan Configuration:")
        scan_config = settings.scan_config
        click.echo(f"Y Scan Distance: {scan_config.y_scan_distance}mm")
        click.echo(f"Scan Step Size: {scan_config.scan_step}mm")
        click.echo(f"Refinement Step: {scan_config.refinement_step}mm")
        
    except Exception as e:
        click.echo(f"❌ Configuration error: {e}", err=True)
        raise click.Abort()


@main.command()
@click.pass_context
def validate_config(ctx):
    """Validate current configuration for safety."""
    try:
        click.echo("🔍 Validating Configuration...")
        
        settings = Settings()
        
        # Validate settings
        settings.validate_settings()
        
        click.echo("✅ Configuration validation passed")
        click.echo("🛡️  All safety parameters meet requirements")
        
        if settings.is_production_mode():
            click.echo("🏭 PRODUCTION MODE - Full safety validation active")
        else:
            click.echo("🧪 TESTING MODE - Reduced safety thresholds")
        
    except Exception as e:
        click.echo(f"❌ Configuration validation failed: {e}", err=True)
        click.echo("🛑 Fix configuration before running scans", err=True)
        raise click.Abort()


@main.command()
@click.option('--output-file', '-o', help='Output file for results (optional)')
@click.pass_context
def calibrate(ctx, output_file: Optional[str]):
    """Calibrate sensor at current position."""
    try:
        click.echo("🎯 Starting Sensor Calibration...")
        
        settings = Settings()
        scanner = DieScannerProd(settings)
        
        # Connect systems
        scanner._connect_systems()
        
        try:
            # Perform calibration
            scanner._calibrate_sensor()
            click.echo("✅ Sensor calibration completed successfully")
            
            # Take test readings
            click.echo("📊 Taking validation readings...")
            heights = scanner.sensor.read_multiple_heights(10, 0.1)
            
            import numpy as np
            avg_height = np.mean(heights)
            std_height = np.std(heights)
            
            click.echo(f"📏 Average Height: {avg_height:.2f}mm")
            click.echo(f"📊 Standard Deviation: {std_height:.2f}mm")
            
            if std_height < 0.1:
                click.echo("🏆 EXCELLENT calibration stability")
            elif std_height < 0.2:
                click.echo("✅ GOOD calibration stability")
            else:
                click.echo("⚠️  MARGINAL calibration stability - check sensor")
            
            # Save results if requested
            if output_file:
                with open(output_file, 'w') as f:
                    f.write(f"Calibration Results\n")
                    f.write(f"Average Height: {avg_height:.2f}mm\n")
                    f.write(f"Standard Deviation: {std_height:.2f}mm\n")
                    f.write(f"Individual Readings: {', '.join(f'{h:.2f}' for h in heights)}\n")
                click.echo(f"📄 Results saved to {output_file}")
        
        finally:
            scanner._disconnect_systems()
            
    except Exception as e:
        click.echo(f"❌ Calibration failed: {e}", err=True)
        raise click.Abort()


if __name__ == '__main__':
    main()