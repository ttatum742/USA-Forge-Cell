from cpppo.server.enip.get_attribute import proxy_simple
import numpy as np
import time
import logging
from typing import List, Optional

## Debug logging
# logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
# logger = logging.getLogger(__name__)

## Error logging 
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

## Info Logging
#logging.basicConfig(level=logging.INFO)
#logger = logging.getLogger(__name__)
  
  
class TorqueDerivativeCalculator(proxy_simple):
    PARAMETERS = dict(proxy_simple.PARAMETERS,
        j1t=proxy_simple.parameter('@0x6B/1/50', 'DINT', 'Nm'),
        j2t=proxy_simple.parameter('@0x6B/1/51', 'DINT', 'Nm'),
        j3t=proxy_simple.parameter('@0x6B/1/52', 'DINT', 'Nm'),
        j4t=proxy_simple.parameter('@0x6B/1/53', 'DINT', 'Nm'),
        j5t=proxy_simple.parameter('@0x6B/1/54', 'DINT', 'Nm'),
        j6t=proxy_simple.parameter('@0x6B/1/55', 'DINT', 'Nm'),
        j1dt=proxy_simple.parameter('@0x6B/1/80', 'DINT', 'Nm/s'),  
        j2dt=proxy_simple.parameter('@0x6B/1/81', 'DINT', 'Nm/s'),
        j3dt=proxy_simple.parameter('@0x6B/1/82', 'DINT', 'Nm/s'),
        j4dt=proxy_simple.parameter('@0x6B/1/83', 'DINT', 'Nm/s'),
        j5dt=proxy_simple.parameter('@0x6B/1/84', 'DINT', 'Nm/s'),
        j6dt=proxy_simple.parameter('@0x6B/1/85', 'DINT', 'Nm/s')
    )

    def __init__(self, ip_address: str):
        super().__init__(host=ip_address, timeout=5.0)
        self.ip_address = ip_address
        self.prev_torques = None
        self.alpha = 0.3  # Filter coefficient for derivatives
        self.filtered_derivatives = np.zeros(6)
        logger.info("Robot connection initialized")
        print("Robot connection initialized")

    def _read_parameter(self, param_name: str) -> Optional[float]:
        """Read a single parameter value"""
        try:
            param_str = self.parameter_substitution(param_name)
            result, = self.read(param_str, checking=True)
            return float(result[0]) if isinstance(result, list) else float(result)
        except Exception as e:
            logger.error(f"Failed to read {param_name}: {str(e)}", exc_info=True)
            return None

    def _read_torques(self) -> Optional[List[float]]:
        """Read all joint torques"""
        torques = []
        for joint in ['j1t', 'j2t', 'j3t', 'j4t', 'j5t', 'j6t']:
            torque = self._read_parameter(joint)
            if torque is None:
                return None
            torques.append(torque)
        return torques

    def _write_derivatives(self, derivatives: np.ndarray) -> bool:
        """Write torque derivatives to the robot controller"""
        try:
            for i, deriv in enumerate(derivatives):
                param_name = ['j1dt', 'j2dt', 'j3dt', 'j4dt', 'j5dt', 'j6dt'][i]
                self._write_parameter(param_name, deriv)
            return True
        except Exception as e:
            logger.error(f"Failed to write derivatives: {str(e)}", exc_info=True)
            return False

    def _write_parameter(self, param_name: str, value: float) -> None:
        """Write a single parameter value"""
        try:
            # Get the parameter string
            param = '%s = (DINT) %s' % (param_name, int(value))
            success, = self.write(
                self.parameter_substitution(param), checking=True)
            if success: print(f"Successfully wrote {param_name}: {value:.2f}")
            logger.info(f"Wrote {param_name}: {value:.2f}")
        except Exception as e:
            logging.warning("Failed to write %s: %s", param_name, str(e), exc_info=True)
            self.close_gateway(exc=e)
            raise

    def _low_pass_filter(self, new_derivatives: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to derivatives"""
        self.filtered_derivatives = (self.alpha * new_derivatives + 
                                   (1 - self.alpha) * self.filtered_derivatives)
        return self.filtered_derivatives

    def _low_pass_filter_torques(self, new_torques: np.ndarray) -> np.ndarray:
        """Apply low-pass filter to raw torques before differentiation"""
        if not hasattr(self, 'filtered_torques'):
            self.filtered_torques = new_torques  # Initialize on first run
        self.filtered_torques = (self.alpha * new_torques + 
                               (1 - self.alpha) * self.filtered_torques)
        return self.filtered_torques
    
    def calculate_derivatives(self) -> Optional[np.ndarray]:
        """Calculate and return filtered torque derivatives"""
        current_torques = self._read_torques()
        if current_torques is None:
            return None
    
        current_torques = np.array(current_torques)
        
        # Apply low-pass filter to torques BEFORE differentiation
        filtered_torques = self._low_pass_filter_torques(current_torques)
        
        if self.prev_torques is not None:
            dt = 1.0 / self.update_rate
            derivatives = (filtered_torques - self.prev_torques) / dt
            
            # Write to robot (no additional filtering needed)
            self._write_derivatives(derivatives)
            return derivatives
        
        self.prev_torques = filtered_torques  # Store filtered torques for next iteration
        return None

    def run(self, update_rate: float = 500.0):
        """Main control loop for torque derivative calculation"""
        self.update_rate = update_rate
        logger.info(f"Starting torque derivative calculation at {update_rate}Hz...")
        
        try:
            while True:
                start_time = time.time()
                
                # Calculate and write derivatives
                derivatives = self.calculate_derivatives()
                if derivatives is None:
                    logger.warning("Failed to calculate derivatives - retrying...")
                    time.sleep(0.1)
                    continue
                
                # Maintain cycle rate
                elapsed = time.time() - start_time
                sleep_time = max(0, (1.0/update_rate) - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("\nStopping torque derivative calculator...")
        except Exception as e:
            logger.error(f"Fatal error: {e}")


if __name__ == "__main__":
    try:
        # Initialize with robot IP address
        logger.info("Initializing torque derivative calculator...")
        print("Initializing torque derivative calculator")
        calculator = TorqueDerivativeCalculator(ip_address="192.168.0.1")
        
        # Start main loop 
        logger.info("Starting main loop...")
        print("Starting main loop")
        calculator.run()
    except Exception as e:
        logger.error(f"Failed to initialize: {e}")
        print("failed to init")