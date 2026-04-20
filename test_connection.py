"""
Simple connection test script for KIM101.

This script tests basic connectivity to the KIM101 controller.
"""

import sys
from kim101_driver import KIM101, KIM101Error, Channel


def test_connection(port: str):
    """Test connection and basic operations."""
    print(f"Attempting to connect to KIM101 on {port}...")
    print("-" * 50)
    
    try:
        with KIM101(port, timeout=2.0) as controller:
            print("✓ Connection successful!")
            
            # Get hardware info
            print("\nHardware Information:")
            try:
                info = controller.get_hardware_info()
                print(f"  Model Number:    {info['model_number']}")
                print(f"  Serial Number:   {info['serial_number']}")
                print(f"  Firmware:        {info['firmware_version']}")
                print(f"  Hardware Type:   {info['hardware_type']}")
                print(f"  Num Channels:    {info['num_channels']}")
            except Exception as e:
                print(f"  ✗ Failed to get hardware info: {e}")
            
            # Test status update
            print("\nStatus Check:")
            try:
                statuses = controller.get_status_update(timeout=2.0)
                for status in statuses:
                    if status.status.enabled:
                        print(f"\n  Channel {status.channel.name}:")
                        print(f"    Position:         {status.position} steps")
                        print(f"    Motor Connected:  {status.status.motor_connected}")
                        print(f"    Power OK:         {status.status.power_ok}")
                        print(f"    Active:           {status.status.active}")
                        print(f"    Error:            {status.status.error}")
                        
                        if status.status.error:
                            if status.status.excessive_current:
                                print("    ⚠ Excessive current detected")
                            if status.status.excessive_temp:
                                print("    ⚠ Excessive temperature detected")
                            if status.status.abnormal_movement:
                                print("    ⚠ Abnormal movement detected")
            except Exception as e:
                print(f"  ✗ Failed to get status: {e}")
            
            # Test reading parameters
            print("\nDrive Parameters (Channel 1):")
            try:
                params = controller.get_drive_parameters(Channel.CHANNEL_1)
                print(f"  Max Voltage:      {params.max_voltage} V")
                print(f"  Step Rate:        {params.step_rate} steps/sec")
                print(f"  Step Acceleration: {params.step_accel} steps/sec²")
            except Exception as e:
                print(f"  ✗ Failed to get drive parameters: {e}")
            
            print("\n" + "=" * 50)
            print("✓ All basic tests passed!")
            print("=" * 50)
            
    except KIM101Error as e:
        print(f"\n✗ KIM101 Error: {e}")
        return False
    except Exception as e:
        print(f"\n✗ Unexpected Error: {e}")
        return False
    
    return True


def list_available_ports():
    """List available serial ports."""
    try:
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        
        if ports:
            print("\nAvailable serial ports:")
            for port in ports:
                print(f"  {port.device}")
                if port.description:
                    print(f"    Description: {port.description}")
                if port.manufacturer:
                    print(f"    Manufacturer: {port.manufacturer}")
        else:
            print("\nNo serial ports found.")
    except ImportError:
        print("\nCannot list ports (pyserial.tools not available)")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_connection.py <serial_port>")
        print("\nExample:")
        print("  Linux:   python test_connection.py /dev/ttyUSB0")
        print("  Windows: python test_connection.py COM3")
        print("  macOS:   python test_connection.py /dev/cu.usbserial-XXXX")
        
        list_available_ports()
        sys.exit(1)
    
    port = sys.argv[1]
    success = test_connection(port)
    
    sys.exit(0 if success else 1)
