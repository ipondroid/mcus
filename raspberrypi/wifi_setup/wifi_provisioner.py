#!/usr/bin/env python3
"""
ESP32 WiFi Provisioning Tool
Command-line tool to configure WiFi on ESP32 via BLE
"""

import asyncio
import argparse
import sys
import subprocess
import re
from typing import List, Dict
from ble_client import ESP32WiFiConfigClient


def scan_wifi_networks() -> List[Dict[str, str]]:
    """Scan for available WiFi networks using iwlist"""
    try:
        # Run iwlist scan command
        result = subprocess.run(['sudo', 'iwlist', 'scan'], 
                              capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print(f"Warning: Failed to scan WiFi networks: {result.stderr}")
            return []
        
        networks = []
        current_network = {}
        
        for line in result.stdout.split('\n'):
            line = line.strip()
            
            # New cell (network)
            if 'Cell' in line and 'Address:' in line:
                if current_network and 'ESSID' in current_network:
                    networks.append(current_network)
                current_network = {}
            
            # ESSID (network name)
            elif 'ESSID:' in line:
                match = re.search(r'ESSID:"([^"]*)"', line)
                if match:
                    current_network['ESSID'] = match.group(1)
            
            # Signal quality
            elif 'Quality=' in line:
                match = re.search(r'Quality=(\d+/\d+)', line)
                if match:
                    current_network['Quality'] = match.group(1)
            
            # Encryption
            elif 'Encryption key:' in line:
                if 'on' in line:
                    current_network['Encryption'] = 'Yes'
                else:
                    current_network['Encryption'] = 'No'
        
        if current_network and 'ESSID' in current_network:
            networks.append(current_network)
        
        networks = [n for n in networks if n.get('ESSID')]
        return sorted(networks, 
                     key=lambda x: x.get('Quality', '0/100'), 
                     reverse=True)
        
    except Exception as e:
        print(f"Warning: Could not scan WiFi networks: {e}")
        return []


def print_wifi_networks(networks: List[Dict[str, str]]):
    """Print available WiFi networks in a formatted table"""
    if not networks:
        print("No WiFi networks found or scanning failed")
        return
    
    print("\nAvailable WiFi Networks:")
    print("-" * 60)
    print(f"{'#':<3} {'SSID':<30} {'Quality':<10} {'Security'}")
    print("-" * 60)
    
    for i, network in enumerate(networks, 1):
        ssid = network.get('ESSID', 'Unknown')
        quality = network.get('Quality', 'Unknown')
        encryption = network.get('Encryption', 'Unknown')
        
        if len(ssid) > 28:
            ssid = ssid[:25] + "..."
        
        print(f"{i:<3} {ssid:<30} {quality:<10} {encryption}")
    
    print("-" * 60)


async def interactive_setup():
    """Interactive WiFi and MQTT setup mode"""
    print("ESP32 Configuration Tool - Interactive Mode")
    print("=" * 50)
    
    client = ESP32WiFiConfigClient()
    
    try:
        print("\n1. Connecting to ESP32...")
        if not await client.connect():
            print("[ERROR] Failed to connect to ESP32")
            print("   Make sure ESP32 is powered on and advertising")
            return False
        
        print("[OK] Connected to ESP32")
        
        print("\n2. Checking current WiFi status...")
        status = await client.get_wifi_status()
        if status == 1:
            print("[OK] ESP32 is currently connected to WiFi")
            
            choice = input("\nWould you like to change WiFi settings? (y/N): ")
            if choice.lower() != 'y':
                return True
        else:
            print("[ERROR] ESP32 is not connected to WiFi")
        
        print("\n3. Scanning for WiFi networks...")
        networks = scan_wifi_networks()
        
        if networks:
            print_wifi_networks(networks)
            
            while True:
                try:
                    choice = input("\nSelect network number (or 0 for manual entry): ")
                    choice_num = int(choice)
                    
                    if choice_num == 0:
                        ssid = input("Enter WiFi SSID: ").strip()
                        break
                    elif 1 <= choice_num <= len(networks):
                        ssid = networks[choice_num - 1]['ESSID']
                        break
                    else:
                        print("Invalid selection")
                except ValueError:
                    print("Please enter a number")
        else:
            ssid = input("\nEnter WiFi SSID: ").strip()
        
        if not ssid:
            print("[ERROR] SSID cannot be empty")
            return False
        
        password = input(f"Enter password for '{ssid}': ").strip()
        
        print(f"\n4. Configuring WiFi: {ssid}")
        if await client.configure_wifi(ssid, password):
            print("[OK] WiFi configuration successful!")
        else:
            print("[ERROR] WiFi configuration failed")
            return False
        
        print("\n5. MQTT Broker Configuration")
        mqtt_choice = input("Would you like to configure MQTT broker? (Y/n): ")
        
        if mqtt_choice.lower() != 'n':
            broker_uri = input("Enter MQTT broker URI (e.g., mqtt://192.168.1.100:1883): ").strip()
            if not broker_uri:
                print("[ERROR] Broker URI cannot be empty, skipping MQTT configuration")
            else:
                username = input("Enter MQTT username (optional): ").strip()
                password = input("Enter MQTT password (optional): ").strip()
                client_id = input("Enter MQTT client ID (default: esp32_001): ").strip()
                
                if not client_id:
                    client_id = "esp32_001"
                
                print(f"\n6. Configuring MQTT broker: {broker_uri}")
                if await client.configure_mqtt(broker_uri, username, password, client_id):
                    print("[OK] MQTT configuration successful!")
                else:
                    print("[ERROR] MQTT configuration failed")
                    return False
        
        return True
            
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        return False
    finally:
        await client.disconnect()


async def scan_esp32_devices():
    """Scan for ESP32 devices"""
    print("Scanning for ESP32 devices...")
    
    client = ESP32WiFiConfigClient()
    device_address = await client.scan_for_esp32(timeout=10)
    
    if device_address:
        print(f"[OK] Found ESP32 device at: {device_address}")
        return True
    else:
        print("[ERROR] No ESP32 devices found")
        return False


async def set_wifi_credentials(ssid: str, password: str):
    """Set WiFi credentials non-interactively"""
    print(f"Configuring WiFi: {ssid}")
    
    client = ESP32WiFiConfigClient()
    
    try:
        if not await client.connect():
            print("[ERROR] Failed to connect to ESP32")
            return False
        
        if await client.configure_wifi(ssid, password):
            print("[OK] WiFi configuration successful!")
            return True
        else:
            print("[ERROR] WiFi configuration failed")
            return False
            
    finally:
        await client.disconnect()


async def get_wifi_status():
    """Get current WiFi status from ESP32"""
    print("Getting WiFi status from ESP32...")
    
    client = ESP32WiFiConfigClient()
    
    try:
        if not await client.connect():
            print("[ERROR] Failed to connect to ESP32")
            return False
        
        status = await client.get_wifi_status()
        if status is not None:
            if status == 1:
                print("[OK] ESP32 is connected to WiFi")
            else:
                print("[ERROR] ESP32 is not connected to WiFi")
            return True
        else:
            print("[ERROR] Could not read WiFi status")
            return False
            
    finally:
        await client.disconnect()


async def set_mqtt_config(broker_uri: str, username: str = "", 
                         password: str = "", client_id: str = "esp32_001"):
    """Set MQTT configuration non-interactively"""
    print(f"Configuring MQTT broker: {broker_uri}")
    
    client = ESP32WiFiConfigClient()
    
    try:
        if not await client.connect():
            print("[ERROR] Failed to connect to ESP32")
            return False
        
        if await client.configure_mqtt(broker_uri, username, password, client_id):
            print("[OK] MQTT configuration successful!")
            return True
        else:
            print("[ERROR] MQTT configuration failed")
            return False
            
    finally:
        await client.disconnect()


async def interactive_mqtt_setup():
    """Interactive MQTT setup only"""
    print("ESP32 MQTT Configuration Tool")
    print("=" * 40)
    
    client = ESP32WiFiConfigClient()
    
    try:
        print("\n1. Connecting to ESP32...")
        if not await client.connect():
            print("[ERROR] Failed to connect to ESP32")
            print("   Make sure ESP32 is powered on and advertising")
            return False
        
        print("[OK] Connected to ESP32")
        
        print("\n2. MQTT Broker Configuration")
        broker_uri = input("Enter MQTT broker URI (e.g., mqtt://192.168.1.100:1883): ").strip()
        if not broker_uri:
            print("[ERROR] Broker URI cannot be empty")
            return False
        
        username = input("Enter MQTT username (optional): ").strip()
        password = input("Enter MQTT password (optional): ").strip()
        client_id = input("Enter MQTT client ID (default: esp32_001): ").strip()
        
        if not client_id:
            client_id = "esp32_001"
        
        print(f"\n3. Configuring MQTT broker: {broker_uri}")
        if await client.configure_mqtt(broker_uri, username, password, client_id):
            print("[OK] MQTT configuration successful!")
            return True
        else:
            print("[ERROR] MQTT configuration failed")
            return False
            
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
        return False
    except Exception as e:
        print(f"[ERROR] Unexpected error: {e}")
        return False
    finally:
        await client.disconnect()


def main():
    parser = argparse.ArgumentParser(
        description="ESP32 WiFi and MQTT Configuration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                          # set WiFi + MQTT
  %(prog)s --set-wifi "MyWiFi" "password123"        # Set WiFi credentials
  %(prog)s --set-mqtt "mqtt://192.168.1.100:1883"   # Set MQTT broker
        """
    )
    
    parser.add_argument('--set-wifi', nargs=2, metavar=('SSID', 'PASSWORD'),
                       help='Set WiFi credentials (SSID PASSWORD)')
    
    parser.add_argument('--set-mqtt', metavar='BROKER_URI',
                       help='Set MQTT broker URI (e.g., mqtt://192.168.1.100:1883)')
    
    args = parser.parse_args()
    
    try:
        if args.set_wifi:
            ssid, password = args.set_wifi
            return asyncio.run(set_wifi_credentials(ssid, password))
        elif args.set_mqtt:
            broker_uri = args.set_mqtt
            username = args.mqtt_user or ""
            password = args.mqtt_pass or ""
            client_id = args.mqtt_client
            return asyncio.run(set_mqtt_config(broker_uri, username, password, client_id))
        else:
            return asyncio.run(interactive_setup())
    
    except KeyboardInterrupt:
        print("\nOperation cancelled")
        return False
    except Exception as e:
        print(f"Error: {e}")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
