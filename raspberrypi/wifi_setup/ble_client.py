#!/usr/bin/env python3
"""
ESP32 WiFi Configuration BLE Client
Connects to ESP32 via BLE and configures WiFi credentials
"""

import asyncio
import logging
from bleak import BleakScanner, BleakClient
from typing import Optional, Tuple

WIFI_CONFIG_SERVICE_UUID = "6ba1c7a0-8123-4567-9abc-def012345678"
WIFI_SSID_CHAR_UUID = "6ba1c7a1-8123-4567-9abc-def012345678"
WIFI_PASSWORD_CHAR_UUID = "6ba1c7a2-8123-4567-9abc-def012345678"
WIFI_STATUS_CHAR_UUID = "6ba1c7a3-8123-4567-9abc-def012345678"
WIFI_COMMAND_CHAR_UUID = "6ba1c7a4-8123-4567-9abc-def012345678"

MQTT_BROKER_CHAR_UUID = "6ba1c7b0-8123-4567-9abc-def012345678"
MQTT_USERNAME_CHAR_UUID = "6ba1c7b1-8123-4567-9abc-def012345678"
MQTT_PASSWORD_CHAR_UUID = "6ba1c7b2-8123-4567-9abc-def012345678"
MQTT_CLIENT_ID_CHAR_UUID = "6ba1c7b3-8123-4567-9abc-def012345678"
MQTT_COMMAND_CHAR_UUID = "6ba1c7b4-8123-4567-9abc-def012345678"

ESP32_DEVICE_NAME = "TH_DEV"

class ESP32WiFiConfigClient:
    def __init__(self):
        self.client: Optional[BleakClient] = None
        self.device_address: Optional[str] = None
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    async def scan_for_esp32(self, timeout: int = 10) -> Optional[str]:
        """Scan for ESP32 device and return its address"""
        self.logger.info(f"Scanning for ESP32 device '{ESP32_DEVICE_NAME}'...")
        
        devices = await BleakScanner.discover(timeout=timeout)
        
        for device in devices:
            if device.name == ESP32_DEVICE_NAME:
                self.logger.info(f"Found ESP32 device: {device.address}")
                return device.address
        
        self.logger.warning("ESP32 device not found")
        return None

    async def connect(self, device_address: str = None) -> bool:
        """Connect to ESP32 device"""
        if device_address:
            self.device_address = device_address
        elif not self.device_address:
            self.device_address = await self.scan_for_esp32()
            
        if not self.device_address:
            self.logger.error("No ESP32 device address available")
            return False
        
        try:
            self.logger.info(f"Connecting to ESP32 at {self.device_address}...")
            self.client = BleakClient(self.device_address)
            await self.client.connect()
            
            await asyncio.sleep(1)
            
            services = self.client.services
            service_list = list(services)
            self.logger.info(f"Found {len(service_list)} services:")
            for service in service_list:
                self.logger.info(f"  Service UUID: {service.uuid}")
                for char in service.characteristics:
                    self.logger.info(f"    Characteristic UUID: {char.uuid}")
            
            wifi_service = None
            for service in service_list:
                if service.uuid.lower() == WIFI_CONFIG_SERVICE_UUID.lower():
                    wifi_service = service
                    break
            
            if not wifi_service:
                self.logger.error("WiFi configuration service not found")
                self.logger.error(f"Expected UUID: {WIFI_CONFIG_SERVICE_UUID}")
                self.logger.error("Checking if service might be found by characteristics...")
                
                for service in service_list:
                    for char in service.characteristics:
                        if char.uuid.lower() == WIFI_SSID_CHAR_UUID.lower():
                            self.logger.info(f"Found WiFi SSID characteristic in service: {service.uuid}")
                            wifi_service = service
                            break
                    if wifi_service:
                        break
                
                if not wifi_service:
                    await self.disconnect()
                    return False
            
            self.logger.info("Successfully connected to ESP32")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect to ESP32: {e}")
            return False

    async def disconnect(self):
        """Disconnect from ESP32 device"""
        if self.client and self.client.is_connected:
            await self.client.disconnect()
            self.logger.info("Disconnected from ESP32")

    async def set_wifi_credentials(self, ssid: str, password: str) -> bool:
        """Send WiFi credentials to ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return False
        
        try:
            # Send SSID
            self.logger.info(f"Sending WiFi SSID: {ssid}")
            ssid_bytes = ssid.encode('utf-8')
            await self.client.write_gatt_char(WIFI_SSID_CHAR_UUID, ssid_bytes)
            
            # Send Password
            self.logger.info("Sending WiFi password...")
            password_bytes = password.encode('utf-8')
            await self.client.write_gatt_char(WIFI_PASSWORD_CHAR_UUID, password_bytes)
            
            self.logger.info("WiFi credentials sent successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send WiFi credentials: {e}")
            return False

    async def send_connect_command(self) -> bool:
        """Send connect command to ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return False
        
        try:
            self.logger.info("Sending WiFi connect command...")
            connect_cmd = bytes([1])  # 1 = connect
            await self.client.write_gatt_char(WIFI_COMMAND_CHAR_UUID, connect_cmd)
            self.logger.info("Connect command sent")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send connect command: {e}")
            return False

    async def send_disconnect_command(self) -> bool:
        """Send disconnect command to ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return False
        
        try:
            self.logger.info("Sending WiFi disconnect command...")
            disconnect_cmd = bytes([0])  # 0 = disconnect
            await self.client.write_gatt_char(WIFI_COMMAND_CHAR_UUID, disconnect_cmd)
            self.logger.info("Disconnect command sent")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send disconnect command: {e}")
            return False

    async def get_wifi_status(self) -> Optional[int]:
        """Read WiFi connection status from ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return None
        
        try:
            self.logger.info("Reading WiFi status...")
            status_bytes = await self.client.read_gatt_char(WIFI_STATUS_CHAR_UUID)
            status = int.from_bytes(status_bytes, byteorder='little')
            
            status_text = "Connected" if status == 1 else "Disconnected"
            self.logger.info(f"WiFi status: {status_text} ({status})")
            return status
            
        except Exception as e:
            self.logger.error(f"Failed to read WiFi status: {e}")
            return None

    async def configure_wifi(self, ssid: str, password: str, 
                           wait_for_connection: bool = True) -> bool:
        """Complete WiFi configuration process"""
        self.logger.info("Starting WiFi configuration process...")
        
        if not await self.set_wifi_credentials(ssid, password):
            return False
        
        if not await self.send_connect_command():
            return False
        
        if wait_for_connection:
            self.logger.info("Waiting for WiFi connection...")
            for i in range(10):  # Wait up to 30 seconds
                await asyncio.sleep(3)
                status = await self.get_wifi_status()
                if status == 1:
                    self.logger.info("WiFi connection successful!")
                    return True
                elif status == 0:
                    self.logger.info(f"Still connecting... ({i+1}/10)")
                else:
                    self.logger.warning("Could not read WiFi status")
            
            self.logger.warning("WiFi connection timeout - please check credentials and network")
            return False
        
        return True

    async def set_mqtt_credentials(self, broker_uri: str, username: str, 
                                  password: str, client_id: str) -> bool:
        """Send MQTT credentials to ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return False
        
        try:
            self.logger.info(f"Sending MQTT broker URI: {broker_uri}")
            broker_bytes = broker_uri.encode('utf-8')
            await self.client.write_gatt_char(MQTT_BROKER_CHAR_UUID, broker_bytes)
            
            self.logger.info(f"Sending MQTT username: {username}")
            username_bytes = username.encode('utf-8')
            await self.client.write_gatt_char(MQTT_USERNAME_CHAR_UUID, username_bytes)
            
            self.logger.info("Sending MQTT password...")
            password_bytes = password.encode('utf-8')
            await self.client.write_gatt_char(MQTT_PASSWORD_CHAR_UUID, password_bytes)
            
            self.logger.info(f"Sending MQTT client ID: {client_id}")
            client_id_bytes = client_id.encode('utf-8')
            await self.client.write_gatt_char(MQTT_CLIENT_ID_CHAR_UUID, client_id_bytes)
            
            self.logger.info("MQTT credentials sent successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send MQTT credentials: {e}")
            return False

    async def send_mqtt_set_command(self) -> bool:
        """Send MQTT configuration set command to ESP32"""
        if not self.client or not self.client.is_connected:
            self.logger.error("Not connected to ESP32")
            return False
        
        try:
            self.logger.info("Sending MQTT set configuration command...")
            set_cmd = bytes([1])  # 1 = set configuration
            await self.client.write_gatt_char(MQTT_COMMAND_CHAR_UUID, set_cmd)
            self.logger.info("MQTT set command sent")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to send MQTT set command: {e}")
            return False

    async def configure_mqtt(self, broker_uri: str, username: str = "", 
                           password: str = "", client_id: str = "esp32_001") -> bool:
        """Complete MQTT configuration process"""
        self.logger.info("Starting MQTT configuration process...")
        
        # Send MQTT credentials
        if not await self.set_mqtt_credentials(broker_uri, username, password, client_id):
            return False
        
        # Send set command
        if not await self.send_mqtt_set_command():
            return False
        
        self.logger.info("MQTT configuration completed!")
        return True


async def main():
    """Example usage"""
    client = ESP32WiFiConfigClient()
    
    try:
        if not await client.connect():
            return
        
        await client.get_wifi_status()
        
    finally:
        await client.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
