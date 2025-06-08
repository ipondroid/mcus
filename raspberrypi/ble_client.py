import asyncio
from bleak import BleakScanner, BleakClient

ENV_SENSING_SERVICE_UUID = "0000181a-0000-1000-8000-00805f9b34fb"
TEMPERATURE_CHAR_UUID = "00002a6e-0000-1000-8000-00805f9b34fb"
HUMIDITY_CHAR_UUID    = "00002a6f-0000-1000-8000-00805f9b34fb"

TARGET_DEVICE_NAME = "TH_DEV"

def temperature_notification_handler(sender, data):
    temp_raw = int.from_bytes(data, byteorder="little", signed=True)
    temp_celsius = temp_raw / 100.0
    print(f"[Notification] Temperature: {temp_celsius:.2f}°C")

def humidity_notification_handler(sender, data):
    hum_raw = int.from_bytes(data, byteorder="little", signed=False)
    humidity = hum_raw / 100.0
    print(f"[Notification] Humidity: {humidity:.2f}%")

async def periodic_read(client):
    while True:
        try:
            temp_raw = await client.read_gatt_char(TEMPERATURE_CHAR_UUID)
            temperpature = int.from_bytes(temp_raw, byteorder="little", signed=True) / 100.0
            print(f"\n[TEMPERATURE_CHAR_VAL] {list(temp_raw)} {temperpature:.2f}")
        except Exception as e:
            print(f"[Read Error - Temperature] {e}")

        await asyncio.sleep(1)

        try:
            hum_raw = await client.read_gatt_char(HUMIDITY_CHAR_UUID)
            humidity = int.from_bytes(hum_raw, byteorder="little", signed=False) / 100.0
            print(f"[HUMIDITY_CHAR_VAL] {list(hum_raw)} {humidity:.2f}")
        except Exception as e:
            print(f"[Read Error - Humidity] {e}")

        await asyncio.sleep(3)

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5)
    target = None
    for d in devices:
        if d.name == TARGET_DEVICE_NAME:
            target = d
            break

    if not target:
        print("TH_DEV not found. Make sure the ESP32 is advertising.")
        return

    print(f"Found device: {target.address}")

    async with BleakClient(target.address) as client:
        print("Connected!")

        services = await client.get_services()
        print("Available services and characteristics:")
        for service in services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                print(f"  Characteristic: {char.uuid} (props: {char.properties})")

  
        read_task = asyncio.create_task(periodic_read(client))

        print("Listening for temperature notifications (press Ctrl+C to exit)...")
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exit.")

