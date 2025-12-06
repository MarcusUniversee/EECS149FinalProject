import asyncio
from bleak import BleakScanner

async def main():
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=5.0)
    for d in devices:
        print(d)

asyncio.run(main())