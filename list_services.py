import asyncio
from bleak import BleakClient

ADDRESS = "B0:D2:78:32:EA:6C"  # HM-10 MAC
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
#B0:D2:78:32:EA:6C

async def main():
    async with BleakClient(ADDRESS) as client:
        print("Connected:", client.is_connected)
        services = client.services
        for id in services.characteristics.keys():
            print(f"Service id {id}")
            print(f"char uid: {services.characteristics[id].uuid}, {services.characteristics[id].properties}")

if __name__ == "__main__":
    asyncio.run(main())