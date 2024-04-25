import asyncio
import websockets

async def receive_message() -> None:
    uri = 'ws://192.168.0.222:9000'
    async with websockets.connect(uri) as websocket:
        while True:
            message = await websocket.recv()
            print(f'Message received: {message}')

asyncio.run(main=receive_message())