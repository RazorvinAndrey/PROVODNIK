import asyncio
import websockets

clients = set()

async def server(websocket, path):
    clients.add(websocket)
    print("New client connected:", websocket.remote_address)

    try:
        async for message in websocket:
            # Пересылаем сообщение другим клиентам
            for client in clients:
                if client != websocket:
                    await client.send(message)
    except websockets.exceptions.ConnectionClosedError:
        print("Client", websocket.remote_address, "disconnected")
    finally:
        clients.remove(websocket)

# Запуск сервера
start_server = websockets.serve(server, "localhost", 8765)

print("WebSocket server is running. Waiting for connections...")

# Создаем цикл событий
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
