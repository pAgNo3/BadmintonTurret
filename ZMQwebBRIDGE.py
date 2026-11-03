import asyncio
import websockets
import zmq
import json

# Setup ZMQ Publisher to talk to the Vision Pipeline
ZMQ_PUB_PORT = 5555
context = zmq.Context()
zmq_pub = context.socket(zmq.PUB)

# Note: In your vision script, the SUB socket binds to 5555, 
# so this PUB socket must 'connect' to it.
zmq_pub.connect(f"tcp://localhost:{ZMQ_PUB_PORT}")

async def ws_handler(websocket):
    print(f"[BRIDGE] Web GUI Dashboard connected.")
    try:
        async for message in websocket:
            try:
                # Parse JSON from Web GUI
                data = json.loads(message)
                
                # If it's a SHOOT command, forward it to ZMQ
                if data.get("command") == "SHOOT":
                    zmq_pub.send_json(data)
                    print(f"[ZMQ TX] Forwarded SHOOT: P:{data.get('pitch')} Y:{data.get('yaw')} S:{data.get('speed')}")
                    
            except json.JSONDecodeError:
                print("[ERROR] Received invalid JSON from WebSocket")
                
    except websockets.exceptions.ConnectionClosed:
        print("[BRIDGE] Web GUI disconnected.")

async def main():
    # Start WebSocket server on port 8080
    async with websockets.serve(ws_handler, "localhost", 8080):
        print("[READY] WebSocket-to-ZMQ Bridge running on ws://localhost:8080")
        print(f"[READY] Forwarding packets to ZMQ tcp://localhost:{ZMQ_PUB_PORT}")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())
