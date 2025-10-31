"""
Receiving robot-teleoperation signal over WebSocket

How to run:

1. Run the receiver (host) script

    python -m examples.lekiwi.ws_receiver

2. Run the sender (client) script

    python -m examples.lekiwi.ws_sender_so101leader_keyboard --host <host-ip-address>

"""

import asyncio
import json
import logging
import time
import socket

from lerobot.utils.robot_utils import busy_wait
from lerobot.robots.lekiwi import LeKiwi, LeKiwiConfig

import websockets

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("TeleopReceiver")

FPS = 30

# shared latest state (thread-safe access via asyncio.Lock)
latest_state = None
latest_lock = asyncio.Lock()


async def websocket_handler(websocket) -> None:
    global latest_state
    # disable Nagle on server side socket
    try:
        sock = websocket.transport.get_extra_info("socket")
        if sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass

    print("Client connected")
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except Exception:
                continue

            # update latest_state quickly and return
            async with latest_lock:
                latest_state = data

            # echo minimal pong so sender can measure RTT
            pong = {"echo_seq": data.get("seq"), "t_send": data.get("t_send")}
            try:
                await websocket.send(json.dumps(pong))
            except Exception:
                pass

    except websockets.ConnectionClosed:
        print("Client disconnected")


async def control_loop(robot):
    global latest_state


    while True:
        t0 = time.perf_counter()
        async with latest_lock:
            state = latest_state

        if state is not None:
            # get action
            action = {x: typ(state[x]) for x, typ in robot.action_features.items() if x in state}

            # send_action may block too -> run in thread
            await asyncio.to_thread(robot.send_action, action)

        # enforce loop rate
        elapsed = time.perf_counter() - t0
        sleep_for = max(0.0, 1.0 / FPS - elapsed)
        if sleep_for > 0.0:
            await asyncio.sleep(sleep_for)


async def main():
    # init robot and pipeline
    robot_config = LeKiwiConfig(port="/dev/ttyACM0", id="my_awesome_lekiwi", cameras={})
    robot = LeKiwi(robot_config)

    robot.connect()
    server = await websockets.serve(websocket_handler, "0.0.0.0", 8765)
    print("Websocket server listening at port 8765")

    # start control loop
    loop = asyncio.get_running_loop()
    loop.create_task(control_loop(robot))

    await server.wait_closed()


if __name__ == "__main__":
    asyncio.run(main())
