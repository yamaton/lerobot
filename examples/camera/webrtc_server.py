import argparse
import asyncio
import logging
import weakref

import cv2
import aiohttp_cors
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from av import VideoFrame
from aiohttp import web


class CameraStreamTrack(VideoStreamTrack):
    """
    A video track that captures frames from a camera device.
    """

    def __init__(self, device):
        super().__init__()
        # The VideoCapture object will be created later, in an async context
        self.device = device
        self.cap = None
        self.last_frame = None
        self._task = None

    async def start_capture(self):
        """Asynchronously start the camera capture."""
        self.cap = cv2.VideoCapture(self.device)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video source: {self.device}")
        self._task = asyncio.create_task(self._read_frames())
        print("Camera capture started.")

    async def _read_frames(self):
        while self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            self.last_frame = frame
            await asyncio.sleep(1 / 30)  # ~30fps
        print("Camera capture task finished.")

    async def recv(self):
        pts, time_base = await self.next_timestamp()

        while self.last_frame is None:
            await asyncio.sleep(0.01)

        vframe = VideoFrame.from_ndarray(self.last_frame, format="bgr24")
        vframe.pts = pts
        vframe.time_base = time_base
        return vframe

    def stop(self):
        if self._task:
            self._task.cancel()
        if self.cap and self.cap.isOpened():
            self.cap.release()
        print("Camera track stopped and released.")


pcs = weakref.WeakSet()


async def offer(request):
    params = await request.json()
    offer_desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    # Get the video track from the app context
    video_track = request.app["video_track"]

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        print(f"Connection state is {pc.connectionState}")
        if pc.connectionState == "failed" or pc.connectionState == "closed":
            await pc.close()
            pcs.discard(pc)
            print("Peer connection closed and removed.")

    pc.addTrack(video_track)

    await pc.setRemoteDescription(offer_desc)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
    )


async def on_startup(app):
    """Create and start the camera track when the app starts."""
    device = app["args"].device
    video_track = CameraStreamTrack(device=device)
    await video_track.start_capture()
    app["video_track"] = video_track


async def on_shutdown(app):
    """Stop the camera track and close peer connections on shutdown."""
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)

    if "video_track" in app:
        app["video_track"].stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC camera streamer")
    parser.add_argument(
        "--device", type=int, default=0, help="Device index for the camera"
    )
    parser.add_argument(
        "--port", type=int, default=8080, help="Port to run the server on"
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    app = web.Application()
    app["args"] = args  # Store args in the app context
    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)
    app.router.add_post("/offer", offer)

    cors = aiohttp_cors.setup(
        app,
        defaults={
            "*": aiohttp_cors.ResourceOptions(
                allow_credentials=True,
                expose_headers="*",
                allow_headers="*",
                allow_methods=["POST", "OPTIONS"],
            )
        },
    )
    for route in list(app.router.routes()):
        cors.add(route)

    web.run_app(app, port=args.port, host="0.0.0.0")
