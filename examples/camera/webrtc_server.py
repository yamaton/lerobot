# webrtc_qsv_server_fixed.py
import argparse
import asyncio
import logging
import weakref

from aiohttp import web
import aiohttp_cors
from aiortc import RTCPeerConnection, RTCSessionDescription
from aiortc.contrib.media import MediaPlayer

pcs = weakref.WeakSet()


async def start_ffmpeg_stream(device, out_port, hw_encoder):
    """
    Start ffmpeg to capture the camera and stream an H.264 MPEG-TS stream
    to udp://127.0.0.1:out_port
    """
    # device can be an int (v4l2 index) or a path
    if isinstance(device, int):
        device_path = f"/dev/video{device}"
    else:
        device_path = str(device)

    # choose encoder and options
    if hw_encoder == "qsv":
        encoder = "h264_qsv"
        # For qsv we want nv12 input; do not request hwaccel for capture->encode
        pre_encode_opts = ["-vf", "format=nv12", "-pix_fmt", "nv12"]
    else:
        encoder = "libx264"
        pre_encode_opts = []

    # build command
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "v4l2",
        "-framerate",
        "30",
        "-video_size",
        "1280x720",
        "-i",
        device_path,
    ]

    # apply pre-encoder options (format/pix_fmt)
    cmd += pre_encode_opts

    # encoder options tuned for low-latency streaming
    cmd += [
        "-c:v",
        encoder,
        "-preset",
        "veryfast",
        "-tune",
        "zerolatency",
        "-g",
        "60",
        "-b:v",
        "2M",
        "-maxrate",
        "2M",
        "-bufsize",
        "4M",
        "-f",
        "mpegts",
        f"udp://127.0.0.1:{out_port}",
    ]

    logging.info("Launching ffmpeg: %s", " ".join(cmd))
    proc = await asyncio.create_subprocess_exec(
        *cmd,
        stdin=asyncio.subprocess.DEVNULL,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    # background drain of stderr for debugging
    async def _drain_stderr():
        try:
            while True:
                line = await proc.stderr.readline()
                if not line:
                    break
                logging.debug("ffmpeg: %s", line.decode(errors="ignore").rstrip())
        except Exception:
            pass

    asyncio.create_task(_drain_stderr())
    logging.info(
        "Started ffmpeg (pid=%s) -> udp://127.0.0.1:%s (%s)",
        proc.pid,
        out_port,
        encoder,
    )
    return proc


async def offer(request):
    params = await request.json()
    offer_desc = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

    player = request.app.get("video_player")
    if player is None:
        return web.Response(status=500, text="Video source not ready")

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_connectionstatechange():
        logging.info("Connection state is %s", pc.connectionState)
        if pc.connectionState in ("failed", "closed"):
            await pc.close()
            pcs.discard(pc)
            logging.info("Peer connection closed and removed.")

    if player.video:
        pc.addTrack(player.video)
    else:
        logging.warning("MediaPlayer has no video track")

    await pc.setRemoteDescription(offer_desc)
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response(
        {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}
    )


async def on_startup(app):
    args = app["args"]
    udp_port = args.udp_port
    hw_encoder = "qsv" if args.hw == "qsv" else "none"

    # start ffmpeg
    app["ffmpeg_proc"] = await start_ffmpeg_stream(
        device=args.device, out_port=udp_port, hw_encoder=hw_encoder
    )

    # wait and retry opening MediaPlayer (to avoid race / I/O errors)
    url = f"udp://127.0.0.1:{udp_port}"
    logging.info("Will attempt to open MediaPlayer on %s (decode=False)", url)

    max_tries = 10
    delay = 0.3
    last_exc = None
    from av import error as av_error

    for attempt in range(1, max_tries + 1):
        # check if ffmpeg died
        proc = app.get("ffmpeg_proc")
        if proc and proc.returncode is not None:
            # ffmpeg already exited
            raise RuntimeError(
                f"ffmpeg terminated early with returncode={proc.returncode}"
            )

        try:
            # decode=False -> treat the incoming stream as already-encoded (passthrough)
            player = MediaPlayer(url, format="mpegts", options={"timeout": "3000000"})
            app["video_player"] = player
            logging.info("MediaPlayer opened successfully on attempt %d", attempt)
            return
        except (OSError, av_error.OSError) as exc:
            last_exc = exc
            logging.warning(
                "Failed to open MediaPlayer on attempt %d/%d: %s",
                attempt,
                max_tries,
                exc,
            )
            await asyncio.sleep(delay)

    # all attempts failed
    raise RuntimeError(
        f"Failed to open MediaPlayer after {max_tries} tries: last error: {last_exc}"
    )


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros, return_exceptions=True)

    player = app.get("video_player")
    if player:
        try:
            await player.stop()
        except Exception:
            pass

    proc = app.get("ffmpeg_proc")
    if proc and proc.returncode is None:
        logging.info("Terminating ffmpeg (pid=%s)", proc.pid)
        proc.terminate()
        try:
            await asyncio.wait_for(proc.wait(), timeout=2.0)
        except asyncio.TimeoutError:
            logging.info("Killing ffmpeg (pid=%s)", proc.pid)
            proc.kill()
            await proc.wait()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="WebRTC camera streamer (fixed)")
    parser.add_argument(
        "--device",
        type=int,
        default=0,
        help="v4l2 device index (e.g. 0 for /dev/video0)",
    )
    parser.add_argument("--port", type=int, default=8080, help="HTTP server port")
    parser.add_argument(
        "--udp-port",
        type=int,
        default=5000,
        help="Local UDP port where ffmpeg streams MPEG-TS",
    )
    parser.add_argument(
        "--hw",
        choices=("qsv", "none"),
        default="qsv",
        help="Use h264_qsv hardware encoder or libx264",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    app = web.Application()
    app["args"] = args
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
