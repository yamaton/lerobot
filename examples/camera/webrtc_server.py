#!/usr/bin/env python3
"""
gst_webrtc_server.py
Simple GStreamer-based WebRTC broadcaster using webrtcbin.

- Captures from /dev/video{device}
- Encodes with qsvh264enc (if available) or x264enc fallback
- Payloads with rtph264pay and sends into webrtcbin
- Signaling endpoint: POST /offer expecting JSON {"sdp": "...", "type":"offer"}
  Responds with JSON {"sdp": "...", "type":"answer"}
"""

import argparse
import asyncio
import logging
import threading
import json
from aiohttp import web

# GObject / GStreamer imports (must be after standard libs)
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstWebRTC", "1.0")
gi.require_version("GstSdp", "1.0")

from gi.repository import Gst, GLib, GstWebRTC, GstSdp

# Initialize GStreamer
Gst.init(None)

log = logging.getLogger("gst-webrtc")
logging.basicConfig(level=logging.INFO)

# Globals (protected by GLib mainloop calls)
GST_LOOP = None
PIPELINE = None
WEBRTC = None  # the webrtcbin element


async def index(request):
    return web.Response(text="GStreamer webrtcbin server is running.\n")


def build_pipeline(device: int, width: int, height: int, framerate: int, use_qsv: bool):
    """
    Build the GStreamer pipeline string. We name the payloader 'pay0' and the webrtc element 'webrtcbin'.
    """
    # choose encoder element
    if use_qsv:
        enc = "qsvh264enc bitrate=2000"  # qsvh264enc commonly exposes 'bitrate' in kbps
    else:
        # software fallback
        enc = "x264enc bitrate=2000 speed-preset=ultrafast tune=zerolatency"

    # pipeline description; note config-interval=1 so SPS/PPS are sent in-band
    pipeline_desc = (
        f"v4l2src device=/dev/video{device} ! "
        f"videoscale ! videoconvert ! "
        f"video/x-raw,format=I420,width={width},height={height},framerate={framerate}/1 ! "
        f"queue ! {enc} ! h264parse ! rtph264pay config-interval=1 name=pay0 pt=96 "
        f"! application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
        f"webrtcbin name=webrtcbin stun-server=stun://stun.l.google.com:19302"
    )
    log.info("Pipeline: %s", pipeline_desc)
    return Gst.parse_launch(pipeline_desc)


def gst_main_start(device, width, height, framerate, use_qsv):
    """
    Create pipeline and run GLib main loop in this thread.
    """
    global GST_LOOP, PIPELINE, WEBRTC

    log.info("Starting GLib main loop and creating pipeline...")
    GST_LOOP = GLib.MainLoop()

    # Build pipeline
    PIPELINE = build_pipeline(device, width, height, framerate, use_qsv)

    # get webrtcbin element
    WEBRTC = PIPELINE.get_by_name("webrtcbin")
    if WEBRTC is None:
        raise RuntimeError("webrtcbin element not found in pipeline!")

    # you can connect signals here for logging ICE candidates if you wish
    def on_ice_candidate(_webrtc, mlineindex, candidate):
        log.debug(
            "webrtc on-ice-candidate mline=%s candidate=%s", mlineindex, candidate
        )
        # in this server implementation we don't forward candidates to the client
        # (ICE trickle isn't required for simple LAN tests). If needed, expose an endpoint.

    WEBRTC.connect("on-ice-candidate", on_ice_candidate)

    # start pipeline
    ret = PIPELINE.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        raise RuntimeError("Failed to start pipeline")

    log.info("Pipeline started, entering GLib MainLoop")
    try:
        GST_LOOP.run()
    finally:
        PIPELINE.set_state(Gst.State.NULL)
        log.info("GLib MainLoop exited, pipeline stopped")


def ensure_sdp_message(sdp_text: str):
    """
    Parse SDP text into GstSdp.SDPMessage.
    Try a couple of available helpers for compatibility.
    """
    # Try the convenient constructor if available
    try:
        res, sdpmsg = GstSdp.SDPMessage.new_from_text(sdp_text)
        if res != GstSdp.SDPResult.OK:
            raise RuntimeError("SDP parse returned non-OK")
        return sdpmsg
    except Exception:
        # fallback to parse_buffer (older bindings)
        res, sdpmsg = GstSdp.SDPMessage.new()
        # parse_buffer expects bytes
        GstSdp.SDPMessage.parse_buffer(sdpmsg, bytes(sdp_text.encode()))
        return sdpmsg


def handle_offer_in_gst(offer_sdp: str, loop, fut: asyncio.Future):
    """
    This function runs in the GLib thread (scheduled via GLib.idle_add).
    It:
      - builds a GstWebRTC.WebRTCSessionDescription from the incoming offer SDP
      - calls set-remote-description on webrtcbin
      - creates an answer (webrtcbin emits create-answer)
      - sets local description and returns the answer SDP (via the asyncio Future)
    """
    global WEBRTC

    log.info("GStreamer: handling remote offer (in GLib thread)")

    try:
        # Parse SDP to GstSdp.SDPMessage
        sdpmsg = ensure_sdp_message(offer_sdp)

        # Build WebRTCSessionDescription
        offer = GstWebRTC.WebRTCSessionDescription.new(
            GstWebRTC.WebRTCSDPType.OFFER, sdpmsg
        )

        # Set remote description (block until complete)
        promise = Gst.Promise.new()
        WEBRTC.emit("set-remote-description", offer, promise)
        promise.interrupt()  # ensure it doesn't block forever in odd cases

        # Create answer (block until the answer is ready)
        promise = Gst.Promise.new()
        WEBRTC.emit("create-answer", None, promise)
        reply = promise.get_reply()
        answer = reply.get_value("answer")

        # Set the local description with the generated answer so ICE/dtls states are correct
        WEBRTC.emit("set-local-description", answer, Gst.Promise.new())

        # Convert answer.sdp (GstSdp.SDPMessage) back to text.
        # Try the high-level as_text() if available, else fall back to serializing manually.
        sdp_text = None
        try:
            sdp_text = answer.sdp.as_text()
        except Exception:
            try:
                # helper function if available in this binding
                sdp_text = GstSdp.sdp_message_as_text(answer.sdp)
            except Exception:
                # Last resort: use manual conversion (shouldn't normally be needed)
                sdp_text = str(answer.sdp)

        # Deliver answer back to asyncio future thread-safely
        loop.call_soon_threadsafe(fut.set_result, {"sdp": sdp_text, "type": "answer"})
        log.info("GStreamer: answer created and returned to HTTP handler")
    except Exception as exc:
        log.exception("Exception while handling offer in GStreamer: %s", exc)
        loop.call_soon_threadsafe(fut.set_exception, exc)

    # Return False to stop the idle handler from repeating
    return False


async def offer_handler(request):
    """
    HTTP handler: receives client's offer SDP and returns the answer SDP produced by GStreamer.
    """
    data = await request.json()
    if "sdp" not in data:
        return web.Response(status=400, text="Missing sdp")

    offer_sdp = data["sdp"]
    loop = asyncio.get_event_loop()
    fut = loop.create_future()

    # Schedule the SDP handling on the GLib main loop thread using idle_add
    GLib.idle_add(handle_offer_in_gst, offer_sdp, loop, fut)

    # Wait for the answer result (set by the GLib thread)
    try:
        answer = await asyncio.wait_for(fut, timeout=10.0)
        return web.json_response(answer)
    except asyncio.TimeoutError:
        return web.Response(status=504, text="Timed out creating answer")
    except Exception as exc:
        return web.Response(status=500, text=f"Error creating answer: {exc}")


def start_gst_thread(device, width, height, framerate, use_qsv):
    t = threading.Thread(
        target=gst_main_start,
        args=(device, width, height, framerate, use_qsv),
        daemon=True,
    )
    t.start()
    return t


def main():
    parser = argparse.ArgumentParser(description="GStreamer webrtcbin camera server")
    parser.add_argument(
        "--device",
        type=int,
        default=0,
        help="v4l2 device index (default 0 -> /dev/video0)",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--framerate", type=int, default=30)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--hw",
        choices=("qsv", "none"),
        default="qsv",
        help="Use qsv encoder or software x264",
    )
    args = parser.parse_args()

    use_qsv = args.hw == "qsv"

    # Start GStreamer pipeline and GLib main loop in background thread
    start_gst_thread(args.device, args.width, args.height, args.framerate, use_qsv)

    # Start aiohttp server for signaling
    app = web.Application()
    app.add_routes([web.get("/", index), web.post("/offer", offer_handler)])

    log.info("Starting HTTP server on 0.0.0.0:%s", args.port)
    web.run_app(app, host="0.0.0.0", port=args.port)


if __name__ == "__main__":
    main()
