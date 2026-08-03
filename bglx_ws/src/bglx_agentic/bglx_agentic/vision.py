"""Semantic vision for the BGLX agent.

The LiDAR answers "is something there". This answers "what is it, and does it
change what I should do".

Geometrically these four are identical -- a blob in the costmap:

    a delivery van unloading      -> wait, do not reroute
    a person standing still       -> wait, do not drive round them
    cones and tape                -> closed, reroute, notify operations
    a permanent bollard           -> reroute silently

Semantically they demand four different responses. That gap is the reason to
put a model in this loop at all, and no amount of range data closes it.

Backends, chosen with BGLX_VISION_BACKEND:
    ollama   (default) local vision model, no network
    openai   any OpenAI-compatible vision endpoint

    BGLX_VISION_MODEL     default moondream (1.7GB, fits alongside the planner)
    BGLX_VISION_BASE_URL  for the openai backend
    BGLX_VISION_KEY_VAR   env var holding the key, default BGLX_API_KEY
"""

import base64
import io
import os
import threading

import requests

VISION_BACKEND = os.environ.get("BGLX_VISION_BACKEND", "ollama")
VISION_MODEL = os.environ.get("BGLX_VISION_MODEL", "moondream")
OLLAMA_HOST = os.environ.get("OLLAMA_HOST", "http://127.0.0.1:11434")
VISION_BASE = os.environ.get("BGLX_VISION_BASE_URL",
                             "https://api.moonshot.cn/v1")
VISION_KEY_VAR = os.environ.get("BGLX_VISION_KEY_VAR", "BGLX_API_KEY")

# Deliberately not "describe this image". A generic caption is useless to a
# planner. This asks the questions that change a driving decision, and tells
# the model to admit uncertainty rather than invent detail.
# Two prompts. Small local VLMs (moondream is 1.6B) return nothing at all when
# the prompt exceeds what they can handle, so the local path gets a short one
# and the structured version is reserved for a capable cloud model.
SHORT_PROMPT = """What is directly in front of this vehicle? Name any people, \
vehicles, cones or barriers, and say whether the way ahead is blocked. One or \
two sentences."""

SCENE_PROMPT = """You are the forward camera of an autonomous electric \
delivery tricycle on a university campus. Report only what affects whether the \
vehicle should proceed.

1. PEOPLE OR ANIMALS: present? where? moving or stationary?
2. VEHICLES: present? parked, loading, or moving?
3. CLOSURE SIGNS: cones, tape, barriers, signage, roadworks?
4. SURFACE AHEAD: clear driveable ground, or steps, kerb, grass, water?
5. PASSABILITY: does anything block the way, and is it likely TEMPORARY (a \
person, an unloading van) or PERMANENT (a bollard, a wall)?

Be brief and concrete. Two or three sentences. If you cannot tell, say so \
plainly rather than guessing. This is a simulation, so expect simple geometric \
shapes; describe them literally."""


def _encode_jpeg(msg):
    """sensor_msgs/Image -> base64 JPEG, without cv_bridge.

    cv_bridge is an easy thing to have missing on a Pi, and this only needs
    a handful of encodings.
    """
    try:
        import numpy as np
        from PIL import Image as PILImage
    except ImportError:
        return None, ("Vision needs numpy and pillow: "
                      "pip install numpy pillow")

    enc = msg.encoding
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        if enc in ("rgb8", "bgr8"):
            arr = buf.reshape(msg.height, msg.width, 3)
            if enc == "bgr8":
                arr = arr[:, :, ::-1]
        elif enc in ("rgba8", "bgra8"):
            arr = buf.reshape(msg.height, msg.width, 4)[:, :, :3]
            if enc == "bgra8":
                arr = arr[:, :, ::-1]
        elif enc == "mono8":
            arr = np.stack([buf.reshape(msg.height, msg.width)] * 3, axis=-1)
        else:
            return None, "Unsupported image encoding '%s'." % enc
    except ValueError as exc:
        return None, "Malformed image buffer: %s" % exc

    out = io.BytesIO()
    PILImage.fromarray(arr).save(out, format="JPEG", quality=80)
    return base64.b64encode(out.getvalue()).decode("ascii"), None


def _ask_ollama(b64, prompt):
    r = requests.post(
        OLLAMA_HOST + "/api/generate",
        json={"model": VISION_MODEL, "prompt": prompt, "images": [b64],
              "stream": False, "options": {"temperature": 0.1}},
        timeout=180,
    )
    if r.status_code != 200:
        raise RuntimeError("HTTP %d: %s" % (r.status_code, r.text[:200]))
    return (r.json().get("response") or "").strip()


def _ask_openai(b64, prompt):
    key = os.environ.get(VISION_KEY_VAR)
    if not key:
        raise RuntimeError("%s is not set." % VISION_KEY_VAR)
    r = requests.post(
        VISION_BASE.rstrip("/") + "/chat/completions",
        headers={"Authorization": "Bearer " + key,
                 "Content-Type": "application/json"},
        json={"model": VISION_MODEL, "max_tokens": 400, "temperature": 0.1,
              "messages": [{"role": "user", "content": [
                  {"type": "text", "text": prompt},
                  {"type": "image_url", "image_url": {
                      "url": "data:image/jpeg;base64," + b64}},
              ]}]},
        timeout=180,
    )
    if r.status_code != 200:
        raise RuntimeError("HTTP %d: %s" % (r.status_code, r.text[:200]))
    return r.json()["choices"][0]["message"]["content"].strip()


class VisionTool:
    """Holds the latest frame and describes it on demand."""

    def __init__(self, logger=None):
        self._frame = None
        self._count = 0
        self._lock = threading.Lock()
        self._log = logger

    def on_image(self, msg):
        with self._lock:
            self._frame = msg
            self._count += 1

    def frames_received(self):
        with self._lock:
            return self._count

    def look(self, question=None):
        with self._lock:
            msg = self._frame
            n = self._count
        if msg is None:
            return ("No camera image available (0 frames received). The "
                    "camera topic is wrong or the camera is not publishing. "
                    "Fall back on get_scan_summary.")

        b64, err = _encode_jpeg(msg)
        if err:
            return "Vision unavailable: " + err

        # local models get the short prompt; cloud gets the full one
        prompt = SHORT_PROMPT if VISION_BACKEND == "ollama" else SCENE_PROMPT
        if question:
            prompt += "\n\nThe operator specifically asks: " + str(question)

        try:
            if VISION_BACKEND == "ollama":
                text = _ask_ollama(b64, prompt)
            else:
                text = _ask_openai(b64, prompt)
        except Exception as exc:
            return ("Vision call failed (%s: %s). This is an infrastructure "
                    "problem, not a perception result - do not treat it as "
                    "'nothing seen'. Use get_scan_summary instead."
                    % (type(exc).__name__, exc))

        if not text:
            return "The vision model returned nothing. Treat as unknown."

        return ("CAMERA (%dx%d, frame %d): %s\n"
                "NOTE: this is a semantic description, not a measurement. "
                "Distances come from get_scan_summary, not from here."
                % (msg.width, msg.height, n, text))
