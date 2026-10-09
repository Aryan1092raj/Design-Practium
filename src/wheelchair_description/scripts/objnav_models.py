"""Models for object search. Heavy imports (torch, ultralytics, open_clip) happen inside the
classes, so the unit tests need no GPU.

Detector: YOLOE with the spoken word as its only class. It stays in PyTorch because a
TensorRT export bakes the class list into the weights.
Verifier: Qwen3-VL-2B on scripts/objnav_vlm_server.sh. It sees the full frame with the box
drawn in red, not a crop; the crop-only check let a wrong "refrigerator" through on 2026-10-04.
"""
import base64
import logging

import cv2
import requests

log = logging.getLogger("objnav_models")

VERIFY_PROMPT = ("Look at the object inside the red box. Is it a {target}? "
                 "Answer only yes or no.")


class Detector:
    def __init__(self, weights, device, conf=0.25):
        from ultralytics import YOLOE
        self.model, self.device, self.conf = YOLOE(weights), device, conf

    def set_target(self, name):
        self.model.set_classes([name])

    def detect(self, rgb):
        r = self.model.predict(rgb[:, :, ::-1], conf=self.conf, device=self.device,
                               half=self.device.startswith("cuda"), retina_masks=True,
                               verbose=False)[0]
        out = []
        for i, box in enumerate(r.boxes.xyxy.cpu().numpy()):
            mask = r.masks.data[i].cpu().numpy() > 0.5 if r.masks is not None else None
            out.append((box.tolist(), float(r.boxes.conf[i]), mask))
        return out


def boxed_jpeg(rgb, box, width=640):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    x1, y1, x2, y2 = (int(v) for v in box)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), max(2, img.shape[1] // 160))
    s = width / img.shape[1]
    img = cv2.resize(img, (width, int(round(img.shape[0] * s))))
    return base64.b64encode(cv2.imencode(".jpg", img)[1]).decode()


def parse_yes(text):
    return text.strip().lstrip("\"'*` ").lower().startswith("yes")


class Verifier:
    def __init__(self, url, timeout=20):
        self.url, self.timeout = url, timeout

    def ping(self):
        try:
            return requests.get(self.url.replace("/v1/chat/completions", "/health"),
                                timeout=2).status_code == 200
        except requests.RequestException:
            return False

    def check(self, rgb, box, target):
        """True only on a clear yes. Errors count as no (ponytail: a server hiccup can reject a
        real object; the search then keeps exploring and sees it again)."""
        try:
            r = requests.post(self.url, timeout=self.timeout, json={
                "temperature": 0, "max_tokens": 8,
                "messages": [{"role": "user", "content": [
                    {"type": "image_url",
                     "image_url": {"url": "data:image/jpeg;base64," + boxed_jpeg(rgb, box)}},
                    {"type": "text", "text": VERIFY_PROMPT.format(target=target)}]}]})
            r.raise_for_status()
            return parse_yes(r.json()["choices"][0]["message"]["content"])
        except (requests.RequestException, KeyError, IndexError, ValueError) as e:
            log.error("VLM check failed: %s", e)
            return False
