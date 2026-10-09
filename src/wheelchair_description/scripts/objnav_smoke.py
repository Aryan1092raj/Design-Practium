#!/usr/bin/env python3
"""Check the object-search models on one image without ROS: boxes, VLM verdicts, timings.
Run on the laptop and again on the Jetson; the numbers decide the model sizes.

    python3 scripts/objnav_smoke.py photo.jpg sofa --weights ~/models/yoloe-26s-seg.pt
"""
import argparse
import time

import cv2

from objnav_models import Detector, Verifier


def main():
    p = argparse.ArgumentParser()
    p.add_argument("image")
    p.add_argument("target")
    p.add_argument("--weights", default="yoloe-26s-seg.pt")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--url", default="http://127.0.0.1:8091/v1/chat/completions")
    a = p.parse_args()

    rgb = cv2.cvtColor(cv2.imread(a.image), cv2.COLOR_BGR2RGB)
    det = Detector(a.weights, a.device)
    t = time.perf_counter()
    det.set_target(a.target)
    print(f"set_target: {(time.perf_counter() - t) * 1000:.0f} ms")
    det.detect(rgb)  # warm-up
    t, n = time.perf_counter(), 20
    for _ in range(n):
        dets = det.detect(rgb)
    print(f"detector: {(time.perf_counter() - t) / n * 1000:.0f} ms/frame, {len(dets)} boxes")

    ver = Verifier(a.url)
    print(f"VLM server up: {ver.ping()}")
    for box, score, _ in dets[:3]:
        t = time.perf_counter()
        ok = ver.check(rgb, box, a.target)
        print(f"box {[round(v) for v in box]} score {score:.2f} -> VLM {'yes' if ok else 'no'}, "
              f"{(time.perf_counter() - t) * 1000:.0f} ms")


if __name__ == "__main__":
    main()
