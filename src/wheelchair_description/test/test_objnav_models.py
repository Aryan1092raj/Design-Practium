import base64

import cv2
import numpy as np

import objnav_models as m


def test_parse_yes():
    assert m.parse_yes("Yes.")
    assert m.parse_yes(' "yes" ')
    assert m.parse_yes("**Yes**")
    assert not m.parse_yes("No")
    assert not m.parse_yes("")


def test_boxed_jpeg_is_640_wide_with_red_box():
    jpg = base64.b64decode(m.boxed_jpeg(np.zeros((960, 1280, 3), np.uint8), [10, 10, 200, 200]))
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (480, 640)
    b, g, r = img[5, 50]  # top edge of the box after scaling by 0.5
    assert r > 150 and g < 100 and b < 100


class Reply:
    def __init__(self, text):
        self.text = text

    def raise_for_status(self):
        pass

    def json(self):
        return {"choices": [{"message": {"content": self.text}}]}


def test_verifier_sends_image_and_reads_yes(monkeypatch):
    sent = {}

    def post(url, json, timeout):
        sent.update(json)
        return Reply("Yes")

    monkeypatch.setattr(m.requests, "post", post)
    assert m.Verifier("http://x/v1/chat/completions").check(
        np.zeros((480, 640, 3), np.uint8), [1, 1, 50, 50], "sofa") is True
    content = sent["messages"][0]["content"]
    assert content[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "sofa" in content[1]["text"]
    assert sent["temperature"] == 0


def test_verifier_error_counts_as_no(monkeypatch):
    def post(*a, **k):
        raise m.requests.ConnectionError("down")

    monkeypatch.setattr(m.requests, "post", post)
    assert m.Verifier("http://x/v1/chat/completions").check(
        np.zeros((480, 640, 3), np.uint8), [1, 1, 50, 50], "sofa") is False


def test_ping_false_when_server_down(monkeypatch):
    def get(*a, **k):
        raise m.requests.ConnectionError("down")

    monkeypatch.setattr(m.requests, "get", get)
    assert m.Verifier("http://x/v1/chat/completions").ping() is False
