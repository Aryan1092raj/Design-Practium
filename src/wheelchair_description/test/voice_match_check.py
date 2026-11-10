"""Place-matching check for voice_nav.py against the live Ollama SLM (no ROS graph, no driving).

Run: source setup.bash --skip && source .venv-voice/bin/activate &&
     python3 src/wheelchair_description/test/voice_match_check.py
Each phrase must resolve to the expected place, or to None (ignored or "I don't know").
"""
import os
import sys
import threading

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import voice_nav  # noqa: E402

CONFIG = os.path.join(os.path.dirname(__file__), "..", "config")
CASES = {
    "locations.yaml": [  # real chair: names only, no desc
        ("take me to dp lab.", "dp lab"), ("go to the dp lab", "dp lab"), ("dp lab", "dp lab"),
        ("go to optical lab", "optical lab"), ("optical lab please", "optical lab"),
        ("optics lab please", "optical lab"), ("take me to the optical lab now", "optical lab"),
        ("I will do a pickle lab.", None), ("doctor", None), ("take me to the doctor", None),
        ("take me to the chemistry lab", None), ("go to the physics lab", None),
        ("bye bye", None), ("I'll leave.", None), ("you can't do it", None),
        ("what is the weather", None), ("don't go to the dp lab", None),
    ],
    "locations_sim.yaml": [  # sim: each place has a desc
        ("I want to watch TV", "living room"), ("take me where I can cook", "kitchen"),
        ("I need some fresh air", "balcony"), ("I want to sleep", "bedroom"),
        ("take me to the gym", "gym"), ("living room please", "living room"),
        ("take me to the garage", None), ("go to the pickle room", None),
        ("what is the weather", None), ("bye bye", None),
    ],
}


class Log:
    def warn(self, msg): print("  warn:", msg)
    info = error = warn


def make(places):
    v = voice_nav.VoiceNav.__new__(voice_nav.VoiceNav)  # skip Node.__init__: no ROS graph needed
    v.places, v.lock, v.busy, v.cancelled, v.said = places, threading.Lock(), False, threading.Event(), []
    v.get_logger = lambda: Log()
    v.say = v.said.append
    v.picked = []
    def run(name, p):
        v.picked.append(name)
        v.busy = False  # the real run() clears busy when the goal ends
    v.run = run
    return v


def resolve(v, text):
    v.picked.clear()
    v.on_text(text)
    for th in threading.enumerate():  # on_text starts run() in a thread
        if th is not threading.current_thread() and th.daemon:
            th.join(1)
    return v.picked[0] if v.picked else None


fails = 0
for file, cases in CASES.items():
    with open(os.path.join(CONFIG, file)) as f:
        v = make(yaml.safe_load(f))
    for text, want in cases:
        got = resolve(v, text)
        ok = got == want
        fails += not ok
        print(f"{'ok  ' if ok else 'FAIL'} {file:20s} {text!r:40s} -> {got!r} (want {want!r})")
print(f"{fails} failures")
sys.exit(1 if fails else 0)
