#!/usr/bin/env python3
"""Load downloaded local models before bringing up wheelchair hardware; never pull."""
import argparse
import json
import math
import os
import sys
import urllib.request
from pathlib import Path


def request(path, body=None, port=11434):
    url = f'http://127.0.0.1:{port}' + path
    req = urllib.request.Request(url, None if body is None else json.dumps(body).encode(),
                                 {'Content-Type': 'application/json'})
    # Never route a local model request through a configured proxy.
    with urllib.request.build_opener(urllib.request.ProxyHandler({})).open(req, timeout=120) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', choices=['30', '31'], required=True)
    parser.add_argument('--mic', choices=['true', 'false'], default='true')
    parser.add_argument('--locations')
    args = parser.parse_args()
    os.environ['HF_HUB_OFFLINE'] = '1'
    if args.version == '30':
        manifest = Path.home() / '.ollama/models/manifests/registry.ollama.ai/library/qwen3-vl/4b-instruct'
        layers = json.loads(manifest.read_text())['layers']
        expected = None
        for suffix in ('image.model', 'image.projector'):
            layer = next(row for row in layers if row['mediaType'].endswith(suffix))
            blob = Path.home() / '.ollama/models/blobs' / layer['digest'].replace(':', '-')
            if not blob.is_file() or blob.stat().st_size != layer['size']:
                raise RuntimeError(f'downloaded Qwen3-VL {suffix} is missing or incomplete')
            if suffix == 'image.model':
                expected = blob.name
        models = request('/v1/models', port=8091).get('data', [])
        if not any(Path(row.get('id', '')).name == expected for row in models):
            raise RuntimeError('port 8091 is serving a different model; stop that server and restart v30')
        print('PASS downloaded Qwen3-VL 4B Instruct and projector; correct model served on port 8091', flush=True)
    if args.version == '31':
        tags = request('/api/tags')
        if not any(row.get('name') == 'qwen3:4b' for row in tags.get('models', [])):
            raise RuntimeError('qwen3:4b is not downloaded; run ollama pull qwen3:4b before the session')
        result = request('/api/generate', {'model': 'qwen3:4b', 'prompt': '',
                                          'stream': False, 'keep_alive': '30m'})
        if not result.get('done') or result.get('model') != 'qwen3:4b':
            raise RuntimeError('Qwen3 did not finish loading')
        print('PASS downloaded Qwen3 4B loaded in local Ollama', flush=True)
    if args.locations:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory('wheelchair_description')) / 'config' / args.locations
        places = yaml.safe_load(path.read_text())
        if not isinstance(places, dict) or not places:
            raise RuntimeError(f'no saved places in {path}')
        for name, place in places.items():
            if not isinstance(place, dict) or any(not math.isfinite(float(place[k])) for k in ('x', 'y', 'yaw')):
                raise RuntimeError(f'invalid pose for {name}')
        print('PASS saved places: ' + ', '.join(places), flush=True)
    if args.mic == 'true':
        import numpy as np
        from faster_whisper import WhisperModel
        device, precision = ('cuda', 'float16') if args.version == '31' else ('cpu', 'int8')
        model = WhisperModel('base.en', device=device, compute_type=precision, local_files_only=True)
        # Exercise lazy encoder initialization so missing CUDA libraries fail before hardware startup.
        segments, _ = model.transcribe(np.zeros(16000, dtype=np.float32), language='en', beam_size=1)
        list(segments)
        print(f'PASS downloaded Whisper base.en inference on {device}/{precision}', flush=True)
    else:
        print('PASS text mode: microphone and Whisper loading disabled', flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('FAIL local model preparation: ' + str(exc), file=sys.stderr)
        raise SystemExit(1)
