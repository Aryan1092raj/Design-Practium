#!/usr/bin/env bash
# Serve Qwen2.5-VL 3B on http://127.0.0.1:8091 for scripts/voice_nav.py.
#
# Uses the llama.cpp server bundled with Ollama and the model Ollama downloaded
# (`ollama pull qwen2.5vl:3b`). Ollama itself runs the image encoder on the CPU and forces
# >= 1024 image tokens, which took ~60 s per frame on the laptop; here the encoder runs on the
# GPU with a 64-512 token budget (~0.5 s per frame).
set -euo pipefail

OLLAMA_LIB=/usr/local/lib/ollama
MANIFEST=$HOME/.ollama/models/manifests/registry.ollama.ai/library/qwen2.5vl/3b
DIGEST=$(python3 -c "import json,sys; print(next(l['digest'] for l in json.load(open(sys.argv[1]))['layers'] if l['mediaType'].endswith('model')))" "$MANIFEST")
MODEL=$HOME/.ollama/models/blobs/${DIGEST/:/-}

ollama stop qwen2.5vl:3b 2>/dev/null || true  # free VRAM if Ollama still holds it

GGML_BACKEND_PATH=$OLLAMA_LIB/cuda_v13/libggml-cuda.so \
LD_LIBRARY_PATH=$OLLAMA_LIB/cuda_v13:$OLLAMA_LIB \
exec "$OLLAMA_LIB/llama-server" --model "$MODEL" --mmproj "$MODEL" --mmproj-offload -ngl 99 \
  -c 2048 --port 8091 --host 127.0.0.1 --no-webui --chat-template chatml \
  --image-min-tokens 64 --image-max-tokens 512
