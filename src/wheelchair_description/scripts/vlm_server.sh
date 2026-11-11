#!/usr/bin/env bash
# Serve Qwen3-VL 4B Instruct on http://127.0.0.1:8091 for scripts/vlm_nav.py.
#
# Uses the llama.cpp server bundled with Ollama and the model Ollama downloaded
# (`ollama pull qwen3-vl:4b-instruct`). Ollama itself runs the image encoder on the CPU and forces
# >= 1024 image tokens, which took ~60 s per frame on the laptop; here the encoder runs on the
# GPU with a 64-512 token budget (~0.5 s per frame). About 4.2 GB of VRAM.
# The Instruct tag, not plain qwen3-vl:4b: that one is the Thinking model, which spends ~250
# tokens reasoning before every answer.
set -euo pipefail

OLLAMA_LIB=/usr/local/lib/ollama
MANIFEST=$HOME/.ollama/models/manifests/registry.ollama.ai/library/qwen3-vl/4b-instruct
layer() {  # blob path of the manifest layer whose media type ends with $1
  python3 -c "import json,sys; print(next(l['digest'] for l in json.load(open(sys.argv[1]))['layers'] if l['mediaType'].endswith(sys.argv[2])).replace(':', '-'))" "$MANIFEST" "$1"
}
MODEL=$HOME/.ollama/models/blobs/$(layer image.model)
MMPROJ=$HOME/.ollama/models/blobs/$(layer image.projector)

ollama stop qwen3-vl:4b-instruct 2>/dev/null || true  # free VRAM if Ollama still holds it

GGML_BACKEND_PATH=$OLLAMA_LIB/cuda_v13/libggml-cuda.so \
LD_LIBRARY_PATH=$OLLAMA_LIB/cuda_v13:$OLLAMA_LIB \
exec "$OLLAMA_LIB/llama-server" --model "$MODEL" --mmproj "$MMPROJ" --mmproj-offload -ngl 99 \
  -c 4096 --port 8091 --host 127.0.0.1 --no-webui --jinja \
  --image-min-tokens 64 --image-max-tokens 512 \
  --cache-ram 0 --parallel 1
# --cache-ram 0: the host-RAM prompt cache (default 8 GB) grew the server from 1.5 to 2.2 GB within
# one sim run and earlyoom killed it twice (m3_u, m3_w). Every request is a new image, so the
# cache never hits. One slot for the same reason: vlm_nav sends one request at a time.
