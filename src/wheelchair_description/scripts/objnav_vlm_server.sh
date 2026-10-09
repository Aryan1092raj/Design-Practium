#!/usr/bin/env bash
# Serve Qwen3-VL-2B-Instruct on http://127.0.0.1:8091 for scripts/object_nav.py.
#
# The VLM only answers "is the object in the red box a <target>?" for detector hits, so it
# needs a small context and a small image budget. llama.cpp is built from source with CUDA
# (docs/superpowers/plans/2026-10-09-vlm-object-nav.md, Task 1); set LLAMA and MODELS to
# override the default paths (e.g. on the Jetson).
set -euo pipefail

LLAMA=${LLAMA:-$HOME/llama.cpp/build/bin/llama-server}
MODELS=${MODELS:-$HOME/models/qwen3-vl-2b}
MMPROJ=$(ls "$MODELS"/mmproj*.gguf | head -1)
MODEL=$(ls "$MODELS"/*Q4_K_M*.gguf | grep -v mmproj | head -1)

exec "$LLAMA" --model "$MODEL" --mmproj "$MMPROJ" -ngl 99 -c 4096 \
  --port 8091 --host 127.0.0.1 --no-webui \
  --image-min-tokens 64 --image-max-tokens 512
