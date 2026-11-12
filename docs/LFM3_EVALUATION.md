# LFM3-VL-2B evaluation — 2026-11-12

LFM3-VL-2B runs on this laptop and saves roughly half the GPU memory, but the tested configurations did not improve this project's perception. Keep Qwen3-VL 4B Instruct as the configured model. No navigation code or hardware limits were changed during this evaluation.

## Availability and compatibility

Liquid AI released LFM3 on October 27, 2026. The appropriate candidate for Mode 3 is **LFM3-VL-2B**, a 1.92B-parameter image/video model with a 128K context window. The 256K context claim applies to the text model. Liquid's docs describe object boxes and points normalized to 0–1000, multiple-image prompts, and reasoning that can be disabled for lower latency.

The official GGUF model and projector loaded successfully in the already installed Ollama-bundled llama.cpp runtime, `0.8.0-dev`, commit `39febd2ad`. The OpenAI-compatible API and JSON-schema verification requests worked. The laptop is an RTX 4050 with 6141 MiB VRAM. Observed GPU allocation with one model loaded was 1990 MiB for LFM3 and 4122 MiB for Qwen; these are sampled allocations, not formal peak measurements.

The downloaded checkpoint is `LiquidAI/LFM3-VL-2B-GGUF`, pinned to revision `1add221a5ea6c047fcf1eaaf2d247939d7cf8849`:

- `LFM3-VL-2B-QAD-Q4_0.gguf`: 1,049,615,648 bytes, SHA256 `621c455376cb96a20891697deb9975c6fb47dd5ea98ed3ad1f3b7e8f24f0b1e6`.
- `mmproj-LFM3-VL-2B-F16.gguf`: 378,996,032 bytes, SHA256 `1d8e6e58eefc093e09d6332f081516e1ddbd5b35cfca7f35463fb270db230b1e`.

Both sizes and hashes matched the official repository metadata. Files and runners remain local in `bench_tmp/lfm3_eval/`.

## Comparison on saved images

The runner calls the actual current `VoiceNav.vlm()` and `VoiceNav.choose()` methods through a plain fixture. It creates no ROS node or action clients and submits no navigation, spin, or velocity commands. Models were loaded sequentially, so they did not compete for VRAM.

Each full comparison used 166 image/phrase cases across 16 saved simulation views and four RGB-D frames from last night's real hallway recording. The simulation labels and exclusions come from the existing `~/wheelchair_bench/verify_check.py`; ambiguous views are excluded. The four real frames show the same chair in one hallway and are not four independent trials. They were queried with `chair`, `that chair`, and four absent objects.

The shared pipeline retains its independent red-box label verification and edge-crop check. Images are resized to 448×336; context is 4096; direct answers have a 120-token limit. These checks score whether a verified box is returned for the right object class. They do not measure bounding-box IoU or navigation success.

| Check | Qwen3-VL 4B Instruct | LFM3 QAD Q4, current prompts | LFM3 QAD Q4, documented grounding prompt |
|---|---:|---:|---:|
| Canonical objects detected, 11 present cases | 9/11 | 4/11 | 3/11 |
| Accepted boxes on 81 canonical absent cases | 0 | 0 | 3 |
| Tables detected, 7 present cases | 6/7 | 5/7 | 4/7 |
| Synonym/modifier detections, 13 present cases | 11/13 | 8/13 | 6/13 |
| Accepted boxes on 26 absent synonym cases | 0 | 2 | 1 |
| Real hallway detections, 8 present phrase/frame cases | 8/8 | 8/8 | 8/8 |
| Accepted boxes on 16 real hallway absent cases | 0 | 0 | 0 |
| Correct found-view result, 14 multi-image checks | 9/14 | 3/14 | 3/14 |

The documented-prompt variant used Liquid's normalized-box system prompt and recommended sampling (`temperature=0.1`, `top_p=1`, `top_k=50`, `min_p=0`, `repeat_penalty=1.1`, fixed seed 0). Reasoning was disabled in both full LFM runs. Qwen used the current production launcher and request settings.

The 14 multi-image checks comprise 13 four-view room sets and one **constructed six-image API/ordinal smoke test**, with the refrigerator placed in photo 3. This is not a six-view physical scan. Only whether `found` identifies a labeled target-containing view, or correctly reports none, is scored. The `go` direction is recorded without judging exploration quality. Qwen correctly preferred photo 3 but returned `found: 0` on the constructed test; LFM without reasoning returned `found: 3` there but failed more other checks.

No HTTP/request errors occurred in the full runs. Qwen had one output truncated by the 120-token limit; both LFM runs had none. All such outputs remain in the raw logs and contribute to the measured result.

Median per-case latency over the canonical set, including verification when requested, was 0.262 s for Qwen, 0.436 s for direct LFM, and 0.451 s for the documented-prompt LFM variant. This mixes present and absent cases: it does not establish that every LFM detection is slower, or provide a formal warmed throughput benchmark.

## Bounded reasoning follow-up

A separate LFM run used the documented grounding prompt, explicit `Photo N` labels between images, reasoning enabled with a 256-token budget, and a 376-token total response limit. It rechecked all 39 present cases plus four negatives that had caused trouble, then the same 14 multi-image checks. It is a selected follow-up, not a repeat of the full absent-object suite.

Reasoning recovered some detections: canonical 7/11, tables 6/7, synonyms 11/13, and real hallway phrases 8/8. It still accepted `refrigerator` and `fridge` on `gym_2`, where the target is absent, and only 8/14 found-view results were correct. Median canonical-case latency increased to 2.98 s. There were no request errors or total-length truncations; the server deliberately bounded reasoning. The full BF16 model and unrestricted reasoning were not tested.

For example, direct LFM boxed several sofas but its independent verifier labeled the marked object `cabinet`. Turning off verification would make the detection count look better while accepting more unrelated objects. That guard remains part of the evaluated pipeline.

## Last night's navigation refusal

`logs/live_voice.log` records a verified chair box, a chair sighting at **2.1 m**, and `Going to the chair` at `1794424826.903`. `logs/manual_nav_002711.log` records `Robot to stop due to StopZone polygon` at `1794424828.966`; voice later reported it could not reach the chair. The RGB snapshot visibly contains the chair in the corridor.

The earlier full-turn clearance restriction and the later StopZone activation are navigation/sensor-geometry problems. Better bounding boxes can help depth grounding, but replacing the VLM does not change the collision monitor's input or geometry. The available stationary captures are earlier than the stopped approach, so they do not prove which live returns activated StopZone. Do not label those returns false, remove them, or shrink the safety envelope without evidence.

The next chair-navigation diagnosis should capture the scan used by collision_monitor, its transform into `base_link`, the current stop polygon, and the triggering points together during a stationary test. The model comparison supplies no evidence for relaxing obstacle checks.

## Reproduction and artifacts

The runner is `bench_tmp/lfm3_eval/evaluate.py`. Raw responses, timings, request prompts, schemas, model metadata and source documentation are saved beside it:

- `qwen.json` and `qwen.calls.jsonl`.
- `lfm3.json` and `lfm3.calls.jsonl`.
- `lfm3_documented.json` and `lfm3_documented.calls.jsonl`.
- `lfm3_thinking.json` and `lfm3_thinking.calls.jsonl`.
- Server/evaluation logs, `run_metadata.json`, `model_metadata.json`, and `sources/`.

The evaluated `vlm_nav.py` SHA256 is `97b65289c7f7f7d877b84b4efa266de8ea40ebc2f69e9ec71b72f0384dd660b0`. Existing uncommitted inspection/footprint changes were preserved.

To replay a full direct comparison against an already running local server:

```bash
source setup.bash --skip
.venv-voice/bin/python -B bench_tmp/lfm3_eval/evaluate.py \
  --url http://127.0.0.1:8092/v1/chat/completions \
  --name lfm3_qad_q4 --out bench_tmp/lfm3_eval/lfm3_replay.json
```

For LFM the existing runtime can serve the downloaded files with `--reasoning off`. Use the CUDA backend/library environment from `vlm_server.sh`, the two model paths above, `--mmproj-offload -ngl 99 -c 4096 --port 8092 --host 127.0.0.1 --jinja --cache-ram 0 --parallel 1`. The bounded follow-up instead uses `--reasoning on --reasoning-budget 256 --no-reasoning-preserve` and runner flags `--lfm-prompt --challenging-only --reasoning-tokens 256`.

All model servers started for this evaluation were stopped afterward. No real-chair or Gazebo acceptance run was performed. The current Qwen launcher, its model-identity gate, and navigation parameters remain as configured.

## Official sources

- [Liquid AI's LFM3 release](https://www.liquid.ai/blog/lfm3). Its benchmark table reports visual grounding 77.3 and embodied reasoning 64.4, but does not compare this exact Qwen3-VL 4B setup or this wheelchair pipeline.
- [LFM3-VL-2B model card](https://huggingface.co/LiquidAI/LFM3-VL-2B).
- [Official GGUF checkpoint](https://huggingface.co/LiquidAI/LFM3-VL-2B-GGUF).
- [Vision capabilities: coordinates, grounding and multiple images](https://docs.liquid.ai/lfm/key-concepts/vision-capabilities).
- [Local llama.cpp deployment](https://docs.liquid.ai/deployment/on-device/llama-cpp) and [reasoning/chat template](https://docs.liquid.ai/lfm/key-concepts/chat-template).

Context7 was not available among this session's tools; these sources were fetched directly from the official sites.
