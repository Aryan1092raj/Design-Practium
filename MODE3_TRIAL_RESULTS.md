# Mode 3 first real-trial preparation — 2026-11-11

The user requested a limited first hardware trial on a chair, door or box and excluded television performance from this trial. The final simulator smoke run reached **2/2** chair/door requests. Full five-target Mode 3 acceptance and real-chair validation remain open. A box has not been validated in this simulator.

| Final run `m3_large_trial_b` | Gap from the retained 0.45 m front probe | Heading error | Recoveries | Physics obstacle contacts |
|---|---:|---:|---:|---:|
| Chair | 0.569 m | 16.2° | 0 | 0 |
| Door panel | 0.943 m | 41.9° | 0 | 0 |

Both satisfy the unchanged 1 m / 45° thresholds. A door means approaching and facing a physical panel, including its visible end; it does not mean opening or traversing it. All 14 CAD collision telemetry topics were advertised, and floor-support contact streams stayed live through both requests. There were zero sampled map overlaps. Five predicted collision-ahead stops occurred on the chair route; smoothness is not established by this check. Moving localization p95 was 0.1335 m and 0.707°.

The live stop check canceled an active search spin and reached zero wheel command in 0.19 s, with no resumed motion. The final benchmark exited normally. All ten packages built. The real localization launch was evaluated without executing hardware actions; an isolated live ZUPT node passed 160 synthetic turn samples with the offset base pose and lateral twist. The real parameter gate passed on that isolated graph.

Mode 3 now uses a conservatively enclosing CAD footprint, padded axle turn room, a corrected real ZUPT base reference and matching AMCL motion model, and checks actual front RGB-D streaming before voice starts. The speed remains 0.40 m/s. Simulator and test processes are stopped; the VLM server remains healthy. No real-chair launch or movement was performed.

Run `./scripts/mode3_real.sh` in an interactive terminal. Power the Arduino first, connect front D455 over USB 3 directly to the laptop, keep the chair still during the two-minute scan check, set/align the RViz pose, then press Enter. Start with a visible chair or door on open floor and keep the power cut within reach. Say “take me to the chair”, “go to the door” or “stop”. Ctrl+C shuts down voice before navigation. See [MODE3_RUN.md](MODE3_RUN.md).

Artifacts: [scored windows](validation/mode3_20261111/m3_large_trial_b_windows.json), [physics contact coverage](validation/mode3_20261111/m3_large_trial_b_contact_summary.json), [exact-stamp localization](validation/mode3_20261111/m3_large_trial_b_scan_summary.json), [stop result](validation/mode3_20261111/mode3_stop_result.json).
