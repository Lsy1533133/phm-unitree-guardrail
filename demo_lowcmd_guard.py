"""demo_lowcmd_guard.py — GATE TEST: spec-based LowCmd stream through the guard.

Honest scope: frames are SYNTHETIC, generated within official URDF limits
(spec-accurate simulator approach, same caliber as our M30T work). This is
NOT real robot field data. The closed production engine is not included;
the closed-engine hook is shown but only exercised if you plug a verifier.

Stream: 2000 benign frames (sinusoidal inside limits, zero violations expected)
        + 6 injected violation frames (2x joint-limit, 2x velocity, 2x effort)
Gate criteria: benign FP = 0 AND injected FN = 0.
Run: python3 demo_lowcmd_guard.py   (or bash run_demo.sh)
"""
from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path

from unitree_action_guard import UnitreeActionGuard, sha256_file

HERE = Path(__file__).parent
PROFILE = json.load(open(HERE / "unitree_profiles_parsed.json"))["z1"]
N_JOINTS = PROFILE["n_joints"]
N_BENIGN = 2000
DT = 0.002  # 500 Hz control loop

INJECT = {100: "q_over", 350: "q_over", 700: "dq_spike", 900: "dq_spike",
          1200: "tau_spike", 1400: "tau_spike"}


def benign_frame(t: float) -> dict:
    """Sinusoidal inside limits: amplitude 60% of each joint's safe span."""
    q, dq, tau = [], [], []
    for k, js in enumerate(PROFILE["joints"]):
        span = (js["q_upper"] - js["q_lower"]) * 0.3
        mid = (js["q_upper"] + js["q_lower"]) / 2
        w = 0.5 + 0.1 * k
        qv = mid + span * math.sin(w * t + k)
        dv = span * w * math.cos(w * t + k)
        q.append(qv); dq.append(dv); tau.append(0.05 * abs(dv) + 0.1)
    return {"q": q, "dq": dq, "tau": tau}


def inject(i: int, fr: dict) -> dict:
    kind = INJECT[i]
    j = PROFILE["joints"][1]  # z1_joint2: effort 60, span [0, 2.967]
    fr = {k: list(v) for k, v in fr.items()}
    if kind == "q_over":
        fr["q"][1] = j["q_upper"] + 0.5                      # 0.5 rad over hard stop
    elif kind == "dq_spike":
        fr["dq"][1] = j["dq_max"] * 3.0                      # 3x velocity limit
    elif kind == "tau_spike":
        fr["tau"][1] = j["tau_max"] * 2.0                    # 2x effort limit
    return fr


def main() -> dict:
    guard = UnitreeActionGuard(PROFILE)
    frames, expected = [], {}
    t = 0.0
    for i in range(N_BENIGN):
        fr = benign_frame(t); t += DT
        if i in INJECT:
            fr = inject(i, fr)
            expected[i] = INJECT[i]
        frames.append(fr)

    t0 = time.perf_counter()
    result = guard.check_stream(frames, dt=DT)
    wall = (time.perf_counter() - t0) * 1000

    blocked_at = {b["frame"]: b["rule"] for b in result["blocked_first3"]}
    # full blocked list needed for FN check — rerun classification per frame
    caught = set()
    for i, fr in enumerate(frames):
        if i in INJECT:
            r = guard.check_frame(fr)
            if r["verdict"] == "INTERCEPT":
                caught.add(i)
    fn = sorted(set(expected) - caught)
    fp = 0  # benign frames: every non-injected frame must PASS; count via stream diff
    benign_frames = [fr for i, fr in enumerate(frames) if i not in INJECT]
    fp = sum(1 for fr in benign_frames if guard.check_frame(fr)["verdict"] != "PASS")

    out = {
        "test": "Unitree Z1 spec-based LowCmd stream through PHM spec-layer guard (synthetic, spec-accurate)",
        "date": time.strftime("%F %T"),
        "profile": {"machine": PROFILE["machine"], "n_joints": PROFILE["n_joints"],
                     "source_url": PROFILE["source_url"]},
        "n_frames": len(frames), "n_benign": len(benign_frames), "n_injected": len(expected),
        "false_positives_on_benign": fp,
        "false_negatives_on_injected": fn,
        "caught_injected": len(caught),
        "stream_verdict": result["stream_verdict"],
        "guard_latency_us_p50": result["guard_latency_us_p50"],
        "guard_latency_us_p95": result["guard_latency_us_p95"],
        "wall_ms_total": round(wall, 1),
        "gate_criteria": "FP=0 and FN=0",
        "gate_verdict": "PASS" if (fp == 0 and not fn) else "REVIEW",
        "honesty_notes": [
            "synthetic frames within official URDF limits; no real robot data",
            "spec-layer rules only; FK collision / singularity layers are closed-source",
        ],
    }
    out_path = HERE / "results" / "unitree_lowcmd_guard_results.json"
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=1))
    # integrity: standalone sidecar file (git-style). We deliberately do NOT
    # embed the hash inside the JSON itself — a self-referential hash is
    # unverifiable (file hash changes after the hash field is written).
    chain = hashlib.sha256(out_path.read_bytes()).hexdigest()
    (HERE / "results" / "unitree_lowcmd_guard_results.sha256").write_text(
        chain + "  unitree_lowcmd_guard_results.json\n")

    print(f"=== Unitree Z1 LowCmd gate test (synthetic, spec-accurate) ===")
    print(f"frames={len(frames)} benign={len(benign_frames)} injected={len(expected)}")
    print(f"FP={fp} FN={len(fn)} caught={len(caught)}")
    print(f"guard p50={result['guard_latency_us_p50']}us p95={result['guard_latency_us_p95']}us")
    print(f"GATE: {out['gate_verdict']} | artifact sha256[:16]={chain[:16]}")
    return out


if __name__ == "__main__":
    main()
