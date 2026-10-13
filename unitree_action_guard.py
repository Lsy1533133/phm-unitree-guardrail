"""unitree_action_guard.py — deterministic pre-execution physics guard for Unitree LowCmd streams.

OPEN scope (this file): spec-layer checks against OFFICIAL Unitree URDF limits
(joint position / joint velocity / effort / inter-frame velocity). The production
rule engine (FK collision / singularity / causal-cone layers, validated on a
1e9-frame synthetic campaign) is CLOSED SOURCE — inject it via the duck-typed
`verifier=` hook (interface in verifier_interface.pyi).

Frame shape mirrors unitree_sdk2_python LowCmd semantics (field names q / dq /
tau), one dict per control frame. This is a spec-level check: it does NOT talk
DDS and does NOT replace Unitree's onboard protections — it is a pre-publish
audit layer between your controller/RL policy and the SDK send call.
"""
from __future__ import annotations

import hashlib
import json
import time
from typing import Any, Dict, List, Optional


class UnitreeActionGuard:
    def __init__(self, profile: Dict[str, Any], verifier: Optional[Any] = None):
        """profile: dict from unitree_profiles_parsed.json (machine/joints[n]).
        verifier: optional duck-typed closed-engine hook exposing
                  verify(frames, dt) -> {"verdict": ..., "blocked_by": [...], ...}.
        """
        self.profile = profile
        self.joints = profile["joints"]
        self.verifier = verifier

    # ---------- per-frame spec layers ----------
    def check_frame(self, frame: Dict[str, Any]) -> Dict[str, Any]:
        q, dq, tau = frame.get("q", []), frame.get("dq", []), frame.get("tau", [])
        if len(q) != len(self.joints):
            return {"verdict": "INTERCEPT", "rule": "R-dim",
                    "evidence": f"frame has {len(q)} joints, profile expects {len(self.joints)}"}
        for i, js in enumerate(self.joints):
            if q[i] < js["q_lower"] or q[i] > js["q_upper"]:
                return {"verdict": "INTERCEPT", "rule": "R-joint-limit",
                        "evidence": f"{js['name']} q={q[i]:.3f} outside [{js['q_lower']:.3f},{js['q_upper']:.3f}] rad"}
            if abs(dq[i]) > js["dq_max"]:
                return {"verdict": "INTERCEPT", "rule": "R-velocity",
                        "evidence": f"{js['name']} |dq|={abs(dq[i]):.2f} > {js['dq_max']} rad/s"}
            if abs(tau[i]) > js["tau_max"]:
                return {"verdict": "INTERCEPT", "rule": "R-effort",
                        "evidence": f"{js['name']} |tau|={abs(tau[i]):.2f} > {js['tau_max']} N·m"}
        return {"verdict": "PASS", "rule": None, "evidence": None}

    # ---------- stream layer (adds inter-frame velocity) ----------
    def check_stream(self, frames: List[Dict[str, Any]], dt: float = 0.002) -> Dict[str, Any]:
        t0 = time.perf_counter()
        blocked: List[Dict[str, Any]] = []
        latencies: List[float] = []
        for i, fr in enumerate(frames):
            t1 = time.perf_counter()
            r = self.check_frame(fr)
            if r["verdict"] == "PASS" and i > 0:
                for j, js in enumerate(self.joints):
                    rate = abs(fr["q"][j] - frames[i - 1]["q"][j]) / dt
                    if rate > js["dq_max"] * 1.5:  # 50% tolerance: inter-frame estimate is noisier than commanded dq
                        r = {"verdict": "INTERCEPT", "rule": "R-rate",
                             "evidence": f"{js['name']} step velocity {rate:.1f} rad/s > 1.5x limit {js['dq_max']}"}
                        break
            latencies.append(time.perf_counter() - t1)
            if r["verdict"] != "PASS":
                blocked.append({"frame": i, **r})

        closed_note = None
        if self.verifier is not None:
            try:
                cr = self.verifier.verify(frames, dt)
                closed_note = cr.get("verdict")
            except Exception as e:  # noqa: BLE001
                closed_note = f"verifier-hook-error: {type(e).__name__}"

        lat = sorted(latencies)
        p50 = lat[len(lat) // 2] * 1e6
        p95 = lat[int(len(lat) * 0.95)] * 1e6
        return {
            "machine": self.profile["machine"],
            "n_frames": len(frames),
            "stream_verdict": "BLOCK" if blocked else "PASS",
            "n_blocked": len(blocked),
            "blocked_first3": blocked[:3],
            "guard_latency_us_p50": round(p50, 2),
            "guard_latency_us_p95": round(p95, 2),
            "closed_engine_verdict": closed_note,
        }


def sha256_file(path: str) -> str:
    return hashlib.sha256(open(path, "rb").read()).hexdigest()
