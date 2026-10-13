# PHM Unitree Guardrail

Deterministic pre-execution physics guard for Unitree robot command streams
(`LowCmd`-shape: per-joint q / dq / tau), checked against **official Unitree
URDF limits** (Z1 arm 6-DOF, Go2 12 leg joints, G1 29-DOF — parsed from
`unitreerobotics/unitree_ros`, provenance embedded). Gate-test artifact
included: 2000-frame stream (fully deterministic, no RNG; violation frames
fixed at 100/350/700/900/1200/1400). Single run observed **no false positives
on 1994 benign frames and no false negatives on 6 injected violations**
(rule-of-three upper bound on FP rate ≈ 0.15% one-sided 95%; sample too small
for statistical guarantees). Guard overhead **p50 2.39 µs / frame** (2026-10-13,
`results/`).

```
your controller / RL policy ──> [PHM spec-layer guard] ──> unitree_sdk2 send (DDS)
                                   │  q-limit / |dq| / |tau| / inter-frame rate
                                   └─ duck-typed hook for the closed FK engine
                                      (collision / singularity / causal-cone)
```

## What this repo is

- `unitree_profiles_parsed.json` — machine profiles parsed from official
  Unitree URDFs (numbers copied from URDF attributes without manual
  modification; not independently re-verified against datasheets —
  regenerate per DATA_SOURCES.md).
- `unitree_action_guard.py` — open spec-layer guard: per-frame joint-limit,
  commanded-velocity and effort checks, plus inter-frame rate check;
  duck-typed `verifier=` hook for a closed production engine.
- `demo_lowcmd_guard.py` — gate test on a 2000-frame spec-accurate synthetic
  `LowCmd`-shape stream with 6 injected violations.

## Gate-test result (2026-10-13)

| Metric | Value |
|---|---|
| Frames | 2000 (1994 benign + 6 injected) |
| False positives on benign | **0** |
| False negatives on injected | **0** (6/6 caught) |
| Guard overhead | p50 2.39 µs, p95 2.65 µs per frame (single run; cloud x86 Linux, Python 3.12, pure in-memory compute no I/O; not extrapolable to other profiles or hardware) |
| Profile | Unitree Z1, 6-DOF (official URDF limits) — demonstrated in this synthetic demo only |

n=2000 single run — a mechanism demonstration, not a statistical reliability
claim.

## Honest boundaries

1. **Synthetic evidence.** Demo frames are generated within official URDF
   limits (frames built from the parsed limits themselves; no claim of
   firmware/fidelity equivalence to real Unitree behavior). This is not real
   robot field data. FP/FN here are in-distribution synthetic metrics, not
   field metrics.
2. **Spec-layer only (open).** FK self-collision, singularity and causal-cone
   layers are closed-source (`CLOSED_SOURCE_LIST.md`). The closed production
   engine has a separate large-scale synthetic campaign — treat any numbers
   from it as vendor-claimed and not independently auditable; they are not
   evidence for this repo's claims.
3. **Profile coverage.** Z1 6-DOF spec check is demonstrated in this
   synthetic demo only. Go2 (12j) and G1 (29j) profiles are parsed and
   shipped but have NO stream demo yet — treat them as unvalidated data
   files. The interface is shape-based (per-joint bounds), not
   morphology-specific; legged-machine demos are the next step.
4. **Not a replacement for onboard safety.** This is an audit/pre-publish
   layer between your policy and the SDK call; it does not talk DDS and does
   not replace Unitree's built-in protections.
5. **Verifier swap-in** verified only for the bundled spec-layer; no
   third-party verifier swap tested yet.

## Run

```bash
bash run_demo.sh     # or: python3 demo_lowcmd_guard.py
```

Expected: `GATE: PASS | artifact sha256[:16]=...` (artifact in `results/`).

## Relation to other PHM artifacts

- `agent-action-verifier` — agent-action line (AgentDojo ASR=0/FP=0,
  openai-agents issues #5330/#5351 verification dataset).
- `phm-drone-audit-cloud` — DJI Cloud API flight-data audit line.
- `phm-guardrail` — NVIDIA NeMo Guardrails flavor.

## License

MIT on the open files. Closed components are not included and not licensed.
