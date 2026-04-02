#!/usr/bin/env python3
"""
pid_tune_auto.py — Parse PID auto-tuning SWO output from pid_tune_diag firmware.

Usage:
    python3 scripts/pid_tune_auto.py [swo_text_file]

Default input: build/swo/swo.txt (output of parse_swo.py)

The firmware runs a staged 4-phase sweep autonomously:
  Phase 1: KP sweep
  Phase 2: KI sweep (with best KP)
  Phase 3: KD sweep (with best KP, KI)
  Phase 4: MIN_SPEED sweep (with best KP, KI, KD)
  Final:   Verification with best params (10 reps)
"""

import sys
import re
from dataclasses import dataclass


@dataclass
class TestEntry:
    kp: int
    ki: int
    kd: int
    min_spd: int
    fwd_mean: int
    fwd_max: int
    rev_mean: int
    rev_max: int
    score: int


def parse_swo(filepath: str):
    """Parse SWO text output from pid_tune_diag."""
    results: list[TestEntry] = []
    bests: dict[int, dict] = {}
    final = None
    final_detail = None

    with open(filepath) as f:
        lines = f.readlines()

    for line in lines:
        line = line.strip()

        # [RESULT] kp=50 ki=0 kd=10 ms=200 fwd_m=2 fwd_x=3 rev_m=-2 rev_x=3 score=8
        m = re.match(
            r'\[RESULT\] kp=(-?\d+) ki=(-?\d+) kd=(-?\d+) ms=(-?\d+) '
            r'fwd_m=(-?\d+) fwd_x=(-?\d+) rev_m=(-?\d+) rev_x=(-?\d+) score=(-?\d+)',
            line
        )
        if m:
            entry = TestEntry(
                kp=int(m.group(1)), ki=int(m.group(2)),
                kd=int(m.group(3)), min_spd=int(m.group(4)),
                fwd_mean=int(m.group(5)), fwd_max=int(m.group(6)),
                rev_mean=int(m.group(7)), rev_max=int(m.group(8)),
                score=int(m.group(9)),
            )
            results.append(entry)
            continue

        # [BEST] PHASE=N param=val score=val
        m = re.match(r'\[BEST\] PHASE=(\d+) (\w+)=(-?\d+) score=(-?\d+)', line)
        if m:
            phase = int(m.group(1))
            bests[phase] = {
                'param': m.group(2),
                'value': int(m.group(3)),
                'score': int(m.group(4)),
            }
            continue

        # [FINAL] KP=50 KI=3 KD=10 MIN_SPD=200
        m = re.match(r'\[FINAL\] KP=(-?\d+) KI=(-?\d+) KD=(-?\d+) MIN_SPD=(-?\d+)', line)
        if m:
            final = {
                'kp': int(m.group(1)),
                'ki': int(m.group(2)),
                'kd': int(m.group(3)),
                'min_spd': int(m.group(4)),
            }
            continue

        # [FINAL] FWD: mean=... max=...  REV: mean=... max=...  score=...
        m = re.match(
            r'\[FINAL\] FWD: mean=(-?\d+) max=(-?\d+)\s+REV: mean=(-?\d+) max=(-?\d+)\s+score=(-?\d+)',
            line
        )
        if m:
            final_detail = {
                'fwd_mean': int(m.group(1)), 'fwd_max': int(m.group(2)),
                'rev_mean': int(m.group(3)), 'rev_max': int(m.group(4)),
                'score': int(m.group(5)),
            }
            continue

    return results, bests, final, final_detail


def main():
    filepath = sys.argv[1] if len(sys.argv) > 1 else 'build/swo/swo.txt'

    print(f"Parsing: {filepath}")
    results, bests, final, final_detail = parse_swo(filepath)

    if not results:
        print("No [RESULT] entries found. Check SWO capture.")
        return 1

    # Phase summaries
    print("\n" + "=" * 70)
    print("PHASE RESULTS")
    print("=" * 70)
    for phase_num in sorted(bests.keys()):
        b = bests[phase_num]
        print(f"  Phase {phase_num}: best {b['param']}={b['value']}  score={b['score']}")

    # All results sorted by score
    print("\n" + "=" * 70)
    print(f"ALL RESULTS ({len(results)} tests, sorted by score)")
    print("=" * 70)
    print(f"  {'KP':>5} {'KI':>5} {'KD':>5} {'MS':>5} | "
          f"{'FWD_m':>6} {'FWD_x':>6} {'REV_m':>6} {'REV_x':>6} | {'Score':>6}")
    print(f"  {'-'*5} {'-'*5} {'-'*5} {'-'*5} | "
          f"{'-'*6} {'-'*6} {'-'*6} {'-'*6} | {'-'*6}")

    results.sort(key=lambda x: x.score)
    for i, r in enumerate(results):
        marker = " <<<" if i == 0 else ""
        print(f"  {r.kp/100:5.2f} {r.ki/100:5.2f} {r.kd/100:5.2f} {r.min_spd:5d} | "
              f"{r.fwd_mean:+6d} {r.fwd_max:6d} {r.rev_mean:+6d} {r.rev_max:6d} | "
              f"{r.score:6d}{marker}")

    # Final verification
    if final and final_detail:
        print("\n" + "=" * 70)
        print("OPTIMAL PARAMETERS (final verification)")
        print("=" * 70)
        print(f"  KP        = {final['kp']/100:.2f}")
        print(f"  KI        = {final['ki']/100:.2f}")
        print(f"  KD        = {final['kd']/100:.2f}")
        print(f"  MIN_SPEED = {final['min_spd']}")
        print(f"  ---")
        print(f"  FWD: mean={final_detail['fwd_mean']:+d}  max={final_detail['fwd_max']}")
        print(f"  REV: mean={final_detail['rev_mean']:+d}  max={final_detail['rev_max']}")
        print(f"  Score: {final_detail['score']}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
