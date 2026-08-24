"""Repeatedly trace a circle in the base-frame y-z plane with POSE TARGETS, awaiting
each point -- so every step is a verified on-robot IK solve and every refusal is visible.

Awaited, not streamed: fire-and-forget poses carry no action_id, so refusals are silent.
Measured on noriA3-0 -- streaming 72 pts/lap at 11Hz left the arm 21cm off the path with
nothing reported, while the awaited lap solved 36/36.
"""
import asyncio
import math
import os
import sys

from nori_sdk import RemoteTeleop, SupabaseSignaling

START = [float(v) for v in sys.argv[1].split(",")]
LAPS  = int(sys.argv[2]) if len(sys.argv) > 2 else 3
STEPS = int(sys.argv[3]) if len(sys.argv) > 3 else 36
X, CY, CZ, R = 0.0, -0.45, 0.92, 0.05

def point(k, n=None):
    th = 2.0 * math.pi * k / (n or STEPS)
    return [X, CY + R * math.cos(th), CZ + R * math.sin(th)]

async def main():
    sig = SupabaseSignaling(os.environ["NORI_SUPABASE_URL"],
                            os.environ.get("NORI_SUPABASE_ANON_KEY", ""),
                            room="NORI-A3-0000", token_provider=None)
    async with RemoteTeleop(sig, on_log=lambda *a: None) as robot:
        await robot.wait_ready()
        robot.set_jog({})
        await asyncio.sleep(0.8)

        entry, legs = point(0), 20
        for i in range(1, legs + 1):
            p = [START[j] + (entry[j] - START[j]) * i / legs for j in range(3)]
            await robot.pose("right", p, wait=True, timeout=12.0)
        print(f"at entry {[round(v,3) for v in entry]}")
        print(f"circle: centre y={CY} z={CZ}, r={R*100:.0f}cm, {STEPS} pts/lap, "
              f"chord {2*R*math.sin(math.pi/STEPS)*100:.1f}cm")

        for lap in range(1, LAPS + 1):
            marks, fails = [], {}
            for k in range(STEPS):
                st = await robot.pose("right", point(k), wait=True, timeout=12.0)
                ok = st.state == "done"
                marks.append("." if ok else "x")
                if not ok:
                    fails.setdefault(st.reason or st.state, []).append(k * 360 // STEPS)
            detail = "  ".join(f"{r}:{d}" for r, d in fails.items())
            print(f"  lap {lap}: {''.join(marks)}  {marks.count('.')}/{STEPS}"
                  + (f"   {detail}" if detail else ""))
        await robot.stop_jog()
        print("demo complete")

asyncio.run(main())
