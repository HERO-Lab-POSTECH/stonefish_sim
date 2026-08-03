#!/usr/bin/env python3
"""ALBC Stonefish smoke check -- narrowed criteria only (2026-08-03 role decision).

PASS/FAIL gates: seam integrity (topics flowing), finiteness (no NaN/Inf),
divergence (bounded rates / position drift), latent health, sim/bridge warnings.
The rms figures printed at the end are QUALITATIVE context only -- this bench
issues no performance verdict, no Isaac ratio, no hydro judgement.

Usage: smoke_check.py <bag_dir> <launch_log>     (exit 0 = PASS, 1 = FAIL)
Adapted from the 2026-07-30 hold harness analyze_hold.py.
"""
import sys

import numpy as np

sys.path.insert(0, "/workspace/install/lib/python3.10/site-packages")
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from albc_bridge.frames import stonefish_odom_to_isaac  # policy's own frame SSOT

# Thresholds: runaway signature is O(100-1000) rad/s and O(1000) m/s
# (2026-07-30 diagnostics); healthy holds measured 0.4-2.2 rad/s rms (n=4).
# Position is NOT gated -- attitude hold leaves translation uncontrolled, so
# free drift (several m/min) is expected physics; velocity is the discriminator.
W_RMS_MAX = 5.0      # rad/s, per-axis rms
V_RMS_MAX = 5.0      # m/s, per-axis rms
LAT_ABS_MAX = 10.0   # healthy |z|max measured ~0.74
MIN_MSGS = 100       # seam gate: each stream must actually flow

rms = lambda x: float(np.sqrt(np.mean(np.square(x))))


def read_bag(uri):
    r = SequentialReader()
    r.open(StorageOptions(uri=uri, storage_id="sqlite3"), ConverterOptions("", ""))
    buf = {}
    while r.has_next():
        topic, data, _ = r.read_next()
        buf.setdefault(topic, []).append(data)
    return buf


def main():
    bag, log_path = sys.argv[1], sys.argv[2]
    buf = read_bag(bag)

    eul, av, pos, vel = [], [], [], []
    for d in buf.get("/albc/odometry", []):
        m = deserialize_message(d, Odometry)
        q, w, p = m.pose.pose.orientation, m.twist.twist.angular, m.pose.pose.position
        v = m.twist.twist.linear
        e, a = stonefish_odom_to_isaac(np.array([q.x, q.y, q.z, q.w]),
                                       np.array([w.x, w.y, w.z]))
        eul.append(e); av.append(a); pos.append([p.x, p.y, p.z]); vel.append([v.x, v.y, v.z])
    eul, av, pos, vel = np.array(eul), np.array(av), np.array(pos), np.array(vel)

    def arr(topic):
        return np.array([deserialize_message(d, Float64MultiArray).data
                         for d in buf.get(topic, [])])
    act, lat = arr("/albc/debug/action"), arr("/albc/debug/latent")

    log = open(log_path, errors="replace").read()
    gates = []

    def gate(name, ok, detail):
        gates.append(ok)
        print("  %-28s %s  %s" % (name, "PASS" if ok else "FAIL", detail))

    print("== smoke gates (narrowed criteria, role decision 2026-08-03) ==")
    gate("seam: streams flowing",
         len(eul) >= MIN_MSGS and len(act) >= MIN_MSGS and len(lat) >= MIN_MSGS,
         "odom=%d act=%d lat=%d (min %d)" % (len(eul), len(act), len(lat), MIN_MSGS))
    if not gates[0]:  # nothing downstream is meaningful without data
        print("SMOKE: FAIL")
        return 1
    gate("finiteness: no NaN/Inf",
         all(np.isfinite(x).all() for x in (eul, av, pos, vel, act, lat)), "odom/act/latent")
    wrms = [rms(av[:, i]) for i in range(3)]
    gate("divergence: angular rate",
         max(wrms) < W_RMS_MAX,
         "rms w=(%.3f, %.3f, %.3f) rad/s (max %.1f)" % (*wrms, W_RMS_MAX))
    vrms = [rms(vel[:, i]) for i in range(3)]
    gate("divergence: linear velocity",
         max(vrms) < V_RMS_MAX,
         "rms v=(%.3f, %.3f, %.3f) m/s (max %.1f)" % (*vrms, V_RMS_MAX))
    dead = int((lat.std(axis=0) < 1e-6).sum())
    gate("latent health", dead == 0 and float(np.abs(lat).max()) < LAT_ABS_MAX,
         "dead dims=%d |z|max=%.3f" % (dead, float(np.abs(lat).max())))
    warn = log.lower().count("non-finite") + log.lower().count("skipping")
    died = log.count("process has died")
    gate("sim/bridge warnings", warn == 0 and died == 0,
         "non-finite/skipping=%d died=%d" % (warn, died))

    print("\n== qualitative context (rms only -- NOT a performance verdict) ==")
    print("  roll %.4f  pitch %.4f rad | wx %.4f  wy %.4f  wz %.4f rad/s"
          % (rms(eul[:, 0]), rms(eul[:, 1]), *wrms))
    print("  free drift %.2f m (uncontrolled DOF, not gated)"
          % float(np.linalg.norm(pos - pos[0], axis=1).max()))
    thr = act[:, 2:8]
    print("  thr rms %.4f | action sat %.1f%%"
          % (rms(thr), 100 * float(np.mean(np.abs(thr) >= 1.0 - 1e-9))))

    ok = all(gates)
    print("\nSMOKE: %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
