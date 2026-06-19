#!/usr/bin/env python3
import struct
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
mjb = sys.argv[1] if len(sys.argv) > 1 else str(ROOT / "assets/m2_metal.xml")
out = sys.argv[2] if len(sys.argv) > 2 else str(ROOT / "web/m2_meshes.bin")
m = mujoco.MjModel.from_xml_path(mjb)

buf = bytearray()
buf += struct.pack("<ii", 0x47314D53, m.nmesh) # use same magic 'G1MS'
for i in range(m.nmesh):
    buf += struct.pack("<ii", int(m.mesh_vertnum[i]), int(m.mesh_facenum[i]))
for i in range(m.nmesh):
    a, n = int(m.mesh_vertadr[i]), int(m.mesh_vertnum[i])
    buf += m.mesh_vert[a:a+n].astype("<f4").tobytes()
for i in range(m.nmesh):
    a, n = int(m.mesh_faceadr[i]), int(m.mesh_facenum[i])
    buf += m.mesh_face[a:a+n].astype("<i4").tobytes()

Path(out).parent.mkdir(parents=True, exist_ok=True)
Path(out).write_bytes(buf)
print(f"wrote {out}: nmesh={m.nmesh} bytes={len(buf)}")
