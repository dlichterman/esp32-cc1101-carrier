import trimesh, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import make_case as mc
base = trimesh.load("case_base.stl"); lid = trimesh.load("case_lid.stl")

def section_xy(mesh, z, ax, color, lw=1.0):
    s = mesh.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    if s is None: return
    for e in s.entities:
        p = s.vertices[e.points]; ax.plot(p[:, 0], p[:, 1], color=color, lw=lw)
def section_xz(mesh, y, ax, color, lw=1.0):
    s = mesh.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0])
    if s is None: return
    for e in s.entities:
        p = s.vertices[e.points]; ax.plot(p[:, 0], p[:, 2], color=color, lw=lw)
def env_box(e):
    b = e.val().BoundingBox(); return b
fig = plt.figure(figsize=(16, 12))
# plan, z = 20 (through the tall parts)
ax = fig.add_subplot(2, 2, 1); ax.set_aspect("equal"); ax.set_title("Plan section at z = 20 mm (case walls, standoffs/pillars, part envelopes)")
section_xy(base, 20, ax, "#1f5fa8", 1.4); section_xy(lid, 14, ax, "#b05a00", 1.0)
ax.plot([0,95,95,0,0],[0,0,55,55,0],"g-",lw=1.2,label="PCB outline")
cols = {"ESP32":"#c0392b","CC1101":"#8e44ad","C1":"#d68910","C2":"#7f8c8d","C3":"#d68910","C4":"#7f8c8d","SMA":"#16a085","Antenna":"#16a085"}
for n, e in mc.env.items():
    b = env_box(e)
    if b.zmax < 8.7 or b.zmin > 30: continue
    c = next((v for k, v in cols.items() if n.startswith(k)), "#555")
    ax.add_patch(plt.Rectangle((b.xmin, b.ymin), b.xmax - b.xmin, b.ymax - b.ymin, fill=True, alpha=0.25, color=c))
    ax.text((b.xmin + b.xmax) / 2, (b.ymin + b.ymax) / 2, n.split(" ")[0] + (" " + n.split(" ")[1] if n.startswith(("ESP32","CC1101","C1","C3")) and len(n.split(" "))>1 else ""), fontsize=6, ha="center")
ax.set_xlim(-6, 102); ax.set_ylim(-6, 64); ax.invert_yaxis() if False else None
# side section through the USB / SMA (y = SMA centre)
ax = fig.add_subplot(2, 2, 2); ax.set_aspect("equal"); ax.set_title("Side section along X through the CC1101 / SMA centre line")
section_xz(base, mc.cc_yc, ax, "#1f5fa8", 1.4); section_xz(lid, mc.cc_yc, ax, "#b05a00", 1.4)
for n, e in mc.env.items():
    b = env_box(e)
    if b.ymin <= mc.cc_yc <= b.ymax:
        ax.add_patch(plt.Rectangle((b.xmin, b.zmin), b.xmax - b.xmin, b.zmax - b.zmin, alpha=0.25, color="#8e44ad" if "CC1101" in n else "#16a085" if "SMA" in n or "Antenna" in n else "#555"))
ax.add_patch(plt.Rectangle((0, 7.0), 95, 1.6, color="green", alpha=0.6)); ax.set_xlim(-6, 106); ax.set_ylim(-1, 32)
# front section through the USB (x = 30)
ax = fig.add_subplot(2, 2, 3); ax.set_aspect("equal"); ax.set_title("Section along Y through the ESP32 / USB centre line (x = 30)")
def section_yz(mesh, x, ax, color):
    s = mesh.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
    if s is None: return
    for e in s.entities:
        p = s.vertices[e.points]; ax.plot(p[:, 1], p[:, 2], color=color, lw=1.4)
section_yz(base, 30, ax, "#1f5fa8"); section_yz(lid, 30, ax, "#b05a00")
for n, e in mc.env.items():
    b = env_box(e)
    if b.xmin <= 30 <= b.xmax and "ESP32" in n:
        ax.add_patch(plt.Rectangle((b.ymin, b.zmin), b.ymax - b.ymin, b.zmax - b.zmin, alpha=0.25, color="#c0392b"))
ax.add_patch(plt.Rectangle((0, 7.0), 55, 1.6, color="green", alpha=0.6)); ax.set_xlim(-6, 64); ax.set_ylim(-1, 32)
# iso view
def iso(mesh, ax, color, alpha=1.0):
    tri = mesh.vertices[mesh.faces]
    n = mesh.face_normals; light = np.array([0.4, -0.5, 0.75]); light /= np.linalg.norm(light)
    sh = np.clip(n @ light, 0.15, 1)[:, None]
    c = np.array(matplotlib.colors.to_rgb(color))[None, :] * sh
    pc = Poly3DCollection(tri, facecolors=np.c_[c, np.full(len(c), alpha)], edgecolor="none"); ax.add_collection3d(pc)
ax = fig.add_subplot(2, 2, 4, projection="3d"); ax.set_title("Base (open) and lid (lifted)")
iso(base, ax, "#4a90d9")
lid2 = lid.copy(); lid2.apply_translation([0, 0, 22]); iso(lid2, ax, "#dddddd", 0.9)
ax.set_xlim(-5, 105); ax.set_ylim(-20, 85); ax.set_zlim(0, 60); ax.view_init(28, -62); ax.set_box_aspect((110, 105, 60))
ax.axis("off")
plt.tight_layout(); plt.savefig("case_preview.png", dpi=110)
print("ok")
