"""Parametric 3D-printable case for the ESP32 + CC1101 carrier (95 x 55 mm board).
Model coordinates: origin = board bottom-left corner, X right, Y up (USB end of the ESP32 at Y = 0), Z up from the case floor.
KiCad coordinates (origin top-left, Y down) are converted with ky().  Run:  python3 make_case.py
"""
import math, json
import cadquery as cq

# ----------------------------------------------------------------- parameters (mm)
BOARD_W, BOARD_H, BOARD_T = 95.0, 55.0, 1.6
HOLES_KICAD = [(4, 4), (91, 4), (4, 51), (91, 51)]       # M3 holes
CLEAR = 1.0            # gap between board edge and inside wall
WALL = 2.4
FLOOR = 2.0
STANDOFF_H = 5.0       # board underside to floor: room for solder tails / pin ends
STANDOFF_OD = 8.0
INSERT_HOLE_D = 4.2    # M3 heat-set insert (about 4.6 mm OD, 5.7 mm long): melt into a 4.2 mm hole
INSERT_DEPTH = 6.0
SOCKET_H = 8.5         # female header height under both modules
HDR_SPACER = 2.5       # black plastic spacer of the male header under each module: it rests on top of the female socket (seen in the CC1101 photos; assumed the same for the ESP32 DevKit)
H_ABOVE = 18.0         # inside height from board top to lid underside
LID_T = 3.0
LIP_T, LIP_H, LIP_GAP = 1.2, 3.0, 0.25
PILLAR_OD, BORE_D, CBORE_D, CBORE_DEPTH = 7.0, 3.4, 6.4, 2.0
CORNER_R = 3.0
# ESP32 DevKit (USB end at the front wall)
ESP_X = 30.0
USB_W, USB_BELOW = 15.0, 4.2                  # notch for the USB-C plug in the front wall: open to the lid, bottom is USB_BELOW under the receptacle centre
# CC1101 breakout, flat on its 2x4 socket, extending to the right, SMA out of the far short edge
CC_PCB = (28.0, 15.0)                          # length (X) x width (Y)
CC_X0 = 60.5                                   # module left edge: header column centres are ~1.6 and ~4.1 mm in from it (from the photos)
CC_T = 1.0                                     # CC1101 breakout PCB thickness (from the photos)
CC_PCB_Y_OFF = 1.0                             # PCB centre sits ~1 mm toward pins 1/2 (model +Y) from the header and SMA centre line (from the photos)
CC_Y_KICAD = 26.81                             # centre of the 2x4 header (KiCad y)
SMA_LEN, SMA_ABOVE_PCB, SMA_OD, SMA_FLANGE = 9.5, 0.5, 6.4, 7.5   # edge-launch SMA: barrel sticks out SMA_LEN past the PCB edge, axis 0.5 mm above the PCB top, 7.5 mm square flange
SMA_SLOT_W = 13.0                              # slot for the antenna nut and connector, open at the top

def ky(y): return BOARD_H - y
Z_BOARD_TOP = FLOOR + STANDOFF_H + BOARD_T
Z_TOP = Z_BOARD_TOP + H_ABOVE
X0, X1 = -CLEAR, BOARD_W + CLEAR
Y0, Y1 = -CLEAR, BOARD_H + CLEAR
ox0, ox1, oy0, oy1 = X0 - WALL, X1 + WALL, Y0 - WALL, Y1 + WALL
cc_yc = ky(CC_Y_KICAD)
mod_pcb_z0 = Z_BOARD_TOP + SOCKET_H + HDR_SPACER
mod_pcb_z1 = mod_pcb_z0 + 1.6          # ESP32 DevKit PCB top
cc_z1 = mod_pcb_z0 + CC_T              # CC1101 breakout PCB top
sma_zc = cc_z1 + SMA_ABOVE_PCB
usb_zc = mod_pcb_z1 + 1.65             # USB-C receptacle centre
USB_Z0 = usb_zc - USB_BELOW
HOLES = [(x, ky(y)) for x, y in HOLES_KICAD]

def box(x0, y0, z0, x1, y1, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl(x, y, z0, z1, d):
    return cq.Workplane("XY").workplane(offset=z0).center(x, y).circle(d / 2).extrude(z1 - z0)
def rrect(xa, ya, xb, yb, r, z0, z1):
    return (cq.Workplane("XY").workplane(offset=z0).center((xa + xb) / 2, (ya + yb) / 2)
            .rect(xb - xa, yb - ya).extrude(z1 - z0).edges("|Z").fillet(r))

# ----------------------------------------------------------------- base tray
base = rrect(ox0, oy0, ox1, oy1, CORNER_R, 0, Z_TOP)
base = base.cut(rrect(X0, Y0, X1, Y1, 1.0, FLOOR, Z_TOP + 1))
for (x, y) in HOLES:
    base = base.union(cyl(x, y, FLOOR - 0.5, FLOOR + STANDOFF_H, STANDOFF_OD))
for (x, y) in HOLES:
    base = base.cut(cyl(x, y, FLOOR + STANDOFF_H - INSERT_DEPTH, FLOOR + STANDOFF_H + 1, INSERT_HOLE_D))
# USB-C plug notch in the front wall, open at the top (the lid closes it)
usb = box(ESP_X - USB_W / 2, oy0 - 1, USB_Z0, ESP_X + USB_W / 2, Y0 + 0.5, Z_TOP + 1).edges("|Y and <Z").fillet(2.0)
base = base.cut(usb)
# SMA / antenna slot in the right wall, open at the top (the lid closes it)
sma_slot_z0 = sma_zc - SMA_SLOT_W / 2
base = base.cut(box(X1 - 0.5, cc_yc - SMA_SLOT_W / 2, sma_slot_z0, ox1 + 1, cc_yc + SMA_SLOT_W / 2, Z_TOP + 1))

# ----------------------------------------------------------------- lid
lid = rrect(ox0, oy0, ox1, oy1, CORNER_R, Z_TOP, Z_TOP + LID_T)
ring = rrect(X0 + LIP_GAP, Y0 + LIP_GAP, X1 - LIP_GAP, Y1 - LIP_GAP, 0.8, Z_TOP - LIP_H, Z_TOP).cut(
       rrect(X0 + LIP_GAP + LIP_T, Y0 + LIP_GAP + LIP_T, X1 - LIP_GAP - LIP_T, Y1 - LIP_GAP - LIP_T, 0.4, Z_TOP - LIP_H - 1, Z_TOP + 1))
ring = ring.cut(box(X1 - 3, cc_yc - SMA_SLOT_W / 2, Z_TOP - LIP_H - 1, ox1 + 1, cc_yc + SMA_SLOT_W / 2, Z_TOP + 1))  # antenna nut clearance
ring = ring.cut(box(ESP_X - USB_W / 2, oy0 - 1, Z_TOP - LIP_H - 1, ESP_X + USB_W / 2, Y0 + 4, Z_TOP + 1))  # USB plug clearance
lid = lid.union(ring)
# pocket in the lid underside above the antenna slot, so the screwed-on antenna nut clears the plate
lid = lid.cut(box(X1 - 9.0, cc_yc - 6.75, Z_TOP - 0.01, ox1 + 1, cc_yc + 6.75, Z_TOP + 1.5))
for (x, y) in HOLES:
    lid = lid.union(cyl(x, y, Z_BOARD_TOP, Z_TOP + 0.01, PILLAR_OD))
for (x, y) in HOLES:
    lid = lid.cut(cyl(x, y, Z_BOARD_TOP - 1, Z_TOP + LID_T + 1, BORE_D))
    lid = lid.cut(cyl(x, y, Z_TOP + LID_T - CBORE_DEPTH, Z_TOP + LID_T + 1, CBORE_D))

# ----------------------------------------------------------------- component envelopes (for clearance checks)
env = {}
esp_y0, esp_y1 = ky(53.6), ky(1.9)
env["ESP32 module + shield"] = box(15.9, esp_y0, mod_pcb_z0, 44.1, esp_y1, mod_pcb_z1 + 3.8)
env["ESP32 USB-C connector"] = box(ESP_X - 4.6, 0.1, mod_pcb_z1, ESP_X + 4.6, esp_y0, mod_pcb_z1 + 3.3)
env["ESP32 sockets"] = box(16.0, esp_y0 + 2.0, Z_BOARD_TOP, 44.0, esp_y1 - 2.0, mod_pcb_z0)
cc_y0, cc_y1 = cc_yc - CC_PCB[1] / 2, cc_yc + CC_PCB[1] / 2
env["CC1101 module PCB + parts"] = box(CC_X0, cc_y0 + CC_PCB_Y_OFF, mod_pcb_z0, CC_X0 + CC_PCB[0], cc_y1 + CC_PCB_Y_OFF, cc_z1 + 2.5)
env["CC1101 socket"] = box(60.7, cc_yc - 5.2, Z_BOARD_TOP, 65.9, cc_yc + 5.2, mod_pcb_z0)
sma_x0 = CC_X0 + CC_PCB[0]
env["SMA connector"] = cq.Workplane("YZ").workplane(offset=sma_x0).center(cc_yc, sma_zc).circle(SMA_OD / 2).extrude(SMA_LEN).union(
    box(sma_x0 - 1.5, cc_yc - SMA_FLANGE / 2, sma_zc - SMA_FLANGE / 2, sma_x0 + 1.5, cc_yc + SMA_FLANGE / 2, sma_zc + SMA_FLANGE / 2))
env["Antenna nut (screwed on)"] = cq.Workplane("YZ").workplane(offset=sma_x0 + SMA_LEN - 6.0).center(cc_yc, sma_zc).circle(5.5).extrude(10.0)
env["C1 470uF (8x12)"] = cyl(5.8, ky(42.25), Z_BOARD_TOP, Z_BOARD_TOP + 12.5, 8.0)
env["C2 100nF"] = box(12.55, ky(46.06) - 2.9, Z_BOARD_TOP, 15.05, ky(46.06) + 2.9, Z_BOARD_TOP + 7.5)
# C3 (rev B: centre y 14.0, body y 11.5..16.5) stands upright, clear of the CC1101 module edge (~y 18.3), 5 mm dia x 11.5 mm
env["C3 22uF (standing)"] = cyl(75.5, ky(14.0), Z_BOARD_TOP, Z_BOARD_TOP + 11.5, 5.0)
env["C4 100nF"] = box(63.9, ky(17.5) - 1.5, Z_BOARD_TOP, 70.1, ky(17.5) + 1.5, Z_BOARD_TOP + 7.5)
board = box(0, 0, FLOOR + STANDOFF_H, BOARD_W, BOARD_H, Z_BOARD_TOP)

def vol(s): return s.val().Volume()
def inter(a, b):
    try: return vol(a.intersect(b))
    except Exception: return 0.0

if __name__ == "__main__":
    report = {"outer_mm": [round(ox1 - ox0, 1), round(oy1 - oy0, 1), round(Z_TOP + LID_T, 1)],
              "board_top_z": round(Z_BOARD_TOP, 2), "inner_top_z": round(Z_TOP, 2), "lid_underside_gap_over_tallest": None,
              "screw": "M3 x 25 mm pan head (+ M3 x 5.7 mm heat-set inserts in the 4 standoffs)", "issues": []}
    for n, e in env.items():
        for part, s in (("base", base), ("lid", lid)):
            v = inter(e, s)
            if v > 0.5: report["issues"].append("%s intersects %s by %.1f mm3" % (n, part, v))
    names = list(env)
    mated = {("SMA connector", "Antenna nut (screwed on)"), ("CC1101 module PCB + parts", "SMA connector")}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if (names[i], names[j]) in mated: continue
            v = inter(env[names[i]], env[names[j]])
            if v > 0.5: report["issues"].append("%s overlaps %s (%.1f mm3)" % (names[i], names[j], v))
    tallest = max(e.val().BoundingBox().zmax for n, e in env.items() if "Antenna" not in n)
    report["tallest_part_z"] = round(tallest, 1); report["clearance_to_lid_mm"] = round(Z_TOP - tallest, 1)
    report["lid_pillar_vs_board"] = "pillars sit on the board top (z=%.1f), standoffs under it (z=%.1f)" % (Z_BOARD_TOP, FLOOR + STANDOFF_H)
    for nm, s in (("case_base", base), ("case_lid", lid)):
        cq.exporters.export(s, nm + ".stl", tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(s, nm + ".step")
        report[nm + "_volume_cm3"] = round(vol(s) / 1000, 1)
    asm = cq.Assembly()
    asm.add(base, name="base", color=cq.Color(0.2, 0.5, 0.8, 1)); asm.add(lid, name="lid", color=cq.Color(0.9, 0.9, 0.9, 0.6))
    asm.add(board, name="pcb", color=cq.Color(0.1, 0.5, 0.2, 1))
    asm.save("case_assembly.step")
    json.dump(report, open("case_report.json", "w"), indent=1)
    print(json.dumps(report, indent=1))
