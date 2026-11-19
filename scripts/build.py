#!/usr/bin/env /usr/bin/python3
"""
Build the ESP32 + CC1101 carrier KiCad project.

  * converts the VictorLamoine ESP32_DevKit_V1_DOIT legacy symbol -> .kicad_sym
  * writes the schematic (.kicad_sch) using library symbols
  * builds the PCB with KiCad's own pcbnew API (footprints, nets, routed tracks, GND pour)

Run with the system python3 (the one that has `pcbnew`):   /usr/bin/python3 build.py
"""
import heapq
import json
import math
import os
import re
import uuid

import numpy as np
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = "esp32_cc1101_carrier"
LIBDIR = os.path.join(HERE, "libs")
# KiCad's stock libraries; override with KICAD_FP_DIR / KICAD_SYM_DIR (e.g. "C:/Program Files/KiCad/10.0/share/kicad/footprints")
STD_FP = os.environ.get("KICAD_FP_DIR", "/usr/share/kicad/footprints")
STD_SYM = os.environ.get("KICAD_SYM_DIR", "/usr/share/kicad/symbols")
REV = "B"                          # board revision: silkscreen + PCB/schematic title block

# ------------------------------------------------------------------ parameters (mm)
BOARD_W, BOARD_H = 95.0, 55.0
ESP_X, ESP_Y = 30.0, 8.0          # position of the ESP32 footprint origin (midpoint between pad 1 and pad 30)
J1_X, J1_Y = 62.0, 23.0           # CC1101 2x4 socket, pin 1
J1_PITCH_FP = ("Connector_PinHeader_2.54mm", "PinHeader_2x04_P2.54mm_Vertical")
# Decoupling, each as an electrolytic + ceramic pair (populate only what is needed):
#   C1/C2 on VIN (ESP32 pad 15) beside the left pin row; C3/C4 on 3V3 beside the CC1101 header.
# (ref, value, x, y, footprint lib, footprint, symbol, rotation, label dx, label dy, pad-1 net)
CAPS = [("C1", "470uF", 5.8, 44.0, "Capacitor_THT", "CP_Radial_D8.0mm_P3.50mm", "Device:C_Polarized", 90, 0.0, -7.6, "VIN"),
        ("C2", "100nF", 13.8, 43.56, "Capacitor_THT", "C_Disc_D5.0mm_W2.5mm_P5.00mm", "Device:C", 270, 0.0, 8.2, "VIN"),
        # rev B: C3 moved up 2.5 mm (y 17.5 -> 15.0) so its 5 mm body clears the CC1101 breakout's edge (~y 18.3)
        ("C3", "22uF", 74.5, 15.0, "Capacitor_THT", "CP_Radial_D5.0mm_P2.00mm", "Device:C_Polarized", 0, 1.0, -3.6, "+3V3"),
        ("C4", "100nF", 64.5, 17.5, "Capacitor_THT", "C_Disc_D5.0mm_W2.5mm_P5.00mm", "Device:C", 0, 2.5, -2.4, "+3V3")]
HOLES = [(4, 4), (BOARD_W - 4, 4), (4, BOARD_H - 4), (BOARD_W - 4, BOARD_H - 4)]
CLEAR, SIG_W, PWR_W = 0.2, 0.3, 0.5
VIA_D, VIA_DRILL = 0.8, 0.4
G = 0.1                            # router grid
VIA_PIN_CLEAR = 2.2                # mm, via centre to any pin centre (hand rework)

# net -> ESP32 pad numbers (VL footprint numbering) / CC1101 socket pins
ESP_PADS = {"VIN": [15],
            "SPI_SCK": [24],        # IO18
            "SPI_MOSI": [30],       # IO23
            "SPI_MISO": [25],       # IO19
            "CC1101_CSN": [29],     # IO22
            "CC1101_GDO0": [26],    # IO21
            "CC1101_GDO2": [20],    # IO4 (not a strapping pin)
            "+3V3": [16], "GND": [14, 17]}
CC_PINS = {1: "GND", 2: "+3V3", 3: "CC1101_GDO0", 4: "CC1101_CSN", 5: "SPI_SCK", 6: "SPI_MOSI", 7: "SPI_MISO", 8: "CC1101_GDO2"}
CC_NAMES = {1: "GND", 2: "VCC", 3: "GDO0", 4: "CSN", 5: "SCK", 6: "MOSI", 7: "MISO", 8: "GDO2"}
NETS = ["GND", "+3V3", "VIN", "SPI_SCK", "SPI_MOSI", "SPI_MISO", "CC1101_CSN", "CC1101_GDO0", "CC1101_GDO2"]

ROOT_UUID = str(uuid.uuid4())
SYM_UUID = {k: str(uuid.uuid4()) for k in ["U1", "J1", "C1", "C2", "C3", "C4", "H1", "H2", "H3", "H4"]}


def uid():
    return str(uuid.uuid4())


def fmt(v):
    s = ("%.4f" % v).rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


# ================================================================== sexpr helpers
def balanced(text, start):
    """text[start] == '(' -> index just after the matching ')'."""
    depth, i, instr = 0, start, False
    while i < len(text):
        c = text[i]
        if instr:
            if c == "\\":
                i += 1
            elif c == '"':
                instr = False
        else:
            if c == '"':
                instr = True
            elif c == "(":
                depth += 1
            elif c == ")":
                depth -= 1
                if depth == 0:
                    return i + 1
        i += 1
    raise ValueError("unbalanced")


def extract_symbol(libfile, name):
    t = open(libfile).read()
    m = re.search(r'\n[ \t]*\(symbol "%s"\s' % re.escape(name), t)
    if not m:
        raise KeyError(name)
    s = t.index("(", m.start())
    return t[s:balanced(t, s)]


def parse_pins(symtext):
    """-> {number: (x, y, angle, name, type)} in symbol coordinates (mm, y up)."""
    pins = {}
    i = 0
    while True:
        m = re.search(r"\(pin (\w+) \w+", symtext[i:])
        if not m:
            break
        s = i + m.start()
        e = balanced(symtext, s)
        blk = symtext[s:e]
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\)", blk)
        num = re.search(r'\(number "([^"]*)"', blk).group(1)
        nm = re.search(r'\(name "([^"]*)"', blk).group(1)
        pins[num] = (float(at.group(1)), float(at.group(2)), float(at.group(3)), nm, m.group(1))
        i = e
    return pins


# ================================================================== 1. libraries
def build_libs():
    # --- symbol: legacy .lib -> .kicad_sym
    lines = open(os.path.join(LIBDIR, "esp32_devkit_v1_doit.lib")).read().splitlines()
    mm = lambda v: float(v) * 0.0254
    rect, pins = None, []
    for ln in lines:
        p = ln.split()
        if p and p[0] == "S":
            rect = [mm(p[1]), mm(p[2]), mm(p[3]), mm(p[4])]
        if p and p[0] == "X":
            name, num, x, y, ln_, orient = p[1], p[2], mm(p[3]), mm(p[4]), mm(p[5]), p[6]
            etype = {"I": "input", "B": "bidirectional", "W": "power_in", "O": "output", "P": "passive"}[p[11]]
            ang = {"R": 0, "L": 180, "U": 90, "D": 270}[orient]
            pins.append((int(num), name, x, y, ln_, ang, etype))
    pins.sort()
    eff = "(effects (font (size 1.27 1.27)))"
    out = ['(kicad_symbol_lib (version 20220914) (generator "build.py")',
           '  (symbol "ESP32_DevKit_V1_DOIT" (in_bom yes) (on_board yes)',
           '    (property "Reference" "U" (at -11.43 34.29 0) %s)' % eff,
           '    (property "Value" "ESP32_DevKit_V1_DOIT" (at 11.43 34.29 0) %s)' % eff,
           '    (property "Footprint" "ESP32_DevKit_V1_DOIT:esp32_devkit_v1_doit" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
           '    (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))',
           '    (symbol "ESP32_DevKit_V1_DOIT_0_1"',
           '      (rectangle (start %s %s) (end %s %s) (stroke (width 0.254) (type default)) (fill (type background)))' % tuple(fmt(v) for v in rect),
           '    )',
           '    (symbol "ESP32_DevKit_V1_DOIT_1_1"']
    for (num, name, x, y, ln_, ang, et) in pins:
        out.append('      (pin %s line (at %s %s %d) (length %s) (name "%s" %s) (number "%d" %s))'
                   % (et, fmt(x), fmt(y), ang, fmt(ln_), name, eff, num, eff))
    out += ["    )", "  )", ")"]
    open(os.path.join(LIBDIR, "esp32_devkit_v1_doit.kicad_sym"), "w").write("\n".join(out) + "\n")

    # --- footprint: keep VL geometry; point the 3D model at the project-local STEP (drop the old KiCad-5 wrl refs)
    fpf = os.path.join(LIBDIR, "esp32_devkit_v1_doit.pretty", "esp32_devkit_v1_doit.kicad_mod")
    t = open(fpf).read()
    if "KIPRJMOD" not in t:
        pos = 0
        while True:
            m = re.search(r"\n\s*\(model ", t[pos:])
            if not m:
                break
            ms = pos + m.start()
            s = ms + 1
            while t[s] in " \t":
                s += 1
            e = balanced(t, s)
            blk = t[s:e]
            if "VL_PACKAGES3D" in blk:
                pos = e          # keep this one
            else:
                t = t[:ms] + t[e:]
                pos = ms
        t = t.replace("${VL_PACKAGES3D}/esp32_devkit_v1_doit.3dshapes/esp32_devkit_v1_doit.step",
                      "${KIPRJMOD}/libs/3dshapes/esp32_devkit_v1_doit.step")
        open(fpf, "w").write(t)

    t = open(fpf).read()
    out, pos = [], 0
    while True:
        i = t.find("(fp_", pos)
        if i < 0:
            break
        e = balanced(t, i)
        blk = t[i:e]
        if '"F.SilkS"' in blk or '"B.SilkS"' in blk:
            blk = re.sub(r"\(width ([0-9.]+)\)", lambda m: "(width %s)" % fmt(max(float(m.group(1)), 0.2)), blk)
        out.append(t[pos:i])
        out.append(blk)
        pos = e
    out.append(t[pos:])
    open(fpf, "w").write("".join(out))

    t = open(fpf).read()
    t = t.replace("(size 1.524 1.524) (drill 0.762)", "(size 1.8 1.8) (drill 1)")
    open(fpf, "w").write(t)

    # --- library tables
    open(os.path.join(HERE, "sym-lib-table"), "w").write(
        '(sym_lib_table\n  (version 7)\n  (lib (name "ESP32_DevKit_V1_DOIT")(type "KiCad")'
        '(uri "${KIPRJMOD}/libs/esp32_devkit_v1_doit.kicad_sym")(options "")(descr "VictorLamoine ESP32 DevKit V1 DOIT"))\n)\n')
    open(os.path.join(HERE, "fp-lib-table"), "w").write(
        '(fp_lib_table\n  (version 7)\n  (lib (name "ESP32_DevKit_V1_DOIT")(type "KiCad")'
        '(uri "${KIPRJMOD}/libs/esp32_devkit_v1_doit.pretty")(options "")(descr "VictorLamoine ESP32 DevKit V1 DOIT"))\n)\n')
    json.dump({"meta": {"filename": PROJ + ".kicad_pro", "version": 1},
               "net_settings": {"classes": [{"name": "Default", "clearance": CLEAR, "track_width": SIG_W,
                                             "via_diameter": VIA_D, "via_drill": VIA_DRILL}],
                                "meta": {"version": 3}},
               "sheets": [], "text_variables": {}},
              open(os.path.join(HERE, PROJ + ".kicad_pro"), "w"), indent=2)


# ================================================================== 2. schematic
def build_sch():
    esp_sym = extract_symbol(os.path.join(LIBDIR, "esp32_devkit_v1_doit.kicad_sym"), "ESP32_DevKit_V1_DOIT")
    sym = {
        "ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT": esp_sym.replace('(symbol "ESP32_DevKit_V1_DOIT"', '(symbol "ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT"', 1),
    }
    for lib, nm in (("Connector_Generic", "Conn_02x04_Odd_Even"), ("power", "GND"), ("power", "+3V3"),
                    ("power", "PWR_FLAG"), ("Mechanical", "MountingHole"), ("Device", "C"), ("Device", "C_Polarized")):
        txt = extract_symbol("%s/%s.kicad_sym" % (STD_SYM, lib), nm)
        sym["%s:%s" % (lib, nm)] = txt.replace('(symbol "%s"' % nm, '(symbol "%s:%s"' % (lib, nm), 1)
    pins = {k: parse_pins(v) for k, v in sym.items()}

    L = ['(kicad_sch (version 20230121) (generator eeschema)', '  (uuid "%s")' % ROOT_UUID, '  (paper "A3")',
         '  (title_block (title "ESP32 + CC1101 carrier") (rev "%s"))' % REV, '  (lib_symbols']
    for v in sym.values():
        L += ["    " + ln for ln in v.splitlines()]
    L.append("  )")

    pwr_n = [0]

    def place(libid, ref, value, fp, x, y, nums, uuid_=None, hide_ref=False, rot=0):
        u = uuid_ or uid()
        L.append('  (symbol (lib_id "%s") (at %s %s %d) (unit 1) (in_bom yes) (on_board yes) (dnp no) (uuid "%s")'
                 % (libid, fmt(x), fmt(y), rot, u))
        h = " hide" if hide_ref else ""
        L.append('    (property "Reference" "%s" (at %s %s 0) (effects (font (size 1.27 1.27))%s))' % (ref, fmt(x), fmt(y - 4), h))
        L.append('    (property "Value" "%s" (at %s %s 0) (effects (font (size 1.27 1.27))%s))' % (value, fmt(x), fmt(y + 4), h))
        L.append('    (property "Footprint" "%s" (at %s %s 0) (effects (font (size 1.27 1.27)) hide))' % (fp, fmt(x), fmt(y)))
        L.append('    (property "Datasheet" "" (at %s %s 0) (effects (font (size 1.27 1.27)) hide))' % (fmt(x), fmt(y)))
        for n in nums:
            L.append('    (pin "%s" (uuid "%s"))' % (n, uid()))
        L.append('    (instances (project "%s" (path "/%s" (reference "%s") (unit 1))))' % (PROJ, ROOT_UUID, ref))
        L.append("  )")
        return u

    def power(name, x, y):
        pwr_n[0] += 1
        place("power:" + name, "#PWR%02d" % pwr_n[0], name, "", x, y, ["1"], hide_ref=True)

    def flag(x, y):
        pwr_n[0] += 1
        place("power:PWR_FLAG", "#FLG%02d" % pwr_n[0], "PWR_FLAG", "", x, y, ["1"], hide_ref=True)

    def wire(x1, y1, x2, y2):
        L.append('  (wire (pts (xy %s %s) (xy %s %s)) (stroke (width 0) (type default)) (uuid "%s"))' % (fmt(x1), fmt(y1), fmt(x2), fmt(y2), uid()))

    def label(name, x, y, ang):
        j = "right bottom" if ang == 180 else "left bottom"
        L.append('  (label "%s" (at %s %s %d) (fields_autoplaced) (effects (font (size 1.27 1.27)) (justify %s)) (uuid "%s"))'
                 % (name, fmt(x), fmt(y), ang, j, uid()))

    def nc(x, y):
        L.append('  (no_connect (at %s %s) (uuid "%s"))' % (fmt(x), fmt(y), uid()))

    # ---- U1
    X, Y = 101.6, 101.6
    place("ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT", "U1", "ESP32_DevKit_V1_DOIT", "ESP32_DevKit_V1_DOIT:esp32_devkit_v1_doit",
          X, Y, sorted(pins["ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT"], key=int), SYM_UUID["U1"])
    ep = pins["ESP32_DevKit_V1_DOIT:ESP32_DevKit_V1_DOIT"]
    pad2net = {}
    for net, nums in ESP_PADS.items():
        for n in nums:
            pad2net[str(n)] = net
    for num, (px, py, ang, nm, et) in ep.items():
        sx, sy = X + px, Y - py
        net = pad2net.get(num)
        if net in ("GND", "+3V3"):
            continue
        if net:
            label(net, sx, sy, 0 if ang == 180 else 180)   # label text points away from the body
        else:
            nc(sx, sy)
    # power pins
    p16, p14, p17 = ep["16"], ep["14"], ep["17"]
    x16, y16 = X + p16[0], Y - p16[1]
    wire(x16, y16, x16, y16 - 2.54)
    power("+3V3", x16, y16 - 2.54)
    wire(x16, y16 - 2.54, x16 + 5.08, y16 - 2.54)
    flag(x16 + 5.08, y16 - 2.54)
    x14, y14 = X + p14[0], Y - p14[1]
    wire(x14, y14, x14, y14 + 2.54)
    power("GND", x14, y14 + 2.54)
    x17, y17 = X + p17[0], Y - p17[1]
    wire(x17, y17, x17, y17 + 7.62)
    power("GND", x17, y17 + 7.62)
    wire(x17, y17 + 7.62, x17 + 5.08, y17 + 7.62)
    flag(x17 + 5.08, y17 + 7.62)

    # ---- J1 (CC1101)
    JX, JY = 190.5, 88.9
    jp = pins["Connector_Generic:Conn_02x04_Odd_Even"]
    place("Connector_Generic:Conn_02x04_Odd_Even", "J1", "CC1101_Module", "Connector_PinHeader_2.54mm:PinHeader_2x04_P2.54mm_Vertical",
          JX, JY, [str(i) for i in range(1, 9)], SYM_UUID["J1"])
    for num, (px, py, ang, nm, et) in jp.items():
        sx, sy = JX + px, JY - py
        net = CC_PINS.get(int(num))
        if net:
            label(net, sx, sy, 0 if ang == 180 else 180)
        else:
            nc(sx, sy)
    L.append('  (text "CC1101 module pinout (datasheet): 1 GND, 2 VCC(3.3V), 3 GDO0, 4 CSN, 5 SCK, 6 MOSI, 7 MISO, 8 GDO2" (at 170 118 0) (effects (font (size 1.27 1.27)) (justify left bottom)) (uuid "%s"))' % uid())
    L.append('  (text "ESPHome pins: clk=GPIO18, mosi=GPIO23, miso=GPIO19, cs=GPIO22, gdo0=GPIO21, gdo2=GPIO4" (at 170 122 0) (effects (font (size 1.27 1.27)) (justify left bottom)) (uuid "%s"))' % uid())

    # ---- caps
    cx = 222.25
    for ref, val, _x, _y, flib, fname, symid, _rot, _dx, _dy, n1 in CAPS:
        place(symid, ref, val, "%s:%s" % (flib, fname), cx, 88.9, ["1", "2"], SYM_UUID[ref])
        if n1 == "VIN":
            label("VIN", cx, 88.9 - 3.81, 0)
            if ref == "C1":
                flag(cx, 88.9 - 3.81)
        else:
            power("+3V3", cx, 88.9 - 3.81)
        power("GND", cx, 88.9 + 3.81)
        cx += 15.24
    # ---- mounting holes
    hx = 170.18
    for i in range(4):
        place("Mechanical:MountingHole", "H%d" % (i + 1), "MountingHole_3.2mm", "MountingHole:MountingHole_3.2mm_M3",
              hx + i * 15.24, 135.89, [], SYM_UUID["H%d" % (i + 1)])
    L.append('  (sheet_instances (path "/" (page "1")))')
    L.append(")")
    open(os.path.join(HERE, PROJ + ".kicad_sch"), "w").write("\n".join(L) + "\n")


# ================================================================== 3. PCB
def mmv(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def build_pcb():
    board = pcbnew.BOARD()
    netinfo = {}
    for i, n in enumerate(NETS):
        ni = pcbnew.NETINFO_ITEM(board, n if n in ("GND", "+3V3") else "/" + n, i + 1)
        board.Add(ni)
        netinfo[n] = ni

    def load(lib_dir, nick, name, ref, value, x, y, sym_key=None):
        fp = pcbnew.FootprintLoad(lib_dir, name)
        fp.SetFPID(pcbnew.LIB_ID(nick, name))
        fp.SetReference(ref)
        fp.SetValue(value)
        fp.SetPosition(mmv(x, y))
        if sym_key:
            fp.SetPath(pcbnew.KIID_PATH("/%s/%s" % (ROOT_UUID, SYM_UUID[sym_key])))
        board.Add(fp)
        return fp

    esp = load(os.path.join(LIBDIR, "esp32_devkit_v1_doit.pretty"), "ESP32_DevKit_V1_DOIT", "esp32_devkit_v1_doit",
               "U1", "ESP32_DevKit_V1_DOIT", ESP_X, ESP_Y, "U1")
    j1 = load(STD_FP + "/Connector_PinHeader_2.54mm.pretty", "Connector_PinHeader_2.54mm", "PinHeader_2x04_P2.54mm_Vertical",
              "J1", "CC1101_Module", J1_X, J1_Y, "J1")
    caps = []
    for ref, val, x, y, flib, fname, _sym, rot, _dx, _dy, _n1 in CAPS:
        cf = load("%s/%s.pretty" % (STD_FP, flib), flib, fname, ref, val, x, y, ref)
        cf.SetOrientationDegrees(rot)
        caps.append(cf)
    holes = []
    for i, (hx, hy) in enumerate(HOLES):
        holes.append(load(STD_FP + "/MountingHole.pretty", "MountingHole", "MountingHole_3.2mm_M3", "H%d" % (i + 1),
                          "MountingHole_3.2mm", hx, hy, "H%d" % (i + 1)))
    for fp in [esp, j1] + caps + holes:
        for t in (fp.Reference(), fp.Value()):
            pass
    for h in holes:
        h.Reference().SetVisible(False)
        h.Value().SetVisible(False)
    esp.Value().SetVisible(False)
    j1.Value().SetVisible(False)
    for c in caps:
        c.Value().SetVisible(False)

    for c, cd in zip(caps, CAPS):
        c.Reference().SetPosition(mmv(cd[2] + cd[8], cd[3] + cd[9]))
    j1.Reference().SetPosition(mmv(J1_X + 1.27, J1_Y - 3.0))

    # ---- nets on pads
    def setnet(fp, num, net):
        for p in fp.Pads():
            if p.GetNumber() == str(num):
                p.SetNet(netinfo[net])
                return
        raise KeyError((fp.GetReference(), num))

    for net, nums in ESP_PADS.items():
        for n in nums:
            setnet(esp, n, net)
    for n, net in CC_PINS.items():
        setnet(j1, n, net)
    for c, cd in zip(caps, CAPS):
        setnet(c, 1, cd[10])      # matches the schematic (pin 1 / "+" = supply, pin 2 = GND)
        setnet(c, 2, "GND")

    # ---- gather pad geometry for the router
    pads = {n: [] for n in NETS}
    obstacles = []   # unnetted copper: (x, y, r)
    for fp in [esp, j1] + caps:
        for p in fp.Pads():
            pos = p.GetPosition()
            x, y = pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
            sz = p.GetSize()
            r = max(pcbnew.ToMM(sz.x), pcbnew.ToMM(sz.y)) / 2.0
            if p.GetShape() in (pcbnew.PAD_SHAPE_RECT, pcbnew.PAD_SHAPE_ROUNDRECT):
                r = math.hypot(pcbnew.ToMM(sz.x), pcbnew.ToMM(sz.y)) / 2.0   # square pad: use corner radius
            thru = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
            layers = (0, 1) if thru else (0,)
            net = p.GetNetname().lstrip("/")
            if net:
                pads[net].append((x, y, layers, r))
            else:
                obstacles.append((layers, x, y, r))
    for h in holes:
        for p in h.Pads():
            pos = p.GetPosition()
            obstacles.append(((0, 1), pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y), 1.6 + 0.3))

    route_and_add(board, netinfo, pads, obstacles)
    print('routed+added', flush=True)
    finish_board(board, netinfo)
    print('finished', flush=True)
    # stitching vias must not sit under silkscreen text/lines
    boxes = []
    for d in board.GetDrawings():
        if d.GetLayerName() in ("F.Silkscreen", "B.Silkscreen"):
            boxes.append(d.GetBoundingBox())
    for fp in board.GetFootprints():
        for it in list(fp.GraphicalItems()) + [fp.Reference(), fp.Value()]:
            if it.GetLayerName() in ("F.Silkscreen", "B.Silkscreen") and (it.GetClass() not in ("FP_TEXT", "PCB_TEXT", "PCB_FIELD") or it.IsVisible()):
                boxes.append(it.GetBoundingBox())
    n_rm = 0
    for t in list(board.GetTracks()):
        if t.GetClass() == "PCB_VIA" and t.GetNetname() == "GND":
            p = t.GetPosition()
            for bb in boxes:
                b2 = pcbnew.BOX2I(bb.GetPosition(), bb.GetSize())
                b2.Inflate(pcbnew.FromMM(0.6))
                if b2.Contains(p):
                    board.Remove(t)
                    n_rm += 1
                    break
    print("stitching vias removed from under silkscreen:", n_rm, flush=True)
    for p_ in board.FindFootprintByReference("J1").Pads():
        if p_.GetNumber() == "1":      # boxed in by the 3V3 neighbour: solid tie instead of starved spokes
            # KiCad 8+ renamed SetZoneConnection -> SetLocalZoneConnection
            (getattr(p_, "SetLocalZoneConnection", None) or p_.SetZoneConnection)(pcbnew.ZONE_CONNECTION_FULL)
    path = os.path.join(HERE, PROJ + ".kicad_pcb")
    board.Save(path)
    # reload (so the board has its project/settings), fill the GND pour, save again
    board = pcbnew.LoadBoard(path)
    board.BuildConnectivity()
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(path)


# ---------------------------------------------------------------- router (2-layer grid A*, vias)
NX, NY = int(round(BOARD_W / G)) + 1, int(round(BOARD_H / G)) + 1


def stamp(mask, x, y, r):
    cx, cy, rc = x / G, y / G, r / G
    x0, x1 = max(0, int(math.floor(cx - rc - 1))), min(NX - 1, int(math.ceil(cx + rc + 1)))
    y0, y1 = max(0, int(math.floor(cy - rc - 1))), min(NY - 1, int(math.ceil(cy + rc + 1)))
    if x0 > x1 or y0 > y1:
        return
    xs = np.arange(x0, x1 + 1)[None, :]
    ys = np.arange(y0, y1 + 1)[:, None]
    mask[y0:y1 + 1, x0:x1 + 1] |= ((xs - cx) ** 2 + (ys - cy) ** 2) <= rc * rc + 1e-9


def stamp_rect(mask, x0, y0, x1, y1):
    a, b = max(0, int(round(x0 / G))), min(NX - 1, int(round(x1 / G)))
    c, d = max(0, int(round(y0 / G))), min(NY - 1, int(round(y1 / G)))
    mask[c:d + 1, a:b + 1] = True


def route_and_add(board, netinfo, pads, obstacles):
    tracks, vias = [], []
    dots = {n: [(L, x, y, r) for (x, y, ls, r) in pads[n] for L in ls] for n in NETS}
    tdots = {n: [] for n in NETS}   # routed track/via copper (for spacing rules)

    # keep-out under the ESP32 antenna (all layers) and ESP32 mid-pitch spots (no threading between header pads)
    ant = (ESP_X - 11.5, ESP_Y - 6.5, ESP_X + 11.5, ESP_Y + 1.2)
    mid_spots = []
    for col in (-12.7, 12.7):
        for i in range(14):
            mid_spots.append((ESP_X + col, ESP_Y + i * 2.54 + 1.27, 0.3))

    def build_masks(net, width, extra=0.0):
        trk = [np.zeros((NY, NX), bool) for _ in range(2)]
        via = [np.zeros((NY, NX), bool) for _ in range(2)]
        for n_ in NETS:                       # rework rule: no via within 2.2 mm of any part pin
            for (x, y, ls, r) in pads[n_]:
                for L in (0, 1):
                    stamp(via[L], x, y, VIA_PIN_CLEAR)
        for (layers, x, y, r) in obstacles:
            for L in (0, 1):
                stamp(via[L], x, y, VIA_PIN_CLEAR)
        rt, rv = CLEAR + width / 2 + 0.02, CLEAR + VIA_D / 2 + 0.02
        for other in NETS:
            if other == net:
                continue
            for (L, x, y, r) in dots[other]:
                stamp(trk[L], x, y, r + rt)
                stamp(via[L], x, y, r + rv)
            for (L, x, y, r) in tdots[other]:      # already-routed copper: keep extra air gap (crosstalk spacing)
                stamp(trk[L], x, y, r + rt + extra)
                stamp(via[L], x, y, r + rv + extra)
        for (layers, x, y, r) in obstacles:
            for L in layers:
                stamp(trk[L], x, y, r + rt)
                stamp(via[L], x, y, r + rv)
        for (x, y, r) in mid_spots:
            for L in (0, 1):
                stamp(trk[L], x, y, r + width / 2)
                stamp(via[L], x, y, r + VIA_D / 2)
        for L in (0, 1):
            stamp_rect(trk[L], *ant)
            stamp_rect(via[L], *ant)
            k = int(math.ceil((0.6 + width / 2) / G))
            for m in (trk[L], via[L]):
                m[:k, :] = m[-k:, :] = True
                m[:, :k] = m[:, -k:] = True
        return trk, via

    snap = lambda x, y: (int(round(x / G)), int(round(y / G)))

    def route_net(net, width, via_cost=80, extra=0.0, f_only=True):
        trk, via = build_masks(net, width, extra)
        if f_only:
            trk[1][:, :] = True      # keep the B.Cu ground plane unbroken: no signal routing on the bottom layer
        P = pads[net]
        if len(P) < 2:
            return True
        tree = set()

        def add_tree(p):
            ix, iy = snap(p[0], p[1])
            for L in p[2]:
                tree.add((L, ix, iy))
        connected = [P[0]]
        add_tree(P[0])
        remaining = list(P[1:])
        while remaining:
            remaining.sort(key=lambda p: min(abs(p[0] - c[0]) + abs(p[1] - c[1]) for c in connected))
            tgt = remaining.pop(0)
            tix, tiy = snap(tgt[0], tgt[1])
            tnodes = {(L, tix, tiy) for L in tgt[2]}
            openq, best, prev = [], {}, {}
            for s in tree:
                best[s] = 0
                heapq.heappush(openq, (abs(s[1] - tix) + abs(s[2] - tiy), 0, s))
            found = None
            while openq:
                f, g, node = heapq.heappop(openq)
                if best.get(node, 1e18) < g:
                    continue
                if node in tnodes:
                    found = node
                    break
                L, ix, iy = node
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    nx, ny = ix + dx, iy + dy
                    if not (0 <= nx < NX and 0 <= ny < NY):
                        continue
                    nn = (L, nx, ny)
                    if trk[L][ny, nx] and nn not in tnodes and nn not in tree:
                        continue
                    pn = prev.get(node)
                    bend = 2 if (pn is not None and pn[0] == L and (ix - pn[1], iy - pn[2]) != (dx, dy)) else 0
                    ng = g + 1 + bend + (0 if L == 0 else 1)   # mild preference for F.Cu
                    if ng < best.get(nn, 1e18):
                        best[nn] = ng
                        prev[nn] = node
                        heapq.heappush(openq, (ng + abs(nx - tix) + abs(ny - tiy), ng, nn))
                oL = 1 - L
                if not via[0][iy, ix] and not via[1][iy, ix] and not trk[oL][iy, ix]:
                    nn = (oL, ix, iy)
                    ng = g + via_cost
                    if ng < best.get(nn, 1e18):
                        best[nn] = ng
                        prev[nn] = node
                        heapq.heappush(openq, (ng + abs(ix - tix) + abs(iy - tiy), ng, nn))
            if found is None:
                return False
            path = [found]
            while path[-1] in prev and path[-1] not in tree:
                path.append(prev[path[-1]])
            path.reverse()
            emit(net, width, path)
            tree.update(path)
            connected.append(tgt)
            add_tree(tgt)
        return True

    def emit(net, width, path):
        xy = lambda n: (n[1] * G, n[2] * G)
        runs, cur = [], [path[0]]
        for a, b in zip(path, path[1:]):
            if a[0] != b[0]:
                runs.append(("run", cur))
                runs.append(("via", a))
                cur = [b]
            else:
                cur.append(b)
        runs.append(("run", cur))
        for kind, d in runs:
            if kind == "via":
                x, y = xy(d)
                vias.append((net, x, y))
                tdots[net] += [(0, x, y, VIA_D / 2), (1, x, y, VIA_D / 2)]
                continue
            L = d[0][0]
            pts = [d[0]]
            for k in range(1, len(d) - 1):
                a, b, c = d[k - 1], d[k], d[k + 1]
                if (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2]):
                    pts.append(b)
            if len(d) > 1:
                pts.append(d[-1])
            for a, b in zip(pts, pts[1:]):
                tracks.append((net, L, xy(a), xy(b), width))
            for n in d:
                x, y = xy(n)
                tdots[net].append((L, x, y, width / 2))

    # ---- routing: try many net orders; keep the one with the shortest total copper and the least bottom-layer
    # use (each B.Cu hop interrupts the B.Cu ground plane, so hops are penalised; ground is stitched around them).
    import time, random as _rnd
    SIGS = ["SPI_SCK", "SPI_MOSI", "SPI_MISO", "CC1101_CSN", "CC1101_GDO0", "CC1101_GDO2"]
    WID = {n: SIG_W for n in SIGS}
    WID["+3V3"] = PWR_W
    WID["VIN"] = PWR_W

    def route_all(order):
        tracks.clear()
        vias.clear()
        for n in NETS:
            tdots[n].clear()
        for net in order:
            ok = False
            for extra in (0.8, 0.5, 0.3, 0.0):  # prefer >=1.0 mm air gap (3W+) between nets, relax only if needed
                state = (len(tracks), len(vias), len(tdots[net]))
                if route_net(net, WID[net], extra=extra, f_only=False):
                    ok = True
                    break
                del tracks[state[0]:]
                del vias[state[1]:]
                del tdots[net][state[2]:]
            if not ok:
                return None
        ln = lambda t: math.hypot(t[3][0] - t[2][0], t[3][1] - t[2][1])
        tot = sum(ln(t) for t in tracks)
        bot = sum(ln(t) for t in tracks if t[1] == 1)
        return tot + 3 * bot + 8 * len(vias)

    rnd = _rnd.Random(7)
    orders = [["SPI_SCK", "SPI_MOSI", "SPI_MISO", "CC1101_CSN", "CC1101_GDO0", "CC1101_GDO2", "+3V3", "VIN"]]
    pool = SIGS + ["+3V3", "VIN"]
    while len(orders) < 30:
        o = pool[:]
        rnd.shuffle(o)
        if o not in orders:
            orders.append(o)
    best = None
    for o in orders:
        t0 = time.time()
        m = route_all(o)
        print("order %-70s score %s (%.1fs)" % (",".join(o), None if m is None else round(m, 1), time.time() - t0), flush=True)
        if m is not None and (best is None or m < best[0]):
            best = (m, o, list(tracks), list(vias), {n: list(tdots[n]) for n in NETS})
    if best is None:
        raise SystemExit("ROUTING FAILED")
    print("BEST", best[1], round(best[0], 1), flush=True)
    tracks[:] = best[2]
    vias[:] = best[3]
    for n in NETS:
        tdots[n][:] = best[4][n]
    # ---- no 90-degree corners: chamfer every orthogonal corner (largest cut that keeps clearance)
    def seg_pt_dist(p, a_, b_):
        vx, vy = b_[0] - a_[0], b_[1] - a_[1]
        L2 = vx * vx + vy * vy
        t_ = 0 if L2 == 0 else max(0, min(1, ((p[0] - a_[0]) * vx + (p[1] - a_[1]) * vy) / L2))
        return math.hypot(p[0] - (a_[0] + t_ * vx), p[1] - (a_[1] + t_ * vy))

    def seg_seg_dist(a_, b_, c_, d_):
        def ccw(p, q, r):
            return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
        if ccw(a_, b_, c_) * ccw(a_, b_, d_) < 0 and ccw(c_, d_, a_) * ccw(c_, d_, b_) < 0:
            return 0.0
        return min(seg_pt_dist(a_, c_, d_), seg_pt_dist(b_, c_, d_), seg_pt_dist(c_, a_, b_), seg_pt_dist(d_, a_, b_))

    def chamfer_ok(net, L, p, q, w):
        need = CLEAR + 0.04
        for (n2, L2, a_, b_, w2) in tracks:
            if n2 != net and L2 == L and seg_seg_dist(p, q, a_, b_) - w / 2 - w2 / 2 < need:
                return False
        for n2 in NETS:
            if n2 != net:
                for (x, y, ls, r) in pads[n2]:
                    if L in ls and seg_pt_dist((x, y), p, q) - r - w / 2 < need:
                        return False
        for (layers, x, y, r) in obstacles:
            if L in layers and seg_pt_dist((x, y), p, q) - r - w / 2 < need:
                return False
        for (n2, x, y) in vias:
            if n2 != net and seg_pt_dist((x, y), p, q) - VIA_D / 2 - w / 2 < need:
                return False
        return True

    kf = lambda p: (round(p[0], 2), round(p[1], 2))
    padpts = {kf((x, y)) for n_ in NETS for (x, y, ls, r) in pads[n_]}
    n_cham = 0
    while True:
        ends = {}
        for i, (n, L, a_, b_, w) in enumerate(tracks):
            ends.setdefault((n, L, kf(a_)), []).append((i, 0))
            ends.setdefault((n, L, kf(b_)), []).append((i, 1))
        done_one = False
        for (n, L, k), lst in ends.items():
            if len(lst) != 2 or k in padpts or lst[0][0] == lst[1][0]:
                continue
            (i1, e1), (i2, e2) = lst
            s1, s2 = tracks[i1], tracks[i2]

            def away(sg, e):
                p0, p1 = (sg[2], sg[3]) if e == 0 else (sg[3], sg[2])
                l_ = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
                return p0, ((p1[0] - p0[0]) / l_, (p1[1] - p0[1]) / l_), l_
            c1, u1, l1 = away(s1, e1)
            c2, u2, l2 = away(s2, e2)
            if abs(u1[0] * u2[0] + u1[1] * u2[1]) > 1e-6:
                continue
            for d in (1.5, 1.0, 0.6):
                if l1 < d + 0.2 or l2 < d + 0.2:
                    continue
                p = (c1[0] + u1[0] * d, c1[1] + u1[1] * d)
                q = (c2[0] + u2[0] * d, c2[1] + u2[1] * d)
                if chamfer_ok(n, L, p, q, s1[4]):
                    tracks[i1] = (n, L, p, s1[3], s1[4]) if e1 == 0 else (n, L, s1[2], p, s1[4])
                    tracks[i2] = (n, L, q, s2[3], s2[4]) if e2 == 0 else (n, L, s2[2], q, s2[4])
                    tracks.append((n, L, p, q, s1[4]))
                    n_cham += 1
                    done_one = True
                    break
            if done_one:
                break
        if not done_one:
            break
    print("45-degree chamfers:", n_cham, flush=True)

    # ---- GND: through-hole pads tie into the B.Cu pour; SMD cap GND pads get a short track + via into it
    for (x, y, layers, r) in pads["GND"]:
        if layers == (0,):
            vx, vy = x - 1.9, y
            tracks.append(("GND", 0, (x, y), (vx, vy), PWR_W))
            vias.append(("GND", vx, vy))

    # ---- stitching vias (tie F.Cu and B.Cu ground together): beside every layer-change via, along both sides of
    # longer traces (~5 mm), as a fence around the board edge (~5 mm) and on an interior grid. All >= 2.2 mm from pins.
    _, via_m = build_masks("GND", VIA_D, extra=0.12)

    def free(px, py):
        ix, iy = snap(px, py)
        if not (0 <= ix < NX and 0 <= iy < NY):
            return None
        if iy * G <= ESP_Y + 2.0:
            return None                                    # antenna notch: no copper here
        if via_m[0][iy, ix] or via_m[1][iy, ix]:
            return None
        return (ix * G, iy * G)

    def spaced(px, py, d):
        return not any(abs(px - vx) < d and abs(py - vy) < d for (_, vx, vy) in vias)
    n_st = 0
    for (n, vx, vy) in [v for v in vias if v[0] != "GND"]:          # (1) return-path via beside each signal via
        got = 0
        for dx, dy in ((2.0, 0), (-2.0, 0), (0, 2.0), (0, -2.0), (1.8, 1.8), (-1.8, 1.8), (1.8, -1.8), (-1.8, -1.8),
                       (3.0, 0), (-3.0, 0), (0, 3.0), (0, -3.0)):
            pt = free(vx + dx, vy + dy)
            if pt and spaced(pt[0], pt[1], 1.9) and got < 2:
                vias.append(("GND", pt[0], pt[1]))
                got += 1
                n_st += 1
    for (n, L, a_, b_, w) in list(tracks):                           # (2) both sides of longer traces
        ln_ = math.hypot(b_[0] - a_[0], b_[1] - a_[1])
        if L != 0 or n == "GND" or ln_ < 6.0:
            continue
        ux, uy = (b_[0] - a_[0]) / ln_, (b_[1] - a_[1]) / ln_
        k = 2.5
        while k < ln_ - 1.5:
            for side in (1, -1):
                pt = free(a_[0] + ux * k - uy * 1.6 * side, a_[1] + uy * k + ux * 1.6 * side)
                if pt and spaced(pt[0], pt[1], 1.9):
                    vias.append(("GND", pt[0], pt[1]))
                    n_st += 1
            k += 5.0
    inset = 1.6                                                       # (3) edge fence
    ring = []
    for k in np.arange(inset, BOARD_W - inset + 1e-6, 5.08):
        ring += [(k, inset), (k, BOARD_H - inset)]
    for k in np.arange(inset, BOARD_H - inset + 1e-6, 5.08):
        ring += [(inset, k), (BOARD_W - inset, k)]
    for (cx_, cy_) in ring:
        pt = free(cx_, cy_)
        if pt and spaced(pt[0], pt[1], 1.9):
            vias.append(("GND", pt[0], pt[1]))
            n_st += 1
    for xx in np.arange(7.62, BOARD_W - 4, 7.62):                    # (4) interior grid
        for yy in np.arange(7.62, BOARD_H - 4, 7.62):
            pt = free(xx, yy)
            if pt and spaced(pt[0], pt[1], 3.8):
                vias.append(("GND", pt[0], pt[1]))
                n_st += 1
    print("stitching vias:", n_st, flush=True)

    # ---- push into the board; snap pad-center stubs for off-grid pads
    def add_track(net, L, a, b, w):
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(mmv(*a))
        t.SetEnd(mmv(*b))
        t.SetWidth(pcbnew.FromMM(w))
        t.SetLayer(pcbnew.F_Cu if L == 0 else pcbnew.B_Cu)
        t.SetNet(netinfo[net])
        board.Add(t)

    for (net, L, a, b, w) in tracks:
        add_track(net, L, a, b, w)
    print('tracks added', flush=True)
    # stubs from true pad centre to the routing lattice node (pads are not all on the 0.1 mm lattice)
    for net in NETS:
        if net == "GND":
            continue
        w = PWR_W if net in ("+3V3", "VIN") else SIG_W
        for (x, y, layers, r) in pads[net]:
            ix, iy = snap(x, y)
            nx_, ny_ = ix * G, iy * G
            if abs(nx_ - x) > 1e-6 or abs(ny_ - y) > 1e-6:
                L = None
                for (n2, L2, a, b, w2) in tracks:
                    if n2 == net and any(abs(p[0] - nx_) < 1e-6 and abs(p[1] - ny_) < 1e-6 for p in (a, b)):
                        L = L2
                        break
                if L is not None and L in layers:
                    add_track(net, L, (x, y), (nx_, ny_), min(w, 0.3))
    for (net, x, y) in vias:
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(mmv(x, y))
        v.SetWidth(pcbnew.FromMM(VIA_D))
        v.SetDrill(pcbnew.FromMM(VIA_DRILL))
        v.SetViaType(pcbnew.VIATYPE_THROUGH)
        v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
        v.SetNet(netinfo[net])
        board.Add(v)


def finish_board(board, netinfo):
    tb = board.GetTitleBlock()
    tb.SetTitle("ESP32 + CC1101 carrier")
    tb.SetRevision(REV)
    # outline
    pts = [(0, 0), (BOARD_W, 0), (BOARD_W, BOARD_H), (0, BOARD_H)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetStart(mmv(*a))
        s.SetEnd(mmv(*b))
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        board.Add(s)

    def text(s, x, y, size=1.0, layer=pcbnew.F_SilkS, angle=0, just=None):
        size = max(size, 1.0)   # JLCPCB: silkscreen text >= 1.0 mm high, >= 0.2 mm stroke
        t = pcbnew.PCB_TEXT(board)
        t.SetText(s)
        t.SetPosition(mmv(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
        t.SetTextThickness(pcbnew.FromMM(0.2))
        t.SetTextAngle(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
        if just == 'r':
            t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_RIGHT)
        elif just == 'l':
            t.SetHorizJustify(pcbnew.GR_TEXT_H_ALIGN_LEFT)
        board.Add(t)
    text("ESP32 + CC1101 carrier rev %s" % REV, 66.0, 4.0, 1.0)
    text("ESP32 DevKit V1: antenna end up", 45.9, 31.0, 1.0, angle=90)
    text("USB end down", 47.5, 31.0, 1.0, angle=90)
    for i in range(4):
        text("%d %s" % (2 * i + 1, CC_NAMES[2 * i + 1]), J1_X - 10.5, J1_Y + i * 2.54, 1.0)
        text("%d %s" % (2 * i + 2, CC_NAMES[2 * i + 2]), J1_X + 2.54 + 8.5, J1_Y + i * 2.54, 1.0)
    # bump every silkscreen stroke/text in the footprints up to the fab minimum
    for fp in board.GetFootprints():
        for g in fp.GraphicalItems():
            if g.GetLayerName() in ("F.Silkscreen", "B.Silkscreen"):
                if g.GetClass() in ("FP_TEXT", "PCB_TEXT"):   # KiCad 7 / KiCad 8+
                    if g.GetTextThickness() < pcbnew.FromMM(0.2):
                        g.SetTextThickness(pcbnew.FromMM(0.2))
                    if g.GetTextHeight() < pcbnew.FromMM(1.0):
                        g.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(1.0), pcbnew.FromMM(1.0)))
                elif g.GetWidth() < pcbnew.FromMM(0.2):
                    g.SetWidth(pcbnew.FromMM(0.2))
        for t in (fp.Reference(), fp.Value()):
            if t.GetTextThickness() < pcbnew.FromMM(0.2):
                t.SetTextThickness(pcbnew.FromMM(0.2))
    # 50 mm ruler on the User.Drawings layer (not exported to fab): print the PDF at 100 % and check it
    for (a_, b_) in [((54, 47.8), (84, 47.8))] + [((54 + 10 * i, 47.0), (54 + 10 * i, 48.6)) for i in range(4)]:
        sh = pcbnew.PCB_SHAPE(board)
        sh.SetShape(pcbnew.SHAPE_T_SEGMENT)
        sh.SetStart(mmv(*a_))
        sh.SetEnd(mmv(*b_))
        sh.SetLayer(pcbnew.Dwgs_User)
        sh.SetWidth(pcbnew.FromMM(0.15))
        board.Add(sh)
    text("30 mm ruler: print at 100%", 69.0, 50.0, 1.0, layer=pcbnew.Dwgs_User)
    # GND pour on B.Cu, kept out from under the ESP32 antenna
    for lay in (pcbnew.B_Cu, pcbnew.F_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(lay)
        z.SetNet(netinfo["GND"])
        z.SetLocalClearance(pcbnew.FromMM(0.3))
        z.SetMinThickness(pcbnew.FromMM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(pcbnew.FromMM(0.3))
        z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.45))
        ol = pcbnew.SHAPE_LINE_CHAIN()
        ax0, ax1, ay1 = ESP_X - 14.0, ESP_X + 14.0, ESP_Y + 1.2   # copper-free notch under/above the ESP32 antenna
        for (x, y) in [(0.5, ay1), (BOARD_W - 0.5, ay1),             # copper-free strip across the full top of both layers
                       (BOARD_W - 0.5, BOARD_H - 0.5), (0.5, BOARD_H - 0.5)]:
            ol.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        ol.SetClosed(True)
        z.Outline().AddOutline(ol)
        board.Add(z)




def center_board(page_w=297.0, page_h=210.0):
    """Shift the finished board so it sits in the middle of the A4 sheet; board corner becomes the plot/drill origin."""
    path = os.path.join(HERE, PROJ + ".kicad_pcb")
    board = pcbnew.LoadBoard(path)
    dx = round((page_w - BOARD_W) / 2 * 2) / 2
    dy = round((page_h - BOARD_H) / 2 * 2) / 2
    v = pcbnew.VECTOR2I(pcbnew.FromMM(dx), pcbnew.FromMM(dy))
    for fp in board.GetFootprints():
        fp.Move(v)
    for t in board.GetTracks():
        t.Move(v)
    for z in board.Zones():
        z.Move(v)
    for d in board.GetDrawings():
        d.Move(v)
    org = pcbnew.VECTOR2I(pcbnew.FromMM(dx), pcbnew.FromMM(dy))
    board.GetDesignSettings().m_SolderMaskExpansion = pcbnew.FromMM(0.05)   # JLCPCB standard mask opening
    board.GetDesignSettings().m_SolderMaskMinWidth = pcbnew.FromMM(0.1)
    board.GetDesignSettings().SetAuxOrigin(org)
    board.GetDesignSettings().SetGridOrigin(org)
    board.Save(path)
    board = pcbnew.LoadBoard(path)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(path)
    print("centred on page: offset %.1f, %.1f mm" % (dx, dy), flush=True)


if __name__ == "__main__":
    build_libs()
    build_sch()
    build_pcb()
    center_board()
    print("built")
