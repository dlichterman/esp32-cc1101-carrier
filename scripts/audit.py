import pcbnew, json, math, subprocess, re, sys
b = pcbnew.LoadBoard("esp32_cc1101_carrier.kicad_pcb")
mm = pcbnew.ToMM
R = {}
tr = [t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]
vias = [t for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
# netlist cross-check
subprocess.run(["kicad-cli","sch","export","netlist","--format","kicadsexpr","--output","reports/net.net","esp32_cc1101_carrier.kicad_sch"],check=True,capture_output=True)
txt = open("reports/net.net").read()
nl = {}
for m in re.finditer(r'\(net \(code "?\d+"?\) \(name "([^"]+)"\)(.*?)(?=\n    \(net |\n  \)\n)', txt, re.S):
    for r_, p_ in re.findall(r'\(node \(ref "([^"]+)"\) \(pin "([^"]+)"\)', m.group(2)):
        nl[(r_, p_)] = m.group(1).lstrip("/")
mis = []
for fp in b.GetFootprints():
    for p in fp.Pads():
        key = (fp.GetReference(), p.GetNumber())
        if key in nl and nl[key] != p.GetNetname().lstrip("/"):
            mis.append((key, nl[key], p.GetNetname()))
R["netlist_pad_mismatches"] = mis
R["netlist_pads_checked"] = len(nl)
# 90-degree corners on signal copper
def ends(t): return (round(mm(t.GetStart().x),2),round(mm(t.GetStart().y),2)),(round(mm(t.GetEnd().x),2),round(mm(t.GetEnd().y),2))
at = {}
for t in tr:
    if t.GetNetname() == "GND": continue
    for e in ends(t): at.setdefault((t.GetNetname(), t.GetLayer(), e), []).append(t)
c90 = 0
for k, l in at.items():
    if len(l) == 2:
        v = []
        for t in l:
            s, e = ends(t); o = e if s == k[2] else s
            v.append((o[0]-k[2][0], o[1]-k[2][1]))
        d = v[0][0]*v[1][0]+v[0][1]*v[1][1]
        if abs(d) < 1e-6*max(1, math.hypot(*v[0])*math.hypot(*v[1])): c90 += 1
R["corners_90deg"] = c90
# signal lengths / layers
L = {}
for t in tr:
    n = t.GetNetname()
    if n == "GND": continue
    L.setdefault(n, {}).setdefault(t.GetLayerName(), 0)
    L[n][t.GetLayerName()] += round(mm(t.GetLength()), 1)
R["signal_length_mm"] = L
R["signal_vias"] = sum(1 for v in vias if v.GetNetname() != "GND")
R["stitching_vias"] = sum(1 for v in vias if v.GetNetname() == "GND")
# via to pin distance
pins = [(mm(p.GetPosition().x), mm(p.GetPosition().y)) for fp in b.GetFootprints() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH]
mind = min(math.hypot(mm(v.GetPosition().x)-x, mm(v.GetPosition().y)-y) for v in vias for x, y in pins)
R["min_via_to_pin_mm"] = round(mind, 2)
# GND under signal (B.Cu zone)
zb = [z for z in b.Zones() if z.GetLayer() == pcbnew.B_Cu][0]
tot = under = 0
for t in tr:
    if t.GetLayer() == pcbnew.F_Cu and t.GetNetname() != "GND":
        N = max(2, int(mm(t.GetLength()) / 0.25))
        for i in range(N + 1):
            f = i / N
            x = t.GetStart().x + (t.GetEnd().x - t.GetStart().x) * f
            y = t.GetStart().y + (t.GetEnd().y - t.GetStart().y) * f
            tot += 1
            if zb.HitTestFilledArea(pcbnew.B_Cu, pcbnew.VECTOR2I(int(x), int(y))): under += 1
R["pct_F.Cu_signal_over_B.Cu_ground"] = round(100 * under / tot, 1)
# silkscreen
sw = [mm(d.GetWidth()) for d in b.GetDrawings() if d.GetLayerName().endswith("Silkscreen") and d.GetClass() == "PCB_SHAPE"]
sw += [mm(i.GetStrokeWidth()) for fp in b.GetFootprints() for i in fp.GraphicalItems() if i.GetLayerName().endswith("Silkscreen") and i.GetClass()=="PCB_SHAPE"]
R["min_silk_line_mm"] = round(min(sw), 3) if sw else None
R["drill_sizes_mm"] = sorted({round(mm(p.GetDrillSize().x), 2) for fp in b.GetFootprints() for p in fp.Pads()} | {round(mm(v.GetDrill()), 2) for v in vias})
bb = b.GetBoardEdgesBoundingBox()
R["outline_mm"] = [round(mm(bb.GetWidth()), 2), round(mm(bb.GetHeight()), 2)]
R["drc"] = open("reports/drc.rpt").read().split("** Found ")[1].split(" **")[0]
json.dump(R, open("reports/report.json", "w"), indent=1)
print(json.dumps(R, indent=1))
