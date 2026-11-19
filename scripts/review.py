import pcbnew, math, re, subprocess, json
mm = pcbnew.ToMM
b = pcbnew.LoadBoard("esp32_cc1101_carrier.kicad_pcb")
ds = b.GetDesignSettings()
R = {}
bb = b.GetBoardEdgesBoundingBox()
L, T, Rr, B = mm(bb.GetX()), mm(bb.GetY()), mm(bb.GetRight()), mm(bb.GetBottom())
R["outline"] = [round(Rr - L, 2), round(B - T, 2)]
# ---- 1. pin map: pad -> net, and ESP pad -> symbol pin name
sym = open("libs/esp32_devkit_v1_doit.kicad_sym").read()
names = {n: nm for nm, n in re.findall(r'\(name "([^"]+)" \(effects[^)]*\(font[^)]*\)\)[^)]*\)\s*\(number "(\d+)"', sym)}
esp = b.FindFootprintByReference("U1"); j1 = b.FindFootprintByReference("J1")
R["esp_used_pads"] = {p.GetNumber() + " " + names.get(p.GetNumber(), "?"): p.GetNetname() for p in esp.Pads() if p.GetNetname()}
R["j1_pads"] = {p.GetNumber(): p.GetNetname() for p in j1.Pads()}
R["esp_row_spacing_mm"] = round(abs(mm(esp.FindPadByNumber("1").GetPosition().x - esp.FindPadByNumber("30").GetPosition().x)), 2)
R["esp_pitch_mm"] = round(mm(esp.FindPadByNumber("2").GetPosition().y - esp.FindPadByNumber("1").GetPosition().y), 2)
# ---- 2. drills / annular rings / hole-to-hole
holes = []
min_ring = 9
for fp in b.GetFootprints():
    for p in fp.Pads():
        d = mm(p.GetDrillSize().x)
        if d > 0:
            holes.append((mm(p.GetPosition().x), mm(p.GetPosition().y), d, fp.GetReference()))
            if p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                min_ring = min(min_ring, (mm(p.GetSize().x) - d) / 2)
for v in b.GetTracks():
    if v.GetClass() == "PCB_VIA":
        d = mm(v.GetDrill()); holes.append((mm(v.GetPosition().x), mm(v.GetPosition().y), d, "via"))
        min_ring = min(min_ring, (mm(v.GetWidth()) - d) / 2)
mh = 99; pair = None
for i in range(len(holes)):
    for j in range(i + 1, len(holes)):
        a, c = holes[i], holes[j]
        g = math.hypot(a[0] - c[0], a[1] - c[1]) - a[2] / 2 - c[2] / 2
        if g < mh: mh, pair = g, (a[3], c[3])
R["min_annular_ring_mm"] = round(min_ring, 3)
R["min_hole_to_hole_edge_mm"] = round(mh, 2); R["closest_holes"] = pair
R["min_drill_mm"] = round(min(h[2] for h in holes), 2)
# ---- 3. copper to board edge (tracks, vias, pads)
def edge_gap(x, y, r):
    return min(x - L, Rr - x, y - T, B - y) - r
mg = 99; who = None
for t in b.GetTracks():
    if t.GetClass() == "PCB_TRACK":
        for pt in (t.GetStart(), t.GetEnd()):
            g = edge_gap(mm(pt.x), mm(pt.y), mm(t.GetWidth()) / 2)
            if g < mg: mg, who = g, "track"
    else:
        g = edge_gap(mm(t.GetPosition().x), mm(t.GetPosition().y), mm(t.GetWidth()) / 2)
        if g < mg: mg, who = g, "via"
for fp in b.GetFootprints():
    for p in fp.Pads():
        r = max(mm(p.GetSize().x), mm(p.GetSize().y)) / 2
        g = edge_gap(mm(p.GetPosition().x), mm(p.GetPosition().y), r)
        if g < mg: mg, who = g, fp.GetReference()
R["min_copper_to_edge_mm"] = round(mg, 2); R["closest_to_edge"] = who
# ---- 4. zones: islands, antenna strip, fills
zi = []
for z in b.Zones():
    lay = z.GetLayer(); fp_ = z.GetFilledPolysList(lay)
    bbz = z.GetBoundingBox()
    ys = [mm(fp_.CVertex(i).y) for i in range(fp_.FullPointCount())] if fp_.OutlineCount() else []
    zi.append({"layer": z.GetLayerName(), "net": z.GetNetname(), "fill_outlines": fp_.OutlineCount(),
               "fill_min_y_from_top_mm": round(min(ys) - T, 2) if ys else None,
               "island_mode": z.GetIslandRemovalMode(), "min_width": round(mm(z.GetMinThickness()), 2),
               "thermal_gap": round(mm(z.GetThermalReliefGap()), 2), "spoke": round(mm(z.GetThermalReliefSpokeWidth()), 2)})
R["zones"] = zi
strip_y = None
for t in b.GetTracks():
    pos = t.GetStart() if t.GetClass() == "PCB_TRACK" else t.GetPosition()
    if mm(pos.y) - T < 9.0: R.setdefault("copper_items_in_antenna_strip", []).append(t.GetClass())
R.setdefault("copper_items_in_antenna_strip", [])
# ---- 5. silkscreen: stroke, text height, inside board, overlaps
txt = []
for d in b.GetDrawings():
    if d.GetClass() == "PCB_TEXT" and d.GetLayerName().endswith("Silkscreen"):
        txt.append((d.GetText(), d.GetBoundingBox(), mm(d.GetTextHeight()), mm(d.GetTextThickness())))
for fp in b.GetFootprints():
    for t in (fp.Reference(), fp.Value()):
        if t.IsVisible() and t.GetLayerName().endswith("Silkscreen"):
            txt.append((t.GetText(), t.GetBoundingBox(), mm(t.GetTextHeight()), mm(t.GetTextThickness())))
R["silk_text_min_height_mm"] = min(t[2] for t in txt); R["silk_text_min_stroke_mm"] = round(min(t[3] for t in txt), 3)
ov = []
for i in range(len(txt)):
    for j in range(i + 1, len(txt)):
        if txt[i][1].Intersects(txt[j][1]): ov.append((txt[i][0], txt[j][0]))
R["overlapping_silk_texts"] = ov
out = [t[0] for t in txt if mm(t[1].GetX()) < L or mm(t[1].GetRight()) > Rr or mm(t[1].GetY()) < T or mm(t[1].GetBottom()) > B]
R["silk_text_outside_board"] = out
# ---- 6. mask margins, widths
R["mask_margin_mm"] = round(mm(ds.m_SolderMaskExpansion), 3); R["mask_min_width_mm"] = round(mm(ds.m_SolderMaskMinWidth), 3)
tw = sorted({round(mm(t.GetWidth()), 2) for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"})
R["track_widths_mm"] = tw
R["via_sizes"] = sorted({(round(mm(v.GetWidth()), 2), round(mm(v.GetDrill()), 2)) for v in b.GetTracks() if v.GetClass() == "PCB_VIA"})
# ---- 7. footprints: ref/value/orientation; polarity pad1 of electrolytics
R["parts"] = {f.GetReference(): [f.GetValue(), str(f.GetFPID().GetLibItemName()), round(mm(f.GetPosition().x) - L, 1), round(mm(f.GetPosition().y) - T, 1)] for f in b.GetFootprints()}
R["C1_pad1_net"] = b.FindFootprintByReference("C1").FindPadByNumber("1").GetNetname()
R["C3_pad1_net"] = b.FindFootprintByReference("C3").FindPadByNumber("1").GetNetname()
print(json.dumps(R, indent=1, default=str))
