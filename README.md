# ESP32 + CC1101 carrier board

A 95 x 55 mm two-layer KiCad carrier that plugs a DOIT ESP32 DevKit V1 (30-pin) and a CC1101 433 MHz breakout (2x4 header, edge-launch SMA) together over SPI, with a matching 3D-printed case. It is not tied to any one firmware: use it for ESPHome, Arduino, MicroPython or anything else that talks to a CC1101 over SPI.


Open `esp32_cc1101_carrier.kicad_pro` in KiCad 10 (rev B files are saved in KiCad 10 format; rev A was KiCad 7).
Project-local libraries are in `libs/` (VictorLamoine ESP32_DevKit_V1_DOIT symbol + footprint + STEP, BSD-with-attribution, see libs/VL_LICENSE.txt).
`build.py` regenerates everything (run with the python that has `pcbnew`; works with KiCad 7 and 10). On Windows set `KICAD_FP_DIR` / `KICAD_SYM_DIR` to KiCad's `share/kicad/footprints` and `share/kicad/symbols`.

| Signal | ESP32 GPIO | CC1101 header pin |
|---|---|---|
| SCK  | 18 | 5 |
| MOSI | 23 | 6 |
| MISO | 19 | 7 |
| CSN  | 22 | 4 |
| GDO0 | 21 | 3 |
| VCC  | 3V3 | 2 |
| GND  | GND | 1 |
GDO2 (pin 8) -> GPIO4 (not a strapping pin). None of the GPIOs used are strapping pins, so boot behaviour is unaffected.

Example ESPHome SPI/CC1101 pin settings: `clk_pin: GPIO18`, `mosi_pin: GPIO23`, `miso_pin: GPIO19`, `cs_pin: GPIO22`, `gdo0_pin: GPIO21`, `gdo2_pin: GPIO4`. The pin map is set at the top of `build.py` if you need a different one for another project.

Fab files: `fab/` (Gerbers + separate PTH/NPTH drill files, 2 layers, 95 x 55 mm, 1.6 mm).

## Capacitors
All through-hole and optional: populate only what you need.

| Ref | Value | Type | Rail | Size | Location |
|---|---|---|---|---|---|
| C1 | 470 µF, 16 V | Aluminium electrolytic, radial | VIN | D8 mm, 3.5 mm pitch, about 12.5 mm tall | Left of the ESP32 |
| C2 | 100 nF, 50 V | Ceramic (MLCC), X7R | VIN | 5 mm lead pitch (disc up to 5 x 2.5 mm) | Left of the ESP32 |
| C3 | 22 µF, 16 V | Aluminium electrolytic, radial | 3V3 | D5 mm, 2 mm pitch, about 11.5 mm tall | Above the CC1101 header |
| C4 | 100 nF, 50 V | Ceramic (MLCC), X7R | 3V3 | 5 mm lead pitch (disc up to 5 x 2.5 mm) | Above the CC1101 header, by its VCC pin |

- Electrolytics are polarised: the square pad (pad 1) is + (VIN or 3V3).
- C3 can be anything from 10 to 47 µF, as long as it is D5 mm or smaller with 2 mm pitch; a wider can would reach under the CC1101 module. Heights above are what the case model assumes (the lid is 18 mm above the board).
- Fit C1 when powering from VIN through long wires or a battery. On USB power it isn't needed. 470 µF is well over the USB inrush limit (10 µF), which can occasionally upset a strict port or hub at plug-in. If you want it fitted on USB anyway, a smaller value (47–100 µF) is a reasonable compromise.

## Layout notes
- Bottom copper (B.Cu) is a solid GND plane; top copper (F.Cu) carries all SPI signals plus a GND pour, tied together with stitching vias.
- Signals stay on F.Cu except two short hops to B.Cu: SCK (about 4.6 mm) and +3V3 (about 8.9 mm), each through one via pair. The rest of B.Cu is the ground plane.
- The top 9.2 mm of the board (above the ESP32 pin rows, full width) has no ground pour on either layer. One MOSI trace (GPIO23 is the top pin of the right-hand row) runs through that strip for about 22 mm.
- Silkscreen: strokes 0.2 mm, text >= 1.0 mm (JLCPCB DFM rates 0.15 mm only as a warning). Solder-mask expansion 0.05 mm.

## Rev B (current)
- C3 (22 uF, 3V3 bulk cap) moved 2.5 mm toward the top of the board (y 17.5 -> 15.0 mm). Its body now clears the CC1101 breakout's edge by about 0.8 mm (module edge estimated from photos, +/-0.5 mm), so it can stand upright instead of being bent over. Its 3V3 trace is about 2 mm longer; nothing else in the routing changed.
- Silkscreen and title block read rev B.

## Rev A: layout refinement
45° corners, ground stitching along traces and edge, vias ≥2.2 mm from pins, thermal-relief settings, 1.0 mm header holes. See `audit-report.md`, `reports/print_1to1_letter.pdf` / `print_1to1_a4.pdf` for a 1:1 fit check (30 mm ruler; print at Actual size).
