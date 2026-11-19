"""Build 1:1 fit-check sheets (Letter and A4, landscape) from reports/print_1to1.pdf. Print at 100 % / Actual size."""
import pymupdf as fitz
src = fitz.open("reports/print_1to1.pdf")
sp = src[0]
bb = None
for dr in sp.get_drawings():
    bb = dr["rect"] if bb is None else bb | dr["rect"]
pad = 12
clip = fitz.Rect(bb.x0 - pad, bb.y0 - pad, bb.x1 + pad, bb.y1 + pad)
for name, (w, h) in {"letter": (792, 612), "a4": (842, 595)}.items():
    out = fitz.open()
    pg = out.new_page(width=w, height=h)
    x0, y0 = 36, 56
    pg.show_pdf_page(fitz.Rect(x0, y0, x0 + clip.width, y0 + clip.height), src, 0, clip=clip)   # 1:1, no scaling
    pg.insert_text((36, 30), "1:1 fit check. Print at 100% / Actual size (NOT 'Fit' or 'Shrink'). The ruler must measure 30 mm.", fontsize=10)
    out.save("reports/print_1to1_%s.pdf" % name)
    print(name, "board area %.1f x %.1f mm" % (clip.width / 72 * 25.4, clip.height / 72 * 25.4))
