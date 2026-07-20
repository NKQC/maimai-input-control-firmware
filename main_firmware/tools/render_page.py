import sys, fitz  # PyMuPDF

# usage: render_page.py <pdf> <out_prefix> <page1based> [<page1based> ...]
src = sys.argv[1]
prefix = sys.argv[2]
pages = [int(x) for x in sys.argv[3:]]
doc = fitz.open(src)
mat = fitz.Matrix(2.2, 2.2)  # ~2.2x zoom for legibility
for p in pages:
    page = doc.load_page(p - 1)
    pix = page.get_pixmap(matrix=mat)
    out = "%s_p%d.png" % (prefix, p)
    pix.save(out)
    print("wrote", out, pix.width, "x", pix.height)
