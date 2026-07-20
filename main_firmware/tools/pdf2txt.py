import sys, fitz  # PyMuPDF

def convert(src, dst):
    doc = fitz.open(src)
    n = doc.page_count
    with open(dst, 'w', encoding='utf-8') as f:
        for i in range(n):
            page = doc.load_page(i)
            txt = page.get_text("text")
            f.write("\n===== PAGE %d =====\n" % (i + 1))
            f.write(txt)
    print("wrote %s  (%d pages)" % (dst, n))

if __name__ == "__main__":
    convert(sys.argv[1], sys.argv[2])
