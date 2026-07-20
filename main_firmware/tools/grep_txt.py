import sys
path = sys.argv[1]
pats = [p.lower() for p in sys.argv[2:]]
with open(path, 'r', encoding='utf-8', errors='replace') as f:
    lines = f.readlines()
cur_page = 0
hits = 0
for i, ln in enumerate(lines):
    low = ln.lower()
    if ln.startswith('===== PAGE '):
        try:
            cur_page = int(ln.split()[2])
        except Exception:
            pass
    for p in pats:
        if p in low:
            print("L%d (page %d): %s" % (i + 1, cur_page, ln.rstrip()[:200]))
            hits += 1
            break
    if hits > 80:
        print("... (truncated at 80 hits)")
        break
print("TOTAL HITS shown:", hits)
