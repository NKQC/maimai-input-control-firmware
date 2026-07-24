"""一次性 DXF -> Rust 静态几何生成脚本 (非工程验证部分, 用后即删)。

解析 touch_map_37inch_16x9.dxf 的 SCREEN_FRAME 与 REGION_01..REGION_34
LWPOLYLINE, 按 SCREEN_FRAME 归一化到 0..1000 (x) / 0..(1000*H/W) (y) 的
UI 画布坐标系 (DXF y-up -> UI y-down), 保留 bulge 圆弧 (SVG A 命令),
输出 touch_geometry.rs 到 control_software/src/。
"""
import math
import sys

DXF_PATH = r"F:\maimaicontrol-V3.0\V4 Build\外边构造\touch_map_37inch_16x9.dxf"
OUT_PATH = r"F:\mai2control\mai2control-v4\control_software\src\touch_geometry.rs"

# 34 区几何候选映射 (index 0..33 -> REGION_NN), 与既定架构结论一致:
# A=[R20,R22,R24,R26,R28,R30,R32,R34] B=[R03..R10] C=[R01,R02]
# D=[R19,R21,R23,R25,R27,R29,R31,R33] E=[R18,R11,R12,R13,R14,R15,R16,R17]
ZONE_TO_REGION = (
    [20, 22, 24, 26, 28, 30, 32, 34]   # A1..A8 (idx 0..7)
    + [3, 4, 5, 6, 7, 8, 9, 10]         # B1..B8 (idx 8..15)
    + [1, 2]                           # C1..C2 (idx 16..17)
    + [19, 21, 23, 25, 27, 29, 31, 33]  # D1..D8 (idx 18..25)
    + [18, 11, 12, 13, 14, 15, 16, 17]  # E1..E8 (idx 26..33)
)
assert len(ZONE_TO_REGION) == 34
assert len(set(ZONE_TO_REGION)) == 34


def parse_dxf_groups(path):
    with open(path, "r", encoding="utf-8", errors="strict") as f:
        lines = [ln.rstrip("\n").rstrip("\r") for ln in f]
    groups = []
    i = 0
    while i + 1 < len(lines):
        code = lines[i].strip()
        value = lines[i + 1]
        groups.append((int(code), value))
        i += 2
    return groups


def extract_lwpolylines(groups):
    """返回 {layer_name: [ (verts=[(x,y)], bulges=[b_after_vertex_i or 0.0]) ]}"""
    entities = []
    i = 0
    n = len(groups)
    while i < n:
        code, val = groups[i]
        if code == 0 and val == "LWPOLYLINE":
            layer = None
            verts = []
            bulges = []
            j = i + 1
            cur_x = None
            while j < n:
                c, v = groups[j]
                if c == 0:
                    break
                if c == 8:
                    layer = v
                elif c == 10:
                    cur_x = float(v)
                elif c == 20:
                    verts.append((cur_x, float(v)))
                    bulges.append(0.0)
                elif c == 42:
                    bulges[-1] = float(v)
                j += 1
            entities.append((layer, verts, bulges))
            i = j
        else:
            i += 1
    return entities


def build_arc_command(x0, y0, x1, y1, bulge, sx, sy):
    """把一条 bulge 边转换为 SVG A 命令 (椭圆, 独立 x/y 缩放不引入旋转)。
    返回 (rx, ry, large_arc_flag, sweep_flag, ex, ey) 均已在 UI 坐标系。
    """
    dx = x1 - x0
    dy = y1 - y0
    chord = math.hypot(dx, dy)
    theta = 4.0 * math.atan(bulge)
    r = chord / (2.0 * math.sin(abs(theta) / 2.0))
    rx = r * sx
    ry = r * sy
    large_arc = 1 if abs(theta) > math.pi else 0
    # 见架构推导: DXF y-up 视觉 CCW(b>0) 经 y_ui = H - y 变换后视觉方向不变,
    # 而 SVG sweep-flag=1 表示视觉顺时针 -> b>0 时 sweep_flag=0, b<0 时=1。
    sweep = 0 if bulge > 0 else 1
    ex = x1 * sx
    ey = y1 * sy
    return rx, ry, large_arc, sweep, ex, ey


def region_to_path(verts, bulges, sx, sy):
    """verts/bulges 为原始 DXF mm 坐标 (y-up); sx/sy 把 x/y 独立缩放到 UI 坐标系
    (0..1000 视口, y 已按 y_ui = (H - y_dxf) 的方式在缩放前预处理, 这里 sy 已含负号)。
    返回 (path_cmds, min_x, min_y, max_x, max_y) 均在 UI 坐标系。
    """
    n = len(verts)
    cmds = []
    xs = []
    ys = []

    def to_ui(pt):
        x, y = pt
        return x * sx, y * sy

    x0u, y0u = to_ui(verts[0])
    cmds.append(f"M {x0u:.2f} {y0u:.2f}")
    xs.append(x0u)
    ys.append(y0u)

    for i in range(n):
        j = (i + 1) % n
        x0, y0 = verts[i]
        x1, y1 = verts[j]
        b = bulges[i]
        if abs(b) < 1e-12:
            x1u, y1u = to_ui((x1, y1))
            cmds.append(f"L {x1u:.2f} {y1u:.2f}")
            xs.append(x1u)
            ys.append(y1u)
        else:
            rx, ry, large_arc, sweep, ex, ey = build_arc_command(x0, y0, x1, y1, b, sx, sy)
            cmds.append(f"A {rx:.2f} {ry:.2f} 0 {large_arc} {sweep} {ex:.2f} {ey:.2f}")
            xs.append(ex)
            ys.append(ey)
            # bbox 近似: 采样弧上若干点以获得更准确包围盒 (bulge 很小, 采样 8 点足够)。
            theta = 4.0 * math.atan(b)
            chord = math.hypot(x1 - x0, y1 - y0)
            r = chord / (2.0 * math.sin(abs(theta) / 2.0))
            # 圆心: 用弦中点 + 垂直偏移
            mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            ndx, ndy = -(y1 - y0) / chord, (x1 - x0) / chord
            h = r * math.cos(abs(theta) / 2.0)
            sign = 1.0 if b > 0 else -1.0
            cx = mx + sign * h * ndx
            cy = my + sign * h * ndy
            a0 = math.atan2(y0 - cy, x0 - cx)
            for k in range(1, 8):
                t = a0 + (theta * k / 8.0)
                sx_pt = cx + r * math.cos(t)
                sy_pt = cy + r * math.sin(t)
                pu_x, pu_y = to_ui((sx_pt, sy_pt))
                xs.append(pu_x)
                ys.append(pu_y)
    cmds.append("Z")
    return " ".join(cmds), min(xs), min(ys), max(xs), max(ys)


def _polygon_hit_points(shifted, bulges, to_ui, nseg=28):
    """把一个分区的原始顶点/bulge序列离散成命中检测用多边形点列(UI坐标系)。
    直线边只取顶点; 圆弧边按 nseg 段均匀采样(>=24, 默认28), 保证扇形/环形分区
    的真实弧面参与 point-in-polygon 判定, 而不是退化成弦(避免命中边界误差)。
    """
    n = len(shifted)
    points = []
    for i in range(n):
        x0, y0 = shifted[i]
        points.append(to_ui((x0, y0)))
        j = (i + 1) % n
        x1, y1 = shifted[j]
        b = bulges[i]
        if abs(b) < 1e-12:
            continue
        dx, dy = x1 - x0, y1 - y0
        chord = math.hypot(dx, dy)
        theta = 4.0 * math.atan(b)
        r = chord / (2.0 * math.sin(abs(theta) / 2.0))
        mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        ndx, ndy = -dy / chord, dx / chord
        h = r * math.cos(abs(theta) / 2.0)
        sign = 1.0 if b > 0 else -1.0
        cx = mx + sign * h * ndx
        cy = my + sign * h * ndy
        a0 = math.atan2(y0 - cy, x0 - cx)
        for k in range(1, nseg):
            t = a0 + (theta * k / nseg)
            points.append(to_ui((cx + r * math.cos(t), cy + r * math.sin(t))))
    return points


def main():
    groups = parse_dxf_groups(DXF_PATH)
    entities = extract_lwpolylines(groups)

    frame = None
    regions = {}
    for layer, verts, bulges in entities:
        if layer == "SCREEN_FRAME":
            frame = verts
        elif layer and layer.startswith("REGION_"):
            idx = int(layer.split("_")[1])
            regions[idx] = (verts, bulges)

    if frame is None:
        print("ERROR: SCREEN_FRAME not found", file=sys.stderr)
        sys.exit(1)
    fxs = [p[0] for p in frame]
    fys = [p[1] for p in frame]
    fw = max(fxs) - min(fxs)
    fh = max(fys) - min(fys)
    print(f"SCREEN_FRAME: {fw:.4f} x {fh:.4f} mm", file=sys.stderr)

    if len(regions) != 34:
        print(f"ERROR: expected 34 regions, found {len(regions)}", file=sys.stderr)
        sys.exit(1)

    # UI 画布固定 1000 x (1000*H/W), 保持长宽比与 SCREEN_FRAME 一致 (16:9 近似)。
    ui_w = 1000.0
    ui_h = 1000.0 * fh / fw
    sx = ui_w / fw
    sy_signed = -ui_h / fh  # 负号实现 y-up -> y-down 翻转 (配合下面平移)。

    # 平移: y_ui = (fh - (y_dxf - min_y)) * (ui_h/fh) = ui_h - (y_dxf-min_y)*ui_h/fh
    # 用 to_ui 时先減去 min_x/min_y, 再乘 sx / sy_signed, 再加 ui_h 偏移(因为 sy_signed<0)。
    min_x = min(fxs)
    min_y = min(fys)

    zone_entries = []
    total_bulge_nonzero = 0
    for zone_idx in range(34):
        region_no = ZONE_TO_REGION[zone_idx]
        verts, bulges = regions[region_no]
        shifted = [(x - min_x, y - min_y) for (x, y) in verts]

        def to_ui(pt):
            x, y = pt
            return x * sx, ui_h + y * sy_signed

        # region_to_path 期望内部自己做 to_ui(x*sx,y*sy); 复用需要自定义 sy 偏移,
        # 故这里内联一个专用版本(避免修改通用函数签名破坏 bulge 数学)。
        n = len(shifted)
        cmds = []
        xs = []
        ys = []
        x0u, y0u = to_ui(shifted[0])
        cmds.append(f"M {x0u:.2f} {y0u:.2f}")
        xs.append(x0u)
        ys.append(y0u)
        nonzero_here = 0
        for i in range(n):
            j = (i + 1) % n
            x0, y0 = shifted[i]
            x1, y1 = shifted[j]
            b = bulges[i]
            if abs(b) < 1e-12:
                x1u, y1u = to_ui((x1, y1))
                cmds.append(f"L {x1u:.2f} {y1u:.2f}")
                xs.append(x1u)
                ys.append(y1u)
            else:
                nonzero_here += 1
                total_bulge_nonzero += 1
                dx, dy = x1 - x0, y1 - y0
                chord = math.hypot(dx, dy)
                theta = 4.0 * math.atan(b)
                r = chord / (2.0 * math.sin(abs(theta) / 2.0))
                rx = r * sx
                ry = r * abs(sy_signed)
                large_arc = 1 if abs(theta) > math.pi else 0
                sweep = 0 if b > 0 else 1
                x1u, y1u = to_ui((x1, y1))
                cmds.append(f"A {rx:.2f} {ry:.2f} 0 {large_arc} {sweep} {x1u:.2f} {y1u:.2f}")
                xs.append(x1u)
                ys.append(y1u)
                # 采样弧中点扩展 bbox (bulge 弧凸出弦, 需纳入包围盒)。
                mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
                ndx, ndy = -dy / chord, dx / chord
                h = r * math.cos(abs(theta) / 2.0)
                sign = 1.0 if b > 0 else -1.0
                cx = mx + sign * h * ndx
                cy = my + sign * h * ndy
                a0 = math.atan2(y0 - cy, x0 - cx)
                for k in range(1, 8):
                    t = a0 + (theta * k / 8.0)
                    sample = (cx + r * math.cos(t), cy + r * math.sin(t))
                    pu_x, pu_y = to_ui(sample)
                    xs.append(pu_x)
                    ys.append(pu_y)
        cmds.append("Z")
        path = " ".join(cmds)
        min_px, min_py = min(xs), min(ys)
        max_px, max_py = max(xs), max(ys)
        cx_label = sum(xs[:n if n else 1]) / max(1, len(xs[:n]))
        cy_label = sum(ys[:n if n else 1]) / max(1, len(ys[:n]))
        # 标签点用顶点(非弧采样点)质心, 更贴合区形视觉中心。
        vx = [to_ui(p)[0] for p in shifted]
        vy = [to_ui(p)[1] for p in shifted]
        label_x = sum(vx) / len(vx)
        label_y = sum(vy) / len(vy)
        hit_points = _polygon_hit_points(shifted, bulges, to_ui)

        zone_entries.append({
            "zone_idx": zone_idx,
            "region_no": region_no,
            "path": path,
            "min_x": min_px,
            "min_y": min_py,
            "width": max_px - min_px,
            "height": max_py - min_py,
            "label_x": label_x,
            "label_y": label_y,
            "bulge_count": nonzero_here,
            "hit_points": hit_points,
        })

    print(f"Total non-zero bulges converted to A-commands: {total_bulge_nonzero}", file=sys.stderr)
    for e in zone_entries:
        print(f"zone[{e['zone_idx']:2d}] <- R{e['region_no']:02d} bulges={e['bulge_count']}", file=sys.stderr)

    # 生成 Rust 源
    out = []
    out.append("//! 由 gen_touch_geometry.py 一次性从 touch_map_37inch_16x9.dxf 生成的 34 区静态几何。")
    out.append("//! 不含运行时 DXF 依赖。坐标系: 0..SCREEN_W (x) / 0..SCREEN_H (y), y-down,")
    out.append("//! 与 SCREEN_FRAME (819.1066898mm x 460.7475130mm) 等比例, 保留原始 bulge 圆弧 (SVG A 命令)。")
    out.append("//! region_id 字段保留 DXF REGION_NN 编号供审计, 不参与绑定索引语义。")
    out.append("#![allow(dead_code)]")
    out.append("")
    out.append(f"pub const SCREEN_W: f32 = {ui_w:.4f};")
    out.append(f"pub const SCREEN_H: f32 = {ui_h:.4f};")
    out.append("")
    out.append("/// 单个逻辑分区(0..33)的静态几何: SVG path(M/L/A/Z) + 局部包围盒 + 标签点,")
    out.append("/// 均在 [`SCREEN_W`]x[`SCREEN_H`] 坐标系下。")
    out.append("pub struct ZoneStaticGeometry {")
    out.append("    pub region_id: u8,")
    out.append("    pub path: &'static str,")
    out.append("    pub min_x: f32,")
    out.append("    pub min_y: f32,")
    out.append("    pub width: f32,")
    out.append("    pub height: f32,")
    out.append("    pub label_x: f32,")
    out.append("    pub label_y: f32,")
    out.append("}")
    out.append("")
    out.append("/// 34 区静态几何表, 下标即绑定 index(0..33), 顺序与既定 A/B/C/D/E 映射一致:")
    out.append("/// A=[R20,R22,R24,R26,R28,R30,R32,R34] B=[R03..R10] C=[R01,R02]")
    out.append("/// D=[R19,R21,R23,R25,R27,R29,R31,R33] E=[R18,R11,R12,R13,R14,R15,R16,R17]")
    out.append("pub static ZONE_GEOMETRY: [ZoneStaticGeometry; 34] = [")
    for e in zone_entries:
        out.append("    ZoneStaticGeometry {")
        out.append(f"        region_id: {e['region_no']},")
        out.append(f"        path: \"{e['path']}\",")
        out.append(f"        min_x: {e['min_x']:.2f},")
        out.append(f"        min_y: {e['min_y']:.2f},")
        out.append(f"        width: {e['width']:.2f},")
        out.append(f"        height: {e['height']:.2f},")
        out.append(f"        label_x: {e['label_x']:.2f},")
        out.append(f"        label_y: {e['label_y']:.2f},")
        out.append("    },")
    out.append("];")
    out.append("")

    # 精确命中检测用多边形点表: 圆弧边按 28 段离散, 直线边只保留顶点。
    # 与 ZONE_GEOMETRY 同下标(0..33), 供 hit_test() point-in-polygon 判定,
    # 替代旧的 bbox 命中层(扇形/环形分区 bbox 严重重叠导致误点)。
    out.append("/// 命中检测用多边形点表(与 [`ZONE_GEOMETRY`] 同下标), 圆弧边已离散(>=28段),")
    out.append("/// 供 [`hit_test`] 做精确 point-in-polygon 判定, 取代旧 bbox 命中层。")
    out.append("pub static ZONE_HIT_POINTS: [&[[f32; 2]]; 34] = [")
    for e in zone_entries:
        pts = ", ".join(f"[{x:.2f}, {y:.2f}]" for x, y in e["hit_points"])
        out.append(f"    &[{pts}],")
    out.append("];")
    out.append("")
    out.append("/// 射线法 point-in-polygon: (px,py) 是否在 poly 内(边界视为命中)。")
    out.append("fn _point_in_polygon(px: f32, py: f32, poly: &[[f32; 2]]) -> bool {")
    out.append("    let n = poly.len();")
    out.append("    if n < 3 {")
    out.append("        return false;")
    out.append("    }")
    out.append("    let mut inside = false;")
    out.append("    let mut j = n - 1;")
    out.append("    for i in 0..n {")
    out.append("        let (xi, yi) = (poly[i][0], poly[i][1]);")
    out.append("        let (xj, yj) = (poly[j][0], poly[j][1]);")
    out.append("        if (yi > py) != (yj > py) {")
    out.append("            let x_at_y = xi + (py - yi) / (yj - yi) * (xj - xi);")
    out.append("            if px < x_at_y {")
    out.append("                inside = !inside;")
    out.append("            }")
    out.append("        }")
    out.append("        j = i;")
    out.append("    }")
    out.append("    inside")
    out.append("}")
    out.append("")
    out.append("/// 精确命中检测: (x,y) 为 [`SCREEN_W`]x[`SCREEN_H`] 坐标系下的一点,")
    out.append("/// 返回命中的分区 index(0..33), 未命中返回 `None`。bbox 预筛 + 逐区")
    out.append("/// point-in-polygon, 取代旧的 34 个重叠 bbox TouchArea 命中层。")
    out.append("pub fn hit_test(x: f32, y: f32) -> Option<usize> {")
    out.append("    for i in 0..34usize {")
    out.append("        let g = &ZONE_GEOMETRY[i];")
    out.append("        if x < g.min_x || x > g.min_x + g.width || y < g.min_y || y > g.min_y + g.height {")
    out.append("            continue;")
    out.append("        }")
    out.append("        if _point_in_polygon(x, y, ZONE_HIT_POINTS[i]) {")
    out.append("            return Some(i);")
    out.append("        }")
    out.append("    }")
    out.append("    None")
    out.append("}")
    out.append("")

    with open(OUT_PATH, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"WROTE {OUT_PATH}", file=sys.stderr)


if __name__ == "__main__":
    main()
