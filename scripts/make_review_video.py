r"""make_review_video.py -- one captioned MP4 from the GIFs that `motions.py --gif` writes, for reviewing a policy.

    python scripts/motions.py all --gif --gif-size 480 360 --walk-policy P.onnx --out DIR     # writes DIR/*.gif + motions_all.log
    python scripts/make_review_video.py DIR [--title "V4 policy"] [--compare OTHER_DIR] [--out DIR/review.mp4]

Each motion gets a caption with its name and its PASS/FAIL line from DIR/motions_all.log (or summary.txt). With
--compare, each motion is shown side by side: OTHER_DIR on the left, DIR on the right. Needs imageio + imageio-ffmpeg.
Headless rendering: set MUJOCO_GL=osmesa (and PYOPENGL_PLATFORM=osmesa) for motions.py on a machine without a display.
"""
import argparse
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ORDER = ["balance", "walk", "obstacles", "fall_getup", "sitstand", "roulade", "pick", "dance"]


def verdicts(d):
    out = {}
    for name in ("summary.txt", "motions_all.log"):
        f = Path(d) / name
        if f.exists():
            for line in f.read_text().splitlines():
                parts = line.split(None, 2)
                if len(parts) >= 2 and parts[0] in ORDER:
                    out[parts[0]] = (parts[1], parts[2] if len(parts) > 2 else "")
    return out


def frames(gif, size):
    r = imageio.get_reader(str(gif))
    for f in r:
        im = Image.fromarray(np.asarray(f)[..., :3]).resize(size)
        yield im


def font(sz):
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf"):
        if Path(p).exists():
            return ImageFont.truetype(p, sz)
    return ImageFont.load_default()


def caption(im, top, sub, colour):
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, im.width, 44], fill=(0, 0, 0))
    d.text((10, 4), top, fill=colour, font=font(20))
    d.text((10, 27), sub[:110], fill=(220, 220, 220), font=font(12))
    return im


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("dir"); ap.add_argument("--compare"); ap.add_argument("--title", default="")
    ap.add_argument("--compare-title", default="before"); ap.add_argument("--out")
    ap.add_argument("--fps", type=int, default=25); ap.add_argument("--size", type=int, nargs=2, default=[480, 360])
    a = ap.parse_args(argv)
    d, size = Path(a.dir), tuple(a.size)
    out = Path(a.out or d / "review.mp4")
    v, vc = verdicts(d), (verdicts(a.compare) if a.compare else {})
    w = imageio.get_writer(str(out), fps=a.fps, codec="libx264", quality=7, macro_block_size=8)
    n = 0
    for m in ORDER:
        g = d / f"{m}.gif"
        if not g.exists():
            continue
        st, note = v.get(m, ("", ""))
        right = list(frames(g, size))
        left = list(frames(Path(a.compare) / f"{m}.gif", size)) if a.compare and (Path(a.compare) / f"{m}.gif").exists() else None
        L = max(len(right), len(left) if left else 0)
        for i in range(L):
            r = right[min(i, len(right) - 1)].copy()
            r = caption(r, f"{a.title} {m}: {st}", note, (90, 220, 90) if st == "PASS" else (240, 90, 90))
            if left:
                l = left[min(i, len(left) - 1)].copy()
                cs, cn = vc.get(m, ("", ""))
                l = caption(l, f"{a.compare_title} {m}: {cs}", cn, (90, 220, 90) if cs == "PASS" else (240, 90, 90))
                canvas = Image.new("RGB", (size[0] * 2, size[1])); canvas.paste(l, (0, 0)); canvas.paste(r, (size[0], 0))
                r = canvas
            w.append_data(np.asarray(r)); n += 1
    w.close()
    print(f"wrote {out} ({n} frames, {n / a.fps:.0f} s)")


if __name__ == "__main__":
    main()
