#!/usr/bin/env python3
"""
Turn the output of `Motion_viewer --capture <dir>` into the README images:
the PNG screenshots become JPG files and the gif_* frame folders become GIF files (needs ffmpeg).

    python tools/make_readme_media.py <capture dir> [--out image] [--gif-width 640]
"""
import argparse
import os
import subprocess
import sys

from PIL import Image


def main():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("capture")
    ap.add_argument("--out", default=os.path.join(root, "image"))
    ap.add_argument("--gif-width", type=int, default=640)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    for name in sorted(os.listdir(args.capture)):
        src = os.path.join(args.capture, name)
        if name.endswith(".png"):
            dst = os.path.join(args.out, name[:-4] + ".jpg")
            Image.open(src).convert("RGB").save(dst, quality=88, optimize=True)
        elif name.startswith("gif_") and os.path.isdir(src):
            dst = os.path.join(args.out, name[4:] + ".gif")
            graph = ("scale=%d:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=128:stats_mode=diff[p];"
                     "[b][p]paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle" % args.gif_width)
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "20", "-i", os.path.join(src, "%04d.png"),
                            "-vf", graph, "-loop", "0", dst], check=True)
        else:
            continue
        print("%-28s %6.0f KB" % (os.path.basename(dst), os.path.getsize(dst) / 1024))


if __name__ == "__main__":
    sys.exit(main())
