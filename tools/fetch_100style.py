#!/usr/bin/env python3
"""
Download the elderly / assisted walking clips shown by the motion viewer from the 100STYLE
dataset, and write trimmed, compact copies to data/100style.

    python tools/fetch_100style.py [--cache DIR] [--out DIR] [--seconds 40]

100STYLE: Ian Mason, Sebastian Starke, Taku Komura, "Real-Time Style Modelling of Human
Locomotion via Feature-Wise Transformations and Local Motion Phases", 2022.
https://www.ianxmason.com/100style/   License: CC BY 4.0

Changes made to the original files: frames outside Frame_Cuts.csv are removed, the walking
clips are cut to the first `--seconds` seconds, and values are rounded to 0.01.
The skeleton (HIERARCHY section) is copied unchanged.
"""
import argparse
import csv
import os
import sys
import tempfile

import requests

# Google Drive ids from the "Full Dataset Table" of the 100STYLE page
FILES = {
    "Neutral_FW": "1-TqZdIJvpr-QVvAmYJREUZkuAgpAQSfd",
    "Neutral_ID": "1Vnz5wnFmOztNzaSBHjSt9v6y0pl0FK8X",
    "Neutral_TR1": "1AnK7HGtuQR4aSVkyWKnZObgapd41KE1N",
    "Old_FW": "1RrYg0B3WYh7LyOZNX7Ww118DB0WJC3L0",
    "Old_ID": "1UHv8qNSd9om48xDDevMQ6pZGoEuqtX3_",
    "Old_TR1": "1CbByQq6RJWVa9y6SLsEFRn3ARpLWwal3",
    "WalkingStickLeft_FW": "1yLIe2kK90VX5yupO7_swD0Zt94O3YPvN",
    "WalkingStickLeft_ID": "1j-YCc55G_TRhaXc7unK89lFEvpbLGJvQ",
    "WalkingStickLeft_TR1": "11T9_oSmEtq6SFfOys6gFeCSG7NIu7yru",
    "WalkingStickRight_FW": "1dEII7GR_wRd6D1RSkx1kUaeyzXGNJTWJ",
    "WalkingStickRight_ID": "1-9WdhN_1FzHu6DjRIaU_RE5vzhM12Ep-",
    "WalkingStickRight_TR1": "1XVMV67l-q1UsDGrKEqw080EVyHYRC8_6",
    "LimpLeft_FW": "1iE3iuUXpBAdZ_RhQ8bt2GMik6sGyE3yc",
    "LimpLeft_ID": "16SL_8GSs8wUuYQcLPteqq2dJE4r-8Hl_",
    "LimpLeft_TR1": "1yfQLHX7alhph03Ta8mJ-rDrcUpcTFMMh",
    "LimpRight_FW": "1fdpBjAYy8Xo5gFHHk8Ekm0gWq7_UoPMv",
    "LimpRight_ID": "1_8pgqndN6Ckt4RCNHOlY5pXNQB6J-K1M",
    "LimpRight_TR1": "1k3wBCkfyDZ_U8nqg-2uol1CJbDDPznOu",
}
FRAME_CUTS = "1d0VM8k4UjA4dDmaviuZMjAf-WQNLUwnZ"


def download(file_id, path):
    if os.path.exists(path) and os.path.getsize(path) > 0:
        return
    url = "https://drive.usercontent.google.com/download"
    with requests.get(url, params={"id": file_id, "export": "download"}, stream=True, timeout=60) as r:
        r.raise_for_status()
        if "text/html" in r.headers.get("Content-Type", ""):
            raise RuntimeError("Google Drive returned a web page instead of " + os.path.basename(path))
        tmp = path + ".part"
        with open(tmp, "wb") as f:
            for chunk in r.iter_content(1 << 20):
                f.write(chunk)
        os.replace(tmp, path)


def number(v):
    s = "%.2f" % v
    s = s.rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def compact(src, dst, first, last):
    with open(src) as f:
        text = f.read()
    header, motion = text.split("MOTION", 1)
    lines = motion.strip().splitlines()
    frame_time = lines[1].split(":")[1].strip()
    frames = lines[2:][first:last]
    with open(dst, "w", newline="\n") as f:
        # delfem2 cannot read empty lines, so keep the header tight
        f.write("\n".join(l.rstrip() for l in header.strip().splitlines() if l.strip()) + "\n")
        f.write("MOTION\nFrames: %d\nFrame Time: %s\n" % (len(frames), frame_time))
        for l in frames:
            f.write(" ".join(number(float(v)) for v in l.split()) + "\n")
    return len(frames)


def main():
    root = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cache", default=os.path.join(tempfile.gettempdir(), "100style"),
                    help="where the original files are downloaded")
    ap.add_argument("--out", default=os.path.join(root, "data", "100style"))
    ap.add_argument("--seconds", type=float, default=40.0, help="length kept from the walking clips")
    args = ap.parse_args()
    os.makedirs(args.cache, exist_ok=True)
    os.makedirs(args.out, exist_ok=True)

    download(FRAME_CUTS, os.path.join(args.cache, "Frame_Cuts.csv"))
    with open(os.path.join(args.cache, "Frame_Cuts.csv")) as f:
        cuts = {row["STYLE_NAME"]: row for row in csv.DictReader(f)}
    for i, (name, file_id) in enumerate(FILES.items()):
        src = os.path.join(args.cache, name + ".bvh")
        download(file_id, src)
        style, kind = name.split("_")
        first, last = int(cuts[style][kind + "_START"]), int(cuts[style][kind + "_STOP"])
        if kind != "ID":
            last = min(last, first + int(args.seconds * 60))
        n = compact(src, os.path.join(args.out, name + ".bvh"), first, last)
        print("[%2d/%d] %-22s %5d frames (%.1f s)" % (i + 1, len(FILES), name, n, n / 60.0), flush=True)


if __name__ == "__main__":
    sys.exit(main())
