"""Copy the libraries MotionTracker uses into its src/ folder and make every include resolvable.

The Arduino IDE compiles a sketch's src/ folder but doesn't put it on the include path, so
includes between the libraries are rewritten to relative paths. See MotionTracker/src/README.md.

usage: python3 tools/vendor_libraries.py <Arduino libraries folder> MotionTracker
"""
import os, re, shutil, sys

libs, sketch = sys.argv[1], sys.argv[2]
dst = os.path.join(sketch, "src")
SRC_EXT = (".h", ".hpp", ".tpp", ".c", ".cpp")

# (library folder, files to take relative to it, destination folder name)
PLAN = [
    ("arduino-lmic",            ["src", "project_config", "LICENSE"], "MCCI_LMIC"),
    ("TinyGPSPlus",             ["src"],                              "TinyGPSPlus"),
    ("XPowersLib",              ["src", "LICENSE"],                   "XPowersLib"),
    ("Adafruit_BME280_Library", ["*", "LICENSE"],                     "Adafruit_BME280"),
    ("Adafruit_Sensor",         ["*", "LICENSE.txt"],                 "Adafruit_Sensor"),
    ("Adafruit_BusIO",          ["*", "LICENSE"],                     "Adafruit_BusIO"),
]

if os.path.isdir(dst):
    shutil.rmtree(dst)
for lib, parts, name in PLAN:
    root = os.path.join(libs, lib)
    out = os.path.join(dst, name)
    os.makedirs(out)
    for part in parts:
        if part == "*":  # top-level source files only
            for f in os.listdir(root):
                if f.endswith(SRC_EXT):
                    shutil.copy2(os.path.join(root, f), out)
        elif os.path.isdir(os.path.join(root, part)):
            shutil.copytree(os.path.join(root, part), os.path.join(out, part),
                            ignore=lambda d, fs: [f for f in fs if os.path.isfile(os.path.join(d, f)) and not f.endswith(SRC_EXT)])
        else:
            shutil.copy2(os.path.join(root, part), out)
    props = os.path.join(root, "library.properties")
    if os.path.exists(props):
        shutil.copy2(props, out)

# index of every vendored header by file name
headers = {}
for d, _, fs in os.walk(dst):
    for f in fs:
        if f.endswith((".h", ".hpp", ".tpp")):
            headers.setdefault(f, []).append(os.path.join(d, f))

inc = re.compile(r'^(\s*#\s*include\s*)([<"])([^>"]+)[>"]', re.M)
changed = 0
for d, _, fs in os.walk(dst):
    for f in fs:
        if not f.endswith(SRC_EXT):
            continue
        path = os.path.join(d, f)
        text = open(path, encoding="utf-8", errors="surrogateescape").read()

        def fix(m):
            global changed
            pre, br, target = m.groups()
            if os.path.exists(os.path.join(d, target)):
                if br == '"':
                    return m.group(0)             # already resolves next to this file
                changed += 1
                return '%s"%s"' % (pre, target)   # <x.h> does not search this file's folder
            cands = [h for h in headers.get(os.path.basename(target), [])
                     if h.replace(os.sep, "/").endswith("/" + target)]
            if len(cands) != 1:
                return m.group(0)                 # system / core header
            changed += 1
            return '%s"%s"' % (pre, os.path.relpath(cands[0], d).replace(os.sep, "/"))

        new = inc.sub(fix, text)
        if new != text:
            open(path, "w", encoding="utf-8", errors="surrogateescape").write(new)
print("rewrote", changed, "includes")
