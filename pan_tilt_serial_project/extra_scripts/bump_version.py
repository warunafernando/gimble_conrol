"""
Pre-build script: read version from version.txt, increment patch (minor) number,
write back, and set APP_VERSION for the build.
"""
Import("env")
import os

PROJECT_DIR = env["PROJECT_DIR"]
VERSION_FILE = os.path.join(PROJECT_DIR, "version.txt")


def read_version():
    if os.path.isfile(VERSION_FILE):
        with open(VERSION_FILE, "r", encoding="utf-8") as f:
            return f.read().strip()
    return "1.0.0"


def bump_patch(ver):
    parts = ver.split(".")
    if len(parts) >= 3:
        try:
            patch = int(parts[2])
            parts[2] = str(patch + 1)
            return ".".join(parts)
        except ValueError:
            pass
    elif len(parts) == 2:
        return ver + ".1"
    return "1.0.1"


def main():
    current = read_version()
    new_ver = bump_patch(current)
    with open(VERSION_FILE, "w", encoding="utf-8") as f:
        f.write(new_ver + "\n")
    # Append compiler flag so we get -D APP_VERSION="1.0.1" (string literal)
    env.Append(BUILD_FLAGS=['-D APP_VERSION=\\"%s\\"' % new_ver])
    print("FW version: %s -> %s" % (current, new_ver))


main()
