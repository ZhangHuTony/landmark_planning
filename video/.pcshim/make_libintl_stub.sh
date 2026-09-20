#!/usr/bin/env bash
# Rebuild the libintl link stub.
#
# Julia's Glib_jll advertises -lintl in glib-2.0.pc, so the linker demands a
# file called libintl.so exists. Nothing actually calls into it: glibc provides
# gettext, dcgettext, bindtextdomain and friends itself, and at run time the
# real Gettext_jll libintl.so.8 is on LD_LIBRARY_PATH (see env.sh). An empty
# shared object is therefore enough to satisfy the link.
set -euo pipefail
echo "" | gcc -shared -o "$(dirname "$0")/lib/libintl.so" -xc -
echo "rebuilt $(dirname "$0")/lib/libintl.so"
