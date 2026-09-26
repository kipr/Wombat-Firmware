#!/usr/bin/env bash
# Checks that the firmware's SPI register map matches libwallaby's copy.
#
# The register indices are the protocol contract between this firmware and
# libwallaby on the Wombat's Raspberry Pi. If the two copies disagree, the host
# reads and writes the wrong registers.
#
# Usage:
#   scripts/check-register-map.sh                     # compare with libwallaby master on GitHub
#   scripts/check-register-map.sh path/to/libwallaby  # compare with a local checkout

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
libwallaby_header="module/core/protected/kipr/core/registers.hpp"

# wallaby.h includes the register-map revision that is actually compiled in.
fw_header_name="$(tr -d '\r' < "$repo_root/Firmware/include/wallaby.h" \
  | sed -nE 's/^[[:space:]]*#include[[:space:]]+"(wallaby_spi_r[0-9]+\.h)".*/\1/p' | head -n 1)"
if [ -z "$fw_header_name" ]; then
  echo "error: no wallaby_spi_r*.h include found in Firmware/include/wallaby.h" >&2
  exit 2
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

if [ $# -ge 1 ]; then
  lw_source="$1/$libwallaby_header"
  if [ ! -f "$lw_source" ]; then
    echo "error: $lw_source not found; pass the root of a libwallaby checkout" >&2
    exit 2
  fi
  cp "$lw_source" "$tmp/registers.hpp"
else
  lw_source="https://raw.githubusercontent.com/kipr/libwallaby/master/$libwallaby_header"
  curl -fsSL "$lw_source" -o "$tmp/registers.hpp"
fi

# Prints "NAME VALUE" for each register define, ignoring line endings and layout.
extract() {
  tr -d '\r' < "$1" \
    | awk '$1 == "#define" && ($2 ~ /^REG_/ || $2 == "WALLABY_SPI_VERSION") { print $2, $3 }' \
    | sort
}

extract "$repo_root/Firmware/include/$fw_header_name" > "$tmp/firmware.txt"
extract "$tmp/registers.hpp" > "$tmp/libwallaby.txt"

if [ ! -s "$tmp/firmware.txt" ]; then
  echo "error: no register defines found in $fw_header_name" >&2
  exit 2
fi

if diff -u --label "firmware ($fw_header_name)" --label "libwallaby ($lw_source)" \
     "$tmp/firmware.txt" "$tmp/libwallaby.txt"; then
  echo "OK: $(wc -l < "$tmp/firmware.txt" | tr -d ' ') register defines match ($fw_header_name vs libwallaby)"
else
  echo "Register maps differ. Protocol changes need matching changes in both repos; see AGENTS.md." >&2
  exit 1
fi
