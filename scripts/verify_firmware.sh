#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

main_file="src/main.cpp"
ini_file="platformio.ini"

echo "[1/3] Checking merge conflict markers..."
if rg -n "^<<<<<<<|^=======|^>>>>>>>" "$main_file"; then
  echo "ERROR: merge conflict markers found in $main_file"
  exit 1
fi

echo "[2/3] Checking required includes in $main_file..."
required_includes=(
  "#include <esp_now.h>"
  "#include <esp_wifi.h>"
  "#include <DHT.h>"
  "#include <SPI.h>"
  "#include <U8g2lib.h>"
)

for include in "${required_includes[@]}"; do
  if ! rg -Fq "$include" "$main_file"; then
    echo "ERROR: missing include '$include' in $main_file"
    exit 1
  fi
done

echo "[3/3] Checking PlatformIO dependencies..."
required_deps=(
  "olikraus/U8g2"
  "adafruit/DHT sensor library"
  "adafruit/Adafruit Unified Sensor"
  "bblanchon/ArduinoJson"
)
for dep in "${required_deps[@]}"; do
  if ! rg -Fq "$dep" "$ini_file"; then
    echo "ERROR: missing dependency '$dep' in $ini_file"
    exit 1
  fi
done

echo "OK: static firmware verification passed."
