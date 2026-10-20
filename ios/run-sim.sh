#!/bin/sh
# Bush Drift for iOS: build, install and start in a simulator (default: the first booted one), then print the
# app's console log after a few seconds. Usage: ./ios/run-sim.sh [simulator id] [seconds]
set -e
cd "$(dirname "$0")"
SIM=${1:-booted}; WAIT=${2:-10}
xcodebuild -project BushDrift.xcodeproj -scheme BushDrift -destination "platform=iOS Simulator,id=$(xcrun simctl list devices booted | grep -oE '[0-9A-F-]{36}' | head -1)" \
    -configuration Debug -derivedDataPath build CODE_SIGNING_ALLOWED=NO build 2>&1 | grep -E "error:|BUILD (SUCCEEDED|FAILED)" | sort -u
xcrun simctl install "$SIM" build/Build/Products/Debug-iphonesimulator/BushDrift.app
xcrun simctl terminate "$SIM" io.github.aedyle.bushdrift 2>/dev/null || true
xcrun simctl launch "$SIM" io.github.aedyle.bushdrift >/dev/null
sleep "$WAIT"
cat "$(xcrun simctl get_app_container "$SIM" io.github.aedyle.bushdrift data)/Library/Application Support/aedyle/Bush Drift/console.log"
