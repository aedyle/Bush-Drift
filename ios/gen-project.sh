#!/bin/sh
# Bush Drift for iOS: writes ios/BushDrift.xcodeproj from ios/project.yml (needs xcodegen: brew install xcodegen).
set -e
cd "$(dirname "$0")"
xcodegen generate --spec project.yml --quiet
echo "open ios/BushDrift.xcodeproj"
