#!/bin/sh
# Bush Drift for iOS: writes ios/BushDrift.xcodeproj from ios/project.yml (needs xcodegen: brew install xcodegen).
set -e
cd "$(dirname "$0")"
# the signing team (Xcode → Settings → Accounts → your team's ID), kept out of the repo in ios/team.local
[ -f team.local ] && BD_IOS_TEAM=$(cat team.local) && export BD_IOS_TEAM
xcodegen generate --spec project.yml --quiet
echo "open ios/BushDrift.xcodeproj"
