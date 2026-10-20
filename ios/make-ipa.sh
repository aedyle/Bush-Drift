#!/bin/sh
# Bush Drift for iOS: an unsigned BushDrift.ipa in ios/out/, for testers who sideload it with their own Apple ID
# (Sideloadly or AltStore sign it on their side). See ios/SIDELOADING.md.
set -e
cd "$(dirname "$0")"
xcodebuild -project BushDrift.xcodeproj -scheme BushDrift -destination 'generic/platform=iOS' -configuration Release \
    -derivedDataPath build CODE_SIGNING_ALLOWED=NO build 2>&1 | grep -E "error:|BUILD (SUCCEEDED|FAILED)" | sort -u
rm -rf out/Payload && mkdir -p out/Payload
cp -R build/Build/Products/Release-iphoneos/BushDrift.app out/Payload/
(cd out && rm -f BushDrift.ipa && zip -qry BushDrift.ipa Payload && rm -rf Payload)
ls -la out/BushDrift.ipa
