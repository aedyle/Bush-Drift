#!/bin/sh
# Publish the browser build to GitHub Pages (session 3b): build, put the play-test page in front of the game
# (index.html = web/playtest.html, play.html = the game) and push it all to the gh-pages branch.
set -e
cd "$(dirname "$0")/.."
./web/build.sh >/dev/null
build="$(date +%Y-%m-%d) $(git rev-parse --short HEAD)"
site=$(mktemp -d)
cp web/out/index.js web/out/index.wasm web/out/index.data "$site/"
cp web/out/index.html "$site/play.html"
sed "s/{{BUILD}}/$build/" web/playtest.html > "$site/index.html"
cp web/playtest/start.png web/playtest/drift.png "$site/"
touch "$site/.nojekyll"
remote=$(git remote get-url origin)
cd "$site"
git init -q -b gh-pages
git add -A
git -c user.name="$(git -C "$OLDPWD" config user.name)" -c user.email="$(git -C "$OLDPWD" config user.email)" commit -q -m "Play-test build $build"
git push -q -f "$remote" gh-pages
echo "published build $build"
rm -rf "$site"
