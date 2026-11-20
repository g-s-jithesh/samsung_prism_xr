#!/bin/sh
# Build the submission zip.
# usage: tools/package.sh <team_name> <estimated_trajectory.txt>
set -e
team=${1:?team name}; traj=${2:?estimated_trajectory.txt}
stage=$(mktemp -d)
mkdir -p "$stage/src"
cp "$traj" "$stage/estimated_trajectory.txt"
cp metadata.json "$stage/"
cp src/main.cpp src/README.md "$stage/src/"
(cd "$stage" && python -m zipfile -c "$OLDPWD/$team.zip" estimated_trajectory.txt metadata.json src)
rm -rf "$stage"
python -m zipfile -l "$team.zip"
