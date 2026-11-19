#!/usr/bin/env bash
# New handoff helper, not the author's historical build.sh.
# Requires installed C++ dependencies and a compatible Pangolin configuration.
set -euo pipefail
orb_source="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/source/ORB_SLAM3" && pwd)"
orb_jobs="${ORB_BUILD_JOBS:-2}"
if ! [[ "$orb_jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "ORB_BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
test -s "$orb_source/Vocabulary/ORBvoc.txt"
for dependency in DBoW2 g2o Sophus; do
  cmake -S "$orb_source/Thirdparty/$dependency" \
    -B "$orb_source/Thirdparty/$dependency/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$orb_source/Thirdparty/$dependency/build" --parallel "$orb_jobs"
done
cmake -S "$orb_source" -B "$orb_source/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$orb_source/build" --target mono_tum --parallel "$orb_jobs"
echo "Built executable: $orb_source/Examples/Monocular/mono_tum"
