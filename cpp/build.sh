#!/usr/bin/env bash
set -euo pipefail

# Simple CMake build helper for wt901_reader
# Usage examples:
#   ./build.sh                 # build Release
#   ./build.sh -d              # build Debug
#   ./build.sh -c              # clean only
#   ./build.sh -r              # rebuild (clean-first)
#   ./build.sh -j 4            # build with 4 jobs
#   ./build.sh --run "--port /dev/ttyCH341USB0 --baud 115200 --addr 0x50"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Use relative build directory for portability
BUILD_DIR="build"
BUILD_TYPE="Release"
JOBS=""
DO_CLEAN=false
CLEAN_FIRST=false
RUN_AFTER=false
RUN_ARGS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--clean)
      DO_CLEAN=true
      shift
      ;;
    -r|--rebuild|--clean-first)
      CLEAN_FIRST=true
      shift
      ;;
    -d|--debug)
      BUILD_TYPE="Debug"
      shift
      ;;
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    -j|--jobs)
      JOBS="$2"
      shift 2
      ;;
    --run)
      RUN_AFTER=true
      RUN_ARGS="${2:-}"
      shift 2
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

mkdir -p "${BUILD_DIR}"

if ${DO_CLEAN}; then
  if [[ -f "${BUILD_DIR}/build.ninja" ]]; then
    ninja -C "${BUILD_DIR}" clean || true
  elif [[ -f "${BUILD_DIR}/Makefile" ]]; then
    make -C "${BUILD_DIR}" clean || true
  else
    rm -rf "${BUILD_DIR}"/* || true
  fi
  # If only cleaning, exit now
  if ! ${CLEAN_FIRST}; then
    echo "Clean complete."
    exit 0
  fi
fi

# Configure using relative source (.) and build (build/) dirs
cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

BUILD_CMD=(cmake --build "${BUILD_DIR}")
if ${CLEAN_FIRST}; then
  BUILD_CMD+=(--clean-first)
fi
if [[ -n "${JOBS}" ]]; then
  BUILD_CMD+=(-j "${JOBS}")
fi

"${BUILD_CMD[@]}"

echo "Build complete: ${BUILD_TYPE}"

if ${RUN_AFTER}; then
  "./${BUILD_DIR}/wt901_reader" ${RUN_ARGS}
fi


