#!/bin/bash

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BINARY="${ROADPAINTER_AUDIO_TEST_BIN:-$SCRIPT_DIR/../build/audio_strip_test}"

if [ ! -x "$BINARY" ]; then
    echo "audio_strip_test not found: $BINARY" >&2
    echo "Build it with: cmake --build $SCRIPT_DIR/../build --target audio_strip_test" >&2
    exit 1
fi

exec "$BINARY" --wav-dir "$SCRIPT_DIR/wav_files" "$@"
