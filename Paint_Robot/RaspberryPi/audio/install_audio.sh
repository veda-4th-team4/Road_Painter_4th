#!/bin/bash

set -euo pipefail

if [ "${EUID}" -ne 0 ]; then
    echo "Run as root: sudo ./install_audio.sh" >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RPI_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
DRIVER_DIR="$RPI_DIR/driver"
KERNEL_RELEASE="$(uname -r)"
MODULE_DIR="/lib/modules/$KERNEL_RELEASE/extra"
BOOT_CONFIG="/boot/firmware/config.txt"
[ -f "$BOOT_CONFIG" ] || BOOT_CONFIG="/boot/config.txt"

if [ ! -f "$BOOT_CONFIG" ]; then
    echo "Raspberry Pi boot config not found" >&2
    exit 1
fi

for command_name in make dtc; do
    if ! command -v "$command_name" >/dev/null 2>&1; then
        echo "Required command not found: $command_name" >&2
        exit 1
    fi
done
if [ ! -d "/lib/modules/$KERNEL_RELEASE/build" ]; then
    echo "Kernel headers not found for $KERNEL_RELEASE" >&2
    exit 1
fi

echo "[1/6] Building kernel module and device-tree overlay"
make -C "$DRIVER_DIR" audio

echo "[2/6] Installing module and overlay"
install -d -m 0755 "$MODULE_DIR" /boot/overlays
install -m 0644 "$DRIVER_DIR/audio_strip_driver.ko" "$MODULE_DIR/"
install -m 0644 "$DRIVER_DIR/audio_strip_overlay.dtbo" /boot/overlays/
depmod -a
printf '%s\n' audio_strip_driver > /etc/modules-load.d/audio-strip.conf

echo "[3/6] Updating boot configuration"
BACKUP="$BOOT_CONFIG.road-painter-audio.bak"
if [ ! -f "$BACKUP" ]; then
    cp -a "$BOOT_CONFIG" "$BACKUP"
fi
# The raw driver and ALSA hifiberry-dac both claim the same I2S controller.
sed -i 's/^[[:space:]]*dtoverlay=hifiberry-dac/# disabled by road-painter audio: &/' "$BOOT_CONFIG"
sed -i 's/^[[:space:]]*dtparam=audio=on/dtparam=audio=off/' "$BOOT_CONFIG"
grep -q '^dtparam=audio=off' "$BOOT_CONFIG" || printf '%s\n' 'dtparam=audio=off' >> "$BOOT_CONFIG"
grep -q '^dtparam=i2s=on' "$BOOT_CONFIG" || printf '%s\n' 'dtparam=i2s=on' >> "$BOOT_CONFIG"
grep -q '^dtoverlay=audio_strip_overlay' "$BOOT_CONFIG" || \
    printf '%s\n' 'dtoverlay=audio_strip_overlay' >> "$BOOT_CONFIG"

echo "[4/6] Installing device permissions"
printf '%s\n' 'KERNEL=="audio_strip", MODE="0660", GROUP="audio"' \
    > /etc/udev/rules.d/99-audio-strip.rules
udevadm control --reload-rules

echo "[5/6] Installing WAV catalog"
install -d -m 0755 /opt/road-painter/audio/wav_files
install -m 0644 "$SCRIPT_DIR"/wav_files/*.wav /opt/road-painter/audio/wav_files/
cat > /etc/default/road-painter-audio <<'EOF'
ROADPAINTER_AUDIO_DIR=/opt/road-painter/audio/wav_files
EOF
chmod 0644 /etc/default/road-painter-audio

echo "[6/6] Verifying installed files"
modinfo "$MODULE_DIR/audio_strip_driver.ko" >/dev/null
for wav in "$SCRIPT_DIR"/wav_files/*.wav; do
    test -r "/opt/road-painter/audio/wav_files/$(basename "$wav")"
done

echo "Installation files are ready. Reboot is required to switch from ALSA"
echo "hifiberry-dac to audio_strip_overlay. The previous boot config is at:"
echo "  $BACKUP"
echo "After reboot, verify with:"
echo "  ls -l /dev/audio_strip"
echo "  dmesg | grep audio_strip"
echo "  $RPI_DIR/build/audio_strip_test task_start"
