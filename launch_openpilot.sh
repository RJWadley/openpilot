#!/usr/bin/env bash
# IQ.Pilot GitHub bootstrap.
#
# Installers like installer.comma.ai/<user>/<branch> clone this branch from GitHub and boot it through
# /data/continue.sh -> launch_openpilot.sh. Instead of starting openpilot, we fetch the IQ.Pilot installer
# for the same channel and let it reinstall the device from git.konn3kt.com, so updates never touch GitHub.

INSTALLER_HOST="https://installer.iqlvbs.com"
FALLBACK_CHANNEL="release"
INSTALLER="/tmp/iq_installer"
CONTINUE="/data/continue.sh"
CONTINUE_BACKUP="/data/continue.sh.iq-bootstrap"
RETRY_SECONDS=10

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

# The GitHub branch picks the IQ.Pilot channel; a branch with no matching channel installs FALLBACK_CHANNEL.
channel="$(git -C "$DIR" rev-parse --abbrev-ref HEAD 2>/dev/null)"
[ -n "$channel" ] && [ "$channel" != "HEAD" ] || channel="$FALLBACK_CHANNEL"

# Same headers AGNOS setup sends when it downloads a custom software installer.
device_type="$({ tr -d '\0' < /sys/firmware/devicetree/base/model; } 2>/dev/null | awk '{print $NF}')"
serial="$(sed -n 's/.*androidboot.serialno=\([^ ]*\).*/\1/p' /proc/cmdline 2>/dev/null)"
os_version="$(cat /VERSION 2>/dev/null)"

download_installer() {
  local status
  status="$(curl -sS -o "$INSTALLER.tmp" -w '%{http_code}' --connect-timeout 10 --max-time 120 \
    -A "AGNOSSetup-$os_version" \
    -H "X-openpilot-serial: $serial" \
    -H "X-openpilot-device-type: $device_type" \
    "$INSTALLER_HOST/$channel")"
  if [ "$status" = "200" ]; then
    chmod +x "$INSTALLER.tmp" && mv "$INSTALLER.tmp" "$INSTALLER"
    return 0
  fi
  rm -f "$INSTALLER.tmp"
  # installer.iqlvbs.com answers 409 for a channel that doesn't exist.
  if [ "$status" = "409" ] && [ "$channel" != "$FALLBACK_CHANNEL" ]; then
    echo "iq-bootstrap: no IQ.Pilot channel '$channel', using '$FALLBACK_CHANNEL'"
    channel="$FALLBACK_CHANNEL"
  else
    echo "iq-bootstrap: installer download for '$channel' failed (HTTP ${status:-none})"
  fi
  return 1
}

run_installer() {
  # The installer skips straight to "Finishing install..." while continue.sh exists. Move it aside, and
  # put it back if the install fails so the next boot retries instead of falling back to AGNOS setup.
  [ -f "$CONTINUE" ] && mv -f "$CONTINUE" "$CONTINUE_BACKUP"
  "$INSTALLER" &
  local pid=$!

  # On success the installer swaps in /data/openpilot, writes a fresh continue.sh, then idles on
  # "Finishing install..." for a minute waiting for the new software to take over. Take over right away.
  while kill -0 "$pid" 2>/dev/null; do
    if [ -f "$CONTINUE" ]; then
      kill "$pid" 2>/dev/null
      wait "$pid" 2>/dev/null
      rm -f "$CONTINUE_BACKUP"
      return 0
    fi
    sleep 1
  done
  wait "$pid" 2>/dev/null

  if [ -f "$CONTINUE" ]; then
    rm -f "$CONTINUE_BACKUP"
    return 0
  fi
  echo "iq-bootstrap: installer exited without finishing the install"
  [ -f "$CONTINUE_BACKUP" ] && mv -f "$CONTINUE_BACKUP" "$CONTINUE"
  return 1
}

while true; do
  if download_installer && run_installer; then
    echo "iq-bootstrap: IQ.Pilot installed, launching"
    # Our checkout was just replaced, so leave its deleted directory before handing over.
    cd /
    exec "$CONTINUE"
  fi
  sleep "$RETRY_SECONDS"
done
