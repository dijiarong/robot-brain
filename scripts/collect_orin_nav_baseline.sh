#!/usr/bin/env bash
# Collect a CPU/memory baseline on the Orin while only the Mid-360 Navigation
# stack is expected to run (no robot-brain WebRTC / ffmpeg).
#
# Usage (on Orin):
#   ./scripts/collect_orin_nav_baseline.sh
#   ./scripts/collect_orin_nav_baseline.sh /tmp/orin-nav-baseline.json
#
# From a laptop (when SSH works):
#   scp scripts/collect_orin_nav_baseline.sh mid360:/tmp/
#   ssh mid360 'bash /tmp/collect_orin_nav_baseline.sh /tmp/orin-nav-baseline.json'
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "error: run this script on the Orin (Linux). On macOS use ssh mid360 …" >&2
  exit 1
fi

OUT="${1:-./docs/evidence/orin-nav-baseline-$(date +%Y%m%d-%H%M%S).json}"
mkdir -p "$(dirname "$OUT")"

have_ffmpeg=false
if pgrep -af '[f]fmpeg' >/dev/null 2>&1; then
  have_ffmpeg=true
fi
have_brain=false
if pgrep -af '[u]vicorn|[r]obot_brain|run_service' >/dev/null 2>&1; then
  have_brain=true
fi
have_webrtc_go2=false
if pgrep -af '[u]nitree_webrtc|webrtc.*9991' >/dev/null 2>&1; then
  have_webrtc_go2=true
fi
have_nav=false
if docker ps --format '{{.Names}}' 2>/dev/null | grep -q navigation \
  || pgrep -af '[n]av2|[l]ivox|[s]uper_lio|[s]port_proxy' >/dev/null 2>&1; then
  have_nav=true
fi

loadavg="$(cat /proc/loadavg)"
mem="$(free -b | awk '/Mem:/{printf "{\"total\":%s,\"used\":%s,\"available\":%s}", $2,$3,$7}')"
nproc="$(nproc)"
uptime_s="$(awk '{print $1}' /proc/uptime)"

# Top CPU consumers (best-effort).
top_cpu="$(ps -eo pid,pcpu,pmem,comm --sort=-pcpu | head -n 12 | tail -n +2 \
  | awk '{printf "{\"pid\":%s,\"cpu\":%s,\"mem\":%s,\"comm\":\"%s\"},", $1,$2,$3,$4}' \
  | sed 's/,$//')"

ok=false
if [[ "$have_ffmpeg" == false && "$have_brain" == false && "$have_webrtc_go2" == false ]]; then
  ok=true
fi

cat >"$OUT" <<EOF
{
  "collected_at": "$(date -Iseconds)",
  "host": "$(hostname)",
  "uptime_seconds": ${uptime_s},
  "nproc": ${nproc},
  "loadavg": "${loadavg}",
  "memory": ${mem},
  "navigation_stack_detected": ${have_nav},
  "ffmpeg_detected": ${have_ffmpeg},
  "robot_brain_detected": ${have_brain},
  "go2_webrtc_detected": ${have_webrtc_go2},
  "expectation": {
    "ffmpeg_detected": false,
    "robot_brain_detected": false,
    "go2_webrtc_detected": false,
    "navigation_stack_detected": true
  },
  "ok": ${ok},
  "top_cpu": [${top_cpu}]
}
EOF

echo "wrote ${OUT}"
python3 -m json.tool "$OUT" | head -n 40

if [[ "$ok" != true ]]; then
  echo "warn: baseline is not Orin-nav-only (ffmpeg / robot-brain / Go2 WebRTC still running)" >&2
  exit 2
fi
exit 0
