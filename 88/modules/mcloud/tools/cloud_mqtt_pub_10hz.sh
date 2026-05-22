#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-10005}"
USER_NAME="${USER_NAME:-admin}"
PASSWORD="${PASSWORD:-emqx_2025_Kl}"
VEHICLE_ID="${VEHICLE_ID:-KL001}"
TARGET_NO="${TARGET_NO:-STACKER_A}"
MODE="${MODE:-all}"
BARRIER_INDEX="${BARRIER_INDEX:-J1_E}"
ARM_STATUS="${ARM_STATUS:-1}"
COUNT="${COUNT:-0}"
INTERVAL_SEC="${INTERVAL_SEC:-0.1}"

BROADCAST_TOPIC="vehicle/broadcast/${VEHICLE_ID}"
REPLY_TOPIC="vehicle/reply/${VEHICLE_ID}"
ALL_BARRIERS=("J1_E" "J1_W" "J4_E" "J4_W")

publish_payload() {
  local topic="$1"
  local payload="$2"
  mosquitto_pub -h "${HOST}" -p "${PORT}" -u "${USER_NAME}" -P "${PASSWORD}" \
    -t "${topic}" -m "${payload}" -q 1
}

build_payload() {
  local mode="$1"
  local seq="$2"
  local barrier_index="${3:-${BARRIER_INDEX}}"
  local now_ms
  now_ms="$(date +%s%3N)"

  case "${mode}" in
    broadcast_array)
      cat <<EOF
[
  {
    "vehicleType": 1,
    "vehicleId": "${VEHICLE_ID}",
    "taskType": 2,
    "speed": 2.5,
    "driveMode": 1,
    "lng": 121.4737,
    "lat": 31.2304,
    "heading": 1.57,
    "gear": 1,
    "timestamp": ${now_ms}
  },
  {
    "vehicleType": 2,
    "vehicleId": "${TARGET_NO}",
    "lng": 121.4738,
    "lat": 31.2305,
    "heading": 0.78,
    "speed": 1.2,
    "taskType": 3,
    "timestamp": ${now_ms}
  }
]
EOF
      ;;
    broadcast_notify)
      cat <<EOF
{
  "targetNo": "${TARGET_NO}",
  "operate": 1,
  "timestamp": ${now_ms}
}
EOF
      ;;
    reply_pass)
      cat <<EOF
{
  "msgId": "${TARGET_NO}_P_$(printf '%04d' "${seq}")",
  "status": 2,
  "msg": "accepted",
  "timestamp": ${now_ms}
}
EOF
      ;;
    reply_nonpass)
      cat <<EOF
{
  "msgId": "${TARGET_NO}_X_$(printf '%04d' "${seq}")",
  "status": 1,
  "msg": "received",
  "timestamp": ${now_ms}
}
EOF
      ;;
    barrier)
      cat <<EOF
{
  "boom_barrier_index": "${barrier_index}",
  "arm_status": ${ARM_STATUS},
  "timestamp": ${now_ms}
}
EOF
      ;;
    *)
      echo "[ERROR] unsupported MODE=${mode}" >&2
      echo "[ERROR] valid MODE: all broadcast_array broadcast_notify reply_pass reply_nonpass barrier" >&2
      exit 1
      ;;
  esac
}

resolve_topic() {
  local mode="$1"
  local barrier_index="$2"

  case "${mode}" in
    broadcast_array|broadcast_notify)
      echo "${BROADCAST_TOPIC}"
      ;;
    reply_pass|reply_nonpass)
      echo "${REPLY_TOPIC}"
      ;;
    barrier)
      echo "/boom_barrier/${barrier_index}/status"
      ;;
    *)
      echo ""
      ;;
  esac
}

publish_one_mode() {
  local mode="$1"
  local seq="$2"
  local barrier_index="${3:-${BARRIER_INDEX}}"
  local topic
  local payload
  topic="$(resolve_topic "${mode}" "${barrier_index}")"
  payload="$(build_payload "${mode}" "${seq}" "${barrier_index}")"
  publish_payload "${topic}" "${payload}"
  echo "[PUB] seq=${seq} mode=${mode} topic=${topic}"
}

main() {
  local seq=1

  echo "[INFO] broker=${HOST}:${PORT}"
  echo "[INFO] mode=${MODE} interval=${INTERVAL_SEC}s count=${COUNT}"
  echo "[INFO] vehicle_id=${VEHICLE_ID} target_no=${TARGET_NO} barrier_index=${BARRIER_INDEX} arm_status=${ARM_STATUS}"

  while true; do
    if [[ "all" == "${MODE}" ]]; then
      publish_one_mode "broadcast_array" "${seq}"
      publish_one_mode "broadcast_notify" "${seq}"
      publish_one_mode "reply_pass" "${seq}"
      publish_one_mode "reply_nonpass" "${seq}"
      for barrier_index in "${ALL_BARRIERS[@]}"; do
        publish_one_mode "barrier" "${seq}" "${barrier_index}"
      done
    else
      publish_one_mode "${MODE}" "${seq}"
    fi

    if [[ "${COUNT}" -gt 0 && "${seq}" -ge "${COUNT}" ]]; then
      break
    fi

    seq=$((seq + 1))
    sleep "${INTERVAL_SEC}"
  done
}

main "$@"
