#!/usr/bin/env bash
set -euo pipefail

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-10005}"
USER="${USER_NAME:-admin}"
PASS="${PASSWORD:-emqx_2025_Kl}"
VEHICLE_ID="${1:-KL001}"

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"

BROADCAST_TOPIC="vehicle/broadcast/${VEHICLE_ID}"
REPLY_TOPIC="vehicle/reply/${VEHICLE_ID}"

echo "[INFO] broker=${HOST}:${PORT} vehicle_id=${VEHICLE_ID}"
echo "[INFO] publish to ${BROADCAST_TOPIC} and ${REPLY_TOPIC}"

pub_file() {
  local topic="$1"
  local file="$2"
  echo "[PUB] topic=${topic} file=$(basename "${file}")"
  mosquitto_pub -h "${HOST}" -p "${PORT}" -u "${USER}" -P "${PASS}" \
    -t "${topic}" -f "${file}" -q 1
}

pub_str() {
  local topic="$1"
  local payload="$2"
  echo "[PUB] topic=${topic} payload=${payload}"
  mosquitto_pub -h "${HOST}" -p "${PORT}" -u "${USER}" -P "${PASS}" \
    -t "${topic}" -m "${payload}" -q 1
}

# 1) vehicle/broadcast JSON array, cover vehicleType=1/2/4 branches
pub_file "${BROADCAST_TOPIC}" "${BASE_DIR}/cloud_mqtt_broadcast_array_full.json"

# 2) vehicle/broadcast JSON object, cover borrow/pass response branch
pub_file "${BROADCAST_TOPIC}" "${BASE_DIR}/cloud_mqtt_broadcast_object_notify.json"

# 3) vehicle/reply valid object, cover msgId/status/msg/timestamp + P,status==2 path
pub_file "${REPLY_TOPIC}" "${BASE_DIR}/cloud_mqtt_reply_pass.json"

# 4) vehicle/reply valid object, cover non-pass path
pub_file "${REPLY_TOPIC}" "${BASE_DIR}/cloud_mqtt_reply_nonpass.json"

# 5) vehicle/reply missing fields, cover warning path
pub_file "${REPLY_TOPIC}" "${BASE_DIR}/cloud_mqtt_reply_missing_fields.json"

# 6) vehicle/broadcast malformed JSON, cover parse_error path
pub_str "${BROADCAST_TOPIC}" '{"bad_json": '

# 7) vehicle/broadcast empty payload, cover empty payload path
mosquitto_pub -h "${HOST}" -p "${PORT}" -u "${USER}" -P "${PASS}" \
  -t "${BROADCAST_TOPIC}" -n -q 1

echo "[DONE] test messages sent."
