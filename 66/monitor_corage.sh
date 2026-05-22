#!/bin/bash

# Global Configuration
LOG_DIR="/century/data/log"
LOG_FILE="$LOG_DIR/monitor_corage_rate.log"
TIMESTAMP_LOG="$LOG_DIR/timestamps.log"
SSH_PASS="123"
REMOTE_USER="corage"
REMOTE_IP="192.168.1.88"
REMOTE_SCRIPT="/home/corage/workspace/product/start.sh"

init_log() {
    mkdir -p "$LOG_DIR"
    touch "$LOG_FILE" "$TIMESTAMP_LOG"
    chmod 644 "$LOG_FILE" "$TIMESTAMP_LOG"
    echo "===== SYSTEM START $(date '+%Y-%m-%d %H:%M:%S.%3N') =====" >> "$LOG_FILE"
}

log_with_timestamp() {
    local message=$1
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S.%3N')
    echo "[$timestamp] $message" >> "$TIMESTAMP_LOG"
}

monitor_corage_rate() {
    local topic="$1"
    local timeout=${2:-3}

    log_with_timestamp "BEGIN monitoring topic: $topic"

    local output
    output=$(timeout "${timeout}s" cyber_channel hz "$topic" 2>&1)
    local rate
    rate=$(echo "$output" | grep -oE 'average rate: [0-9.]+' | tail -1 | awk '{print $NF}')

    log_with_timestamp "GOT rate for $topic: ${rate:-NULL}"

    {
        echo "[$(date '+%H:%M:%S.%3N')] Topic: $topic"
        echo "[$(date '+%H:%M:%S.%3N')] Raw Output: $output"
        echo "[$(date '+%H:%M:%S.%3N')] Parsed Rate: ${rate:-NULL}"
    } >> "$LOG_FILE"

    if [[ -z "$rate" ]]; then
        log_with_timestamp "WARNING: No valid frequency detected for $topic"
        echo "[$(date '+%H:%M:%S.%3N')] WARNING: Restarting corage module..." >> "$LOG_FILE"
        
        if ! docker start bridge-ros2; then
            log_with_timestamp "ERROR: Local bridge-ros2 start failed"
            return 1
        else
            log_with_timestamp "SUCCESS: Local bridge-ros2 started"
        fi

        if ! sshpass -p "$SSH_PASS" ssh -t "${REMOTE_USER}@${REMOTE_IP}" '
            source ~/.bashrc
            cd "$(dirname "'"$REMOTE_SCRIPT"'")"
            echo '"$SSH_PASS"' | sudo -S bash -c "source /etc/profile; ./'"$(basename "$REMOTE_SCRIPT")"'"
        ' >> "$LOG_FILE" 2>&1; then
            log_with_timestamp "ERROR: Remote restart failed"
            return 2
        else
            log_with_timestamp "SUCCESS: Remote module restarted"
        fi

        log_with_timestamp "ACTION: Sleeping 20s after restart"
        sleep 20
    else
        log_with_timestamp "INFO: Normal rate detected: $rate Hz"
    fi
}

main() {
    init_log
    log_with_timestamp "MAIN: Monitoring started"

    while true; do
        log_with_timestamp "CHECK: Process count verification"
        if [ $(ps -ef | grep -v grep | grep -e mainboard -e bridge | wc -l) -lt 3 ]; then
            log_with_timestamp "WARNING: Essential processes < 3"
            continue
        fi
        
        log_with_timestamp "CYCLE: New monitoring cycle started"
        monitor_corage_rate "/century/prediction"
        monitor_corage_rate "/century/localization/pose"
        monitor_corage_rate "/century/perception/obstacles"
        
        log_with_timestamp "CYCLE: Monitoring cycle completed"
        sleep 5
    done
}

main "$@"
