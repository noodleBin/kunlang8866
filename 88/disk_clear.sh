#!/bin/bash

LOG_FILE="/century/data/log/cleanup_daemon.log"
TARGET_DIRS=("/century/data/log" "/century/data/bag")
THRESHOLD_PERCENT=80
CHECK_INTERVAL=10
BATCH_SIZE=20

mkdir -p "$(dirname "$LOG_FILE")"
touch "$LOG_FILE"
chmod 666 "$LOG_FILE" 2>/dev/null

# Run cleanup at low CPU/IO priority so big-file deletions do not stall the
# realtime perception pipeline. (ionice is a no-op on the 'none' nvme IO
# scheduler but harmless; renice reduces the periodic find/sort scan cost.)
renice -n 19 -p $$ >/dev/null 2>&1
ionice -c3 -p $$ >/dev/null 2>&1

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> "$LOG_FILE"
}


cleanup_loop() {
    log "===== Disk Cleanup Daemon Started ====="
    while true; do
        today_log_count=$(find /century/data/log/ -type f -newermt $(date +%Y-%m-%d) 2>/dev/null | wc -l)
        usage=$(df -P /century | awk 'NR==2 {print $5}' | tr -d '%')
        adjusted_batch_size=$((BATCH_SIZE + today_log_count))
        log "===== Start scanning disk (Usage: ${usage}%) ====="
        if [ "$usage" -gt "$THRESHOLD_PERCENT" ]; then
            log "Disk usage $usage% exceeds threshold $THRESHOLD_PERCENT%. Cleaning..."
            find "${TARGET_DIRS[@]}" -type f -printf "%T@ %p\n" 2>/dev/null | \
                awk '{printf "%d %s\n", $1, $2}' | \
                sort -n | head -n "$adjusted_batch_size" | cut -d' ' -f2- | while read -r file; do
                    usage=$(df -P /century | awk 'NR==2 {print $5}' | tr -d '%')
                    if [ "$usage" -le $THRESHOLD_PERCENT ]; then
                        log "Disk usage dropped to ${usage}% (<=$THRESHOLD_PERCENT%), stopping current cleanup cycle."
			            break
                    fi

                    if [ -f "$file" ]; then
                        if [[ "$file" == /century/data/log/* ]]; then
                            file_date=$(stat -c %y "$file" | cut -d' ' -f1)
                            today=$(date +%Y-%m-%d)
                            if [ "$file_date" == "$today" ]; then
                                log "Skipped (today's file): $file"
                                continue
                            fi
                        fi
                        if rm -f "$file"; then
                            log "Deleted: $file"
                            # Spread deletions over time so a batch of large
                            # .record files does not cause an IO burst that
                            # stalls realtime perception (~0.5s latency spikes).
                            sleep 0.3
                        else
                            log "Failed to delete: $file"
                        fi
                    fi
                done
        else
                sleep "$CHECK_INTERVAL"
        fi
    done
}

function disk_clear_all() {
    log "Starting disk cleanup task"

    local base_path=$(readlink -f /century) || { log "Error: Failed to resolve /century symlink" "ERROR"; return 1; }
    local base_name=$(basename "$base_path")
    local workspace=$(dirname "$base_path")
    local tmp_dir="$workspace/tmp"

    if [[ "$workspace" != /home/nvidia/* ]] && [[ "$workspace" != /home/nvidia ]]; then
        log "Error: Workspace '$workspace' is not under /home/nvidia" "ERROR"
        return 1
    fi

    mkdir -p "$tmp_dir" 2>/dev/null || { log "Error: Failed to create tmp directory $tmp_dir" "ERROR"; return 1; }

    log "Cleaning workspace, preserving $base_name, vehicle_conf*, Vehicle_conf*, tmp"
    find "$workspace" -mindepth 1 -maxdepth 1 \
        -not -name "$base_name" \
        -not -name "vehicle_conf*" \
        -not -name "Vehicle_conf*" \
        -not -name "tmp" \
        -exec rm -rf {} + 2>/dev/null || log "Warning: Potential errors during workspace cleanup" "WARN"

    local max_size=100G
    local target_size=70G

    local current_size=$(du -sb "$tmp_dir" 2>/dev/null | cut -f1) || { log "Error: Failed to get $tmp_dir size" "ERROR"; return 1; }
    local max_size_bytes=$(numfmt --from=iec $max_size 2>/dev/null) || { log "Error: Invalid max_size value $max_size" "ERROR"; return 1; }
    local target_size_bytes=$(numfmt --from=iec $target_size 2>/dev/null) || { log "Error: Invalid target_size value $target_size" "ERROR"; return 1; }

    if [ $current_size -gt $max_size_bytes ]; then
        log "tmp directory oversized ($(numfmt --to=iec $current_size)), cleaning to $target_size"

        local before_size=$current_size
        local deleted_count=0

        while IFS= read -r file; do
            [ $(du -sb "$tmp_dir" | cut -f1) -le $target_size_bytes ] && break
            rm -f "$file" 2>/dev/null && ((deleted_count++)) || log "Warning: Failed to delete $file" "WARN"
            [ $((deleted_count % 100)) -eq 0 ] && log "Deleted $deleted_count files, current size: $(du -sh "$tmp_dir" | cut -f1)"
        done < <(find "$tmp_dir" -type f -printf "%T@ %p\n" 2>/dev/null | sort -n | cut -d' ' -f2-)

        log "Cleanup complete: Deleted $deleted_count files, reduced from $(numfmt --to=iec $before_size) to $(numfmt --to=iec $(du -sb "$tmp_dir" | cut -f1))"
    else
        log "tmp directory size normal ($(numfmt --to=iec $current_size)), no cleanup needed"
    fi

    log "Disk cleanup task completed"
}

# clear all disk
disk_clear_all
# disk_clear_all >> "$LOG_FILE" 2>&1 &
cleanup_loop >> "$LOG_FILE" 2>&1 &
