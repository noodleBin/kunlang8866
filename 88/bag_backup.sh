#!/bin/bash

trap 'echo "INTERRUPTED" >&2; exit 130' INT

set -euo pipefail

# --------------------------
# Single instance lock
# --------------------------
LOCKFILE="/tmp/bag_backup.lock"
exec 200>"$LOCKFILE"
flock -n 200 || { echo "Another instance of bag_backup.sh is running. Exiting."; exit 1; }

# --------------------------
# Default parameters
# --------------------------
DATE=$(date +%Y%m%d)
START_TS=$(date +%Y%m%d%H%M%S)
N=2
GAP_THRESHOLD=50
SIZE_STABLE_SEC=5 # Duration for continuous stability
DOWN_WAIT_SEC=5
DRY_RUN=0
MAX_FILES=5
# Maximum wait time for file stability detection (5 minutes)
MAX_STABILITY_WAIT_SEC=300

# --------------------------
# Log file
# --------------------------
LOG_FILE="$HOME/workspace/tmp/bag_backup_$(date +%y%m%d).txt"
mkdir -p "$(dirname "$LOG_FILE")"

log() {
    msg="$(date '+%Y-%m-%d %H:%M:%S') | $*"
    echo "$msg"
    echo "$msg" >> "$LOG_FILE"
}

# --------------------------
# Parse command line arguments
# --------------------------
while getopts "d:t:n:" opt; do
    case "$opt" in
        d) DATE="$OPTARG" ;;
        t) START_TS="$OPTARG" ;;
        n) N="$OPTARG" ;;
        *) ;;
    esac
done

SRC_DIR="/century/data/bag/$DATE"
DST_DIR="$HOME/workspace/tmp/bag_backup/$DATE"

log "SRC_DIR=$SRC_DIR"
log "DST_DIR=$DST_DIR"
log "START_TS=$START_TS"
log "N=$N"
log "GAP_THRESHOLD=${GAP_THRESHOLD}s, SIZE_STABLE_SEC=${SIZE_STABLE_SEC}s, DOWN_WAIT_SEC=${DOWN_WAIT_SEC}s, DRY_RUN=$DRY_RUN, MAX_STABILITY_WAIT_SEC=${MAX_STABILITY_WAIT_SEC}s"

mkdir -p "$DST_DIR"

# --------------------------
# Helper function: filename -> Unix timestamp
# --------------------------
ts_to_sec() {
    ts="$1"
    formatted="${ts:0:4}-${ts:4:2}-${ts:6:2} ${ts:8:2}:${ts:10:2}:${ts:12:2}"
    date -d "$formatted" +%s
}

# --------------------------
# Read current file list (optimized to read the latest MAX_FILES)
# --------------------------
read_files() {
    shopt -s nullglob
    
    local file_list
    # Use ls with sort to read and sort the file list, and limit quantity via tail -n $MAX_FILES
    mapfile -t file_list < <(
        ls -d "$SRC_DIR"/*.record 2>/dev/null | sort | tail -n "$MAX_FILES"
    )
    
    local basename_list=()
    for fullpath in "${file_list[@]}"; do
        basename_list+=("$(basename "$fullpath")")
    done

    if (( ${#basename_list[@]} > 0 )); then
        printf "%s\n" "${basename_list[@]}"
    fi
    shopt -u nullglob
}

# --------------------------
# Find file closest to START_TS
# --------------------------
closest_file=""
closest_diff=999999
start_sec=$(ts_to_sec "$START_TS")

mapfile -t initial_files < <(read_files)

for fname in "${initial_files[@]}"; do
    ts=${fname%.record}

    if [[ "$ts" -le "$START_TS" ]]; then
        ts_sec=$(ts_to_sec "$ts")
        diff=$((start_sec - ts_sec))

        if (( diff < closest_diff )); then
            closest_diff=$diff
            closest_file="$fname"
        fi
    fi
done

if [[ -z "$closest_file" ]]; then
    log "No file found before START_TS $START_TS, exiting."
    exit 1
fi

if (( closest_diff > GAP_THRESHOLD )); then
    log "Closest file to START_TS: $closest_file (diff ${closest_diff}s) exceeds GAP_THRESHOLD=$GAP_THRESHOLD s, exiting."
    exit 1
else
    log "Closest file to START_TS: $closest_file (diff ${closest_diff}s)"
fi

# --------------------------
# Sort all files to locate index
# --------------------------
mapfile -t all_files < <(read_files)
idx=-1
for i in "${!all_files[@]}"; do
    if [[ "${all_files[i]}" == "$closest_file" ]]; then
        idx=$i
        break
    fi
done

if (( idx == -1 )); then
    log "Error: Closest file $closest_file not found in list (list refresh issue?)."
    exit 1
fi

# --------------------------
# Backup N files upward (excluding current file)
# --------------------------
up_files=("${all_files[@]:0:idx}")

# Fix: Initialize prev_sec to closest_file timestamp to ensure interval check on first iteration.
prev_sec=$(ts_to_sec "${closest_file%.record}") 
count=0

for ((i=${#up_files[@]}-1; i>=0; i--)); do
    file="${up_files[i]}"
    ts=${file%.record}
    ts_sec=$(ts_to_sec "$ts")

    gap=$((prev_sec - ts_sec))
    log "Upward backup: $file (interval with next: ${gap}s)"

    if (( gap > GAP_THRESHOLD )); then
        log "Stop upward backup due to large gap (${gap}s)"
        break
    fi

    prev_sec="$ts_sec"
    (( DRY_RUN == 0 )) && mv "$SRC_DIR/$file" "$DST_DIR/"
    (( ++count >= N )) && break
done

# --------------------------
# Backup N files downward (including current file)
# --------------------------
down_count=0
prev_sec="" 
current_file_to_process="$closest_file"

while (( down_count < N )); do

    # --- Step 1: Prepare current file for processing ---
    fullpath="$SRC_DIR/$current_file_to_process"

    if [[ ! -f "$fullpath" ]]; then
        log "Error: File $current_file_to_process not found (unexpected missing). Stop."
        break
    fi

    # Log time interval info
    ts=${current_file_to_process%.record}
    ts_sec=$(ts_to_sec "$ts")
    if [[ -n "$prev_sec" ]]; then
        gap=$((ts_sec - prev_sec))
        log "Downward backup: $current_file_to_process (interval with previous: ${gap}s)"
    else
        log "Starting file (downward): $current_file_to_process"
    fi

    # --------------------------
    # --- Step 2: Wait for current file to stabilize ---
    # --------------------------
    size1=$(stat -c%s "$fullpath")
    mtime1=$(stat -c%Y "$fullpath")

    log "Waiting for file to become stable: $current_file_to_process (Initial Size: $size1 bytes, MTime: $mtime1). Max wait: ${MAX_STABILITY_WAIT_SEC}s"
    stable=0
    wait_count=0

    while (( wait_count < MAX_STABILITY_WAIT_SEC )); do
        sleep 1
        wait_count=$((wait_count+1))
        
        if [[ ! -f "$fullpath" ]]; then
            log "Error: File $current_file_to_process disappeared during stabilization."
            exit 1
        fi
        
        size2=$(stat -c%s "$fullpath")
        mtime2=$(stat -c%Y "$fullpath")
        
        if (( size1 == size2 && mtime1 == mtime2 )); then
            stable=$((stable+1))
            if (( stable >= SIZE_STABLE_SEC )); then
                log "File stabilized after $wait_count seconds."
                break
            fi
        else
            log "Debug Stability Check: File changed (Size $size1 -> $size2, MTime $mtime1 -> $mtime2). Resetting stable count (Wait: ${wait_count}s)."
            size1=$size2
            mtime1=$mtime2
            stable=0
        fi
    done

    if (( stable < SIZE_STABLE_SEC )); then
        log "Error: File $current_file_to_process did not stabilize within ${MAX_STABILITY_WAIT_SEC}s. Exiting."
        exit 1
    fi

    # --- Step 3: Move file ---
    log "File complete: $current_file_to_process  --> start copying"
    (( DRY_RUN == 0 )) && mv "$fullpath" "$DST_DIR/"
    
    last_moved_file="$current_file_to_process"
    prev_sec="$ts_sec"
    down_count=$((down_count+1))

    (( down_count >= N )) && break

    # --- Step 4: Loop to detect next new file ---
    log "Waiting for the next file to appear after $last_moved_file ..."
    
    found_next_file=""
    
    for (( w=1; w<=DOWN_WAIT_SEC; w++ )); do
        sleep 1
        mapfile -t current_list < <(read_files)
        
        log "Debug Downward Scan: Try ${w}/${DOWN_WAIT_SEC}s. Files found: ${#current_list[@]}. Latest file: ${current_list[-1]:-None}"
        
        for fname in "${current_list[@]}"; do
            if [[ "$fname" > "$last_moved_file" ]]; then
                found_next_file="$fname"
                log "Detected next file: $found_next_file (at attempt ${w}s)"
                break 2
            fi
        done
    done

    # --- Step 5: Evaluate detection result ---
    if [[ -n "$found_next_file" ]]; then
        current_file_to_process="$found_next_file"
    else
        log "Time out. No new file appeared after $last_moved_file within $DOWN_WAIT_SEC seconds."
        log "Debug Info: Current files in SRC_DIR:"
        ls -l "$SRC_DIR"/*.record 2>/dev/null || echo "  (No .record files found)"
        log "Stop downward backup."
        break
    fi

done

log "Backup job finished."

exit 0