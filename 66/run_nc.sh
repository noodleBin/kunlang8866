#!/bin/bash
#set -x
SCRIPT_PID=$$
SCRIPT_NAME="run_nc.sh"


#OLD_PIDS=$(pgrep -f "$SCRIPT_NAME" | grep -v "$SCRIPT_PID")
#if [[ -n "$OLD_PIDS" ]]; then
#    echo "Killing old instances of $SCRIPT_NAME: $OLD_PIDS"
#    echo '123' | sudo -SE kill -9 $OLD_PIDS
#fi

OLD_PIDS=$(pgrep -f "$SCRIPT_NAME" | grep -v $$ | xargs ps -o pid=,etimes= | awk '$2 > 5 {print $1}')
echo $OLD_PIDS
if [[ -n "$OLD_PIDS" ]]; then
    echo "Killing old instances of $SCRIPT_NAME: $OLD_PIDS"
    echo '123' | sudo -SE kill -9 $OLD_PIDS
fi

pkill -9 nc

mkdir -p /century/data/bag/"$(date +%Y%m%d)"/beiyun

start_nc() {
    local ip=$1
    local port=$2
    local name=$3
    local LOG_FILE="/century/data/bag/"$(date +%Y%m%d)"/beiyun/beiyun_${3}_${SCRIPT_PID}_$(date +"%Y%m%d%H%M").log.${port}"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] start listen nc..." | tee -a "$LOG_FILE"
    while true; do
	
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] start nc: $ip:$port" | tee -a "$LOG_FILE"
        nohup nc "$ip" "$port" >> "$LOG_FILE" 2>&1 &
        NC_PID=$!
        wait $NC_PID
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] nc ($NC_PID) exit,restart nc..." | tee -a "$LOG_FILE"
        sleep 2
	local LOG_FILE="/century/data/bag/"$(date +%Y%m%d)"/beiyun/beiyun_${3}_${SCRIPT_PID}_$(date +"%Y%m%d%H%M").log.${port}"
    done
}

(start_nc 192.168.1.151 3333 'icom3' &)
(start_nc 192.168.1.151 4444 'icom4' &)

nohup bash -c "sleep infinity" > /dev/null 2>&1 &
disown -a
exit 0
