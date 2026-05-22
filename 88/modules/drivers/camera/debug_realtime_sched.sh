#!/usr/bin/env bash
set -u

PATTERN="${1:-camera|mainboard|cyber|component}"

echo "===== realtime scheduling diagnostic ====="
echo "time: $(date '+%F %T %z')"
echo "host: $(hostname)"
echo "user: $(id)"
echo

if [[ "${1:-}" =~ ^[0-9]+$ ]]; then
  PID="$1"
else
  echo "===== candidate processes ====="
  ps -ef | grep -E "$PATTERN" | grep -v grep || true
  echo
  PID="$(pgrep -f "$PATTERN" | head -n 1 || true)"
fi

if [[ -z "${PID:-}" ]]; then
  echo "ERROR: no process matched pattern: $PATTERN"
  echo "Usage:"
  echo "  bash $0 <pid>"
  echo "  bash $0 '<process-name-regex>'"
  exit 1
fi

if [[ ! -d "/proc/$PID" ]]; then
  echo "ERROR: /proc/$PID does not exist"
  exit 1
fi

echo "===== selected process ====="
echo "PID=$PID"
echo -n "cmdline: "
tr '\0' ' ' < "/proc/$PID/cmdline"
echo
echo -n "exe: "
readlink -f "/proc/$PID/exe" 2>/dev/null || true
echo -n "cwd: "
readlink -f "/proc/$PID/cwd" 2>/dev/null || true
echo

echo "===== process scheduling ====="
ps -o pid,ppid,user,group,cls,rtprio,pri,ni,psr,cmd -p "$PID"
echo

echo "===== process limits ====="
grep -E "Max realtime priority|Max nice priority|Max locked memory" "/proc/$PID/limits" || true
echo

echo "===== current shell limits ====="
echo -n "ulimit -r: "
ulimit -r
echo -n "ulimit -e: "
ulimit -e
echo

echo "===== process capabilities ====="
grep -E "CapInh|CapPrm|CapEff|CapBnd|CapAmb" "/proc/$PID/status" || true
CAPEFF="$(awk '/CapEff/ {print $2}' "/proc/$PID/status" 2>/dev/null || true)"
if command -v capsh >/dev/null 2>&1 && [[ -n "$CAPEFF" ]]; then
  echo "decoded CapEff:"
  capsh --decode="0x$CAPEFF" || true
else
  echo "decoded CapEff: capsh not available"
fi
echo

echo "===== executable file capability ====="
EXE="$(readlink -f "/proc/$PID/exe" 2>/dev/null || true)"
if [[ -n "$EXE" ]]; then
  ls -l "$EXE" || true
  if command -v getcap >/dev/null 2>&1; then
    getcap "$EXE" || true
  else
    echo "getcap not available"
  fi
else
  echo "cannot resolve executable path"
fi
echo

echo "===== minimal chrt test from this shell ====="
if command -v chrt >/dev/null 2>&1; then
  chrt -r 90 true >/tmp/chrt_90.out 2>&1
  RET90=$?
  echo "chrt -r 90 true: ret=$RET90"
  if [[ $RET90 -ne 0 ]]; then
    sed 's/^/  /' /tmp/chrt_90.out
  fi

  chrt -r 1 true >/tmp/chrt_1.out 2>&1
  RET1=$?
  echo "chrt -r 1 true: ret=$RET1"
  if [[ $RET1 -ne 0 ]]; then
    sed 's/^/  /' /tmp/chrt_1.out
  fi
else
  echo "chrt not available"
fi
echo

echo "===== realtime threads in selected process ====="
ps -eLo pid,tid,cls,rtprio,pri,psr,comm | awk -v pid="$PID" '$1 == pid {print}'
echo

echo "===== interpretation hints ====="
echo "1. If office has Max realtime priority > 0 but field has 0, field cannot use SCHED_RR."
echo "2. If office CapEff/CapBnd decodes cap_sys_nice but field does not, field lacks CAP_SYS_NICE."
echo "3. If office executable has cap_sys_nice+ep but field does not, align file capability."
echo "4. If chrt fails on field but succeeds on office, the difference is OS/user/capability/ulimit, not camera code."
