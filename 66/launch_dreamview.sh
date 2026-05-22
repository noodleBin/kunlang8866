#!/bin/bash
if pgrep -f dreamview | grep -vw $$ >/dev/null; then
    if ps -eo stat,cmd | grep -w dreamview | grep -v grep | grep -q Z; then
      pgrep -f dreamview | grep -vw $$ | xargs -r kill -9
      bash /century/scripts/bootstrap.sh restart full > /dev/null 2>&1 &
    fi
else
    bash /century/scripts/bootstrap.sh restart full > /dev/null 2>&1 &
fi

