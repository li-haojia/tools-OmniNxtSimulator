#!/bin/bash
if [ -z "$1" ]; then
  echo "Usage: $0 <filename>"
  exit 1
fi

FILENAME=$1

pids=$(pgrep -f "$FILENAME")

if [ -z "$pids" ]; then
  echo "No processes found with filename: $FILENAME"
else
  echo "Killing processes with filename: $FILENAME"
  for pid in $pids; do
    kill -9 "$pid"
    echo "Killed process with PID: $pid"
  done
fi