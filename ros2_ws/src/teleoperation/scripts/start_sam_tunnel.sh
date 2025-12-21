#!/usr/bin/env bash
set -e

LOCAL_PORT=18000
REMOTE_PORT=8000
REMOTE_HOST="rtx_jump"

if lsof -i tcp:${LOCAL_PORT} &>/dev/null; then
    echo "[SAM TUNNEL] Port ${LOCAL_PORT} already in use, skipping tunnel."
    exit 0
fi

echo "[SAM TUNNEL] Forwarding localhost:${LOCAL_PORT} → ${REMOTE_HOST}:${REMOTE_PORT}"

ssh -f -N \
    -o ExitOnForwardFailure=yes \
    -o ServerAliveInterval=60 \
    -o StrictHostKeyChecking=no \
    -L ${LOCAL_PORT}:localhost:${REMOTE_PORT} \
    ${REMOTE_HOST}
