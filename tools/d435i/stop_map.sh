#!/usr/bin/env bash
# Shut down mapping cleanly so RTAB-Map writes its database (the 3D map).
cd "$(dirname "$0")"

if ! docker ps --format '{{.Names}}' | grep -qx mining-map; then
  echo "Mapping container 'mining-map' is not running." >&2
  docker rm mining-map >/dev/null 2>&1 || true
  exit 1
fi

NAME="$(cat maps/.current 2>/dev/null || echo mapping_$(date +%Y%m%d_%H%M%S))"

echo "Stopping (this can take up to 60 s while the database is written)..."
docker stop -t 60 mining-map >/dev/null
docker logs mining-map > "maps/$NAME.log" 2>&1
docker rm mining-map >/dev/null

# container runs as root; give files back to you
docker run --rm -v "$PWD":/ws mining-ros chown -R "$(id -u):$(id -g)" /ws/maps

echo "Done:"
ls -lh maps/"$NAME".*
