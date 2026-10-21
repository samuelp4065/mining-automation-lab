#!/usr/bin/env bash
# Start headless mapping, show live status, and stop when you press q.
#   ./map.sh [name]              -> saves maps/<name>.db (default: mapping_<timestamp>)
#   ./map.sh [name] --detach     -> start in the background only (stop later with ./stop_map.sh)
cd "$(dirname "$0")"

NAME=""; DETACH=0; USE_INFRA=false
for a in "$@"; do
  case "$a" in
    -d|--detach) DETACH=1 ;;
    --infra) USE_INFRA=true ;;
    *) NAME="$a" ;;
  esac
done
NAME="${NAME:-mapping_$(date +%Y%m%d_%H%M%S)}"
mkdir -p maps

if ! docker image inspect mining-ros >/dev/null 2>&1; then
  docker build -t mining-ros . || exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -qx mining-map; then
  echo "Container 'mining-map' already exists. Run ./stop_map.sh first." >&2
  exit 1
fi

# keep the name so stop_map.sh knows what to call the log
echo "$NAME" > maps/.current

docker run -d --name mining-map \
  --net=host --privileged \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v /dev:/dev \
  -v "$PWD":/ws \
  mining-ros \
  bash -c "source /opt/ros/humble/setup.bash && exec ros2 launch /ws/launch/mapping_full.launch.py database_path:=/ws/maps/$NAME.db viz:=false use_infra:=$USE_INFRA" \
  >/dev/null || exit 1

echo "Mapping started -> maps/$NAME.db"

if [ "$DETACH" = 1 ]; then
  echo "  logs:  docker logs -f mining-map"
  echo "  stop:  ./stop_map.sh"
  exit 0
fi

echo "Hold still ~10 s at the start, then move slowly with overlapping views."
echo "Press q to stop and save the map."
echo

docker exec mining-map bash -c \
  "source /opt/ros/humble/setup.bash && exec python3 /ws/scripts/monitor.py /ws/maps/$NAME.db" &
MON=$!

stop_all() {
  kill "$MON" 2>/dev/null
  echo; echo
  ./stop_map.sh
  exit 0
}
trap stop_all INT TERM

while true; do
  if read -rsn1 -t 1 key </dev/tty; then
    case "$key" in q|Q) stop_all ;; esac
  fi
  if ! docker ps --format '{{.Names}}' | grep -qx mining-map; then
    kill "$MON" 2>/dev/null
    echo; echo "Mapping container exited unexpectedly. Last log lines:" >&2
    docker logs --tail 20 mining-map >&2
    docker logs mining-map > "maps/$NAME.log" 2>&1
    docker rm mining-map >/dev/null 2>&1
    exit 1
  fi
done
