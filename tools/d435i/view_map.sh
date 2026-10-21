#!/usr/bin/env bash
# List saved maps (newest = 1), open one in the RTAB-Map database viewer, or delete.
#   ./view_map.sh            interactive menu
#   ./view_map.sh <number>   open that entry of the list directly
cd "$(dirname "$0")"

list_maps() {
  MAPS=()
  while IFS= read -r f; do MAPS+=("$f"); done < <(ls -t maps/*.db 2>/dev/null)
}

show_list() {
  list_maps
  echo
  if [ ${#MAPS[@]} -eq 0 ]; then
    echo "No maps in maps/."
    return
  fi
  printf "  %-3s %-32s %-9s %s\n" "#" "NAME" "SIZE" "DATE"
  local i=1 f name size date
  for f in "${MAPS[@]}"; do
    name="$(basename "$f" .db)"
    size="$(du -h "$f" | cut -f1)"
    date="$(date -r "$f" '+%Y-%m-%d %H:%M')"
    printf "  %-3s %-32s %-9s %s\n" "$i" "$name" "$size" "$date"
    i=$((i + 1))
  done
}

open_map() {
  local db="$1"
  xhost +local:docker >/dev/null 2>&1
  echo "Opening $db (close the viewer window to return)"
  docker run --rm \
    --net=host --gpus all \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$PWD":/ws \
    -u "$(id -u):$(id -g)" -e HOME=/tmp \
    mining-ros \
    bash -c "source /opt/ros/humble/setup.bash && rtabmap-databaseViewer /ws/$db"
}

delete_map() {
  local db="$1" base="${1%.db}"
  local files=("$base".*)
  echo "This will permanently delete:"
  printf "  %s\n" "${files[@]}"
  read -rp "Type 'yes' to confirm: " ans
  if [ "$ans" = "yes" ]; then
    rm -f -- "${files[@]}"
    echo "Deleted."
  else
    echo "Cancelled."
  fi
}

valid_index() {
  [[ "$1" =~ ^[0-9]+$ ]] && [ "$1" -ge 1 ] && [ "$1" -le ${#MAPS[@]} ]
}

# direct mode: ./view_map.sh 2
if [ -n "$1" ]; then
  list_maps
  if valid_index "$1"; then
    open_map "${MAPS[$(($1 - 1))]}"
    exit 0
  fi
  echo "No entry '$1'." >&2
  show_list >&2
  exit 1
fi

while true; do
  show_list
  echo
  echo "  <number>  open      d<number>  delete (e.g. d2)      q  quit"
  read -rp "> " choice
  case "$choice" in
    q|Q|"") [ "$choice" = "" ] && continue; exit 0 ;;
    d[0-9]*)
      n="${choice#d}"
      if valid_index "$n"; then delete_map "${MAPS[$((n - 1))]}"; else echo "Invalid number."; fi ;;
    *)
      if valid_index "$choice"; then open_map "${MAPS[$((choice - 1))]}"; else echo "Invalid choice."; fi ;;
  esac
done
