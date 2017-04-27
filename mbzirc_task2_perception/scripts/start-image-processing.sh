#!/usr/bin/env bash
PROGNAME=$(basename $0)
REMOTE_IP="192.168.96.190" #lysithea
LOCAL_IP="192.168.96.188"

usage() {
  echo "Usage: $PROGNAME [option]"
  echo
  echo "options:"
  echo "  -h, --help"
  echo "  -n, --no-communication        run script without UDP communication program."
  echo "  -r, --remote-ip <ARG>         specify remote ip for sending data. default ip is $REMOTE_IP"
  echo "  -l, --large-data              send large data to remote pc. small data is used by default."
  echo
  exit 1
}

for OPT in "$@"
do
  case "$OPT" in
    '-h'|'--help' )
      usage
      exit 1
      ;;
    '-n'|'--no-communication' )
      FLG_COM=1
      shift 1
      ;;
    '-r'|'--remote-ip' )
      if [[ -z "$2" ]] || [[ "$2" =~ ^-+ ]]; then
        echo "$PROGNAME : -r and --remote-ip option requires an argument!"
      fi
      REMOTE_IP="$2"
      shift 1
      ;;
    '-l'|'--large-data' )
      FLG_LARGE=1
      shift 1
      ;;
    '--'|'-' )
      shift 1
      param+=( "$@" )
      break
      ;;
    -*)
      echo "$PROGNAME: unknown option $(echo $1 | sed 's/^-*//'). Check '$PROGNAME -h'" 1>&2
      exit 1
      ;;
  esac
done

byobu-tmux start-server
byobu-tmux new-session -s image-proc -d -n zed \; new-window -n handeye
sleep 1

byobu-tmux send-keys -t zed "roslaunch mbzirc_task2_perception multi_resolution_zed.launch" C-m
byobu-tmux send-keys -t handeye "roslaunch netusbcam netusbcam.launch" C-m

if [ -z $FLG_COM ]; then
    byobu-tmux new-window -n udp
    if [ -z $FLG_LARGE ]; then
        byobu-tmux send-keys -t udp "roslaunch mbzirc_task2_network aero_udp.launch REMOTE_IP:='$REMOTE_IP' LOCAL_IP:='$LOCAL_IP' USE_LARGEDATA:='false'" C-m
    else
        byobu-tmux send-keys -t udp "roslaunch mbzirc_task2_network aero_udp.launch REMOTE_IP:='$REMOTE_IP' LOCAL_IP:='$LOCAL_IP' USE_LARGEDATA:='true'" C-m
    fi
fi

#byobu-tmux new-window -n long-range
#byobu-tmux send-keys -t long-range "roslaunch mbzirc_task2_perception long_range_panel_finder.launch" C-m
byobu-tmux new-window -n middle-range
byobu-tmux send-keys -t middle-range "roslaunch mbzirc_task2_perception middle_range_panel_finder.launch" C-m
byobu-tmux new-window -n short-range
byobu-tmux send-keys -t short-range "roslaunch mbzirc_task2_perception short_range_panel_finder.launch" C-m
byobu-tmux new-window -n get-roi
byobu-tmux send-keys -t get-roi "roslaunch mbzirc_task2_perception get_roi_on_panel.launch" C-m
byobu-tmux new-window -n panel-proc
byobu-tmux send-keys -t panel-proc "roslaunch mbzirc_task2_perception panel_image_processing.launch" C-m
byobu-tmux new-window -n handeye-proc
byobu-tmux send-keys -t handeye-proc "roslaunch mbzirc_task2_perception handeye_image_processing.launch" C-m

byobu-tmux attach-session -t image-proc
