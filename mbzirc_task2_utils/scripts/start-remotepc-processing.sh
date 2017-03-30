#!/usr/bin/env bash
PROGNAME=$(basename $0)
LOCAL_IP="192.168.97.136"
REMOTE_IP="192.168.96.188"

usage() {
  echo "Usage: $PROGNAME [option]"
  echo
  echo "options:"
  echo "  -h, --help"
  echo "  -i, --local-ip <ARG>         specify remote ip for sending data. default ip is $LOCAL_IP"
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
    '-i'|'--local-ip' )
      if [[ -z "$2" ]] || [[ "$2" =~ ^-+ ]]; then
        echo "$PROGNAME : -i and --local-ip option requires an argument!"
      fi
      LOCAL_IP="$2"
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
byobu-tmux new-session -s remote-pc -d -n roscore roscore \; new-window -n udp \; new-window -n joy \; new-window -n sixad
sleep 1

byobu-tmux send-keys -t roscore "roscore" C-m

if [ -z $FLG_LARGE ]; then
    byobu-tmux send-keys -t udp "roslaunch mbzirc_task2_network aero_remotepc.launch LOCAL_IP:='$LOCAL_IP' REMOTE_IP:='$REMOTE_IP' USE_LARGEDATA:='false'" C-m
else
    byobu-tmux send-keys -t udp "roslaunch mbzirc_task2_network aero_remotepc.launch LOCAL_IP:='$LOCAL_IP' REMOTE_IP:='$REMOTE_IP' USE_LARGEDATA:='true'" C-m
fi

byobu-tmux send-keys -t joy "rosrun joy joy_node" C-m
byobu-tmux send-keys -t sixad "sixad -s" C-m


byobu-tmux attach-session -t remote-pc
