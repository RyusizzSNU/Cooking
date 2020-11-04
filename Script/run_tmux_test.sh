tmux kill-session
sleep 1

tmux new-session -d
tmux set -g mouse on

tmux split-window -h -p 70

tmux select-pane -L
tmux split-window -v -p 25

tmux select-pane -D
tmux split-window -v -p 25

tmux select-pane -t 0
tmux send "source ~/.bashrc" C-m
tmux send "roscore" C-m

tmux select-pane -t 1
tmux send "python Run/urx_wizard.py Config/urx_config.json left" C-m

tmux select-pane -t 2
tmux send "python Run/urx_wizard.py Config/urx_config.json right" C-m

tmux select-pane -t 3

tmux attach-session -d