#!/bin/sh
tmux new-session -d
tmux split-window -v 
tmux split-window -h 'ecal_rec_gui --config ~/WildPose_v1.1/src/wildpose_bringup/config/config.ecalrec'
tmux split-window -h
tmux select-pane -t 0
tmux -2 attach-session -d