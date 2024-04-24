#!/bin/bash

cd pizzaro-firmware

# 启动tmux
tmux new-session -d -s my_session

# 重命名第一个窗口为window1
tmux rename-window -t my_session:0 'window1'

# 垂直分割成上下两个窗格
tmux split-window -t my_session:0 -v -p 55

tmux split-window -t my_session:0 -v -p 20

# 上面的窗格水平分割成四个田字布局
tmux select-pane -t my_session:0.0
tmux split-window -t my_session:0.0 -h -p 50
tmux select-pane -t my_session:0.2
tmux split-window -t my_session:0.2 -h -p 50


# 重命名4个田字窗格
tmux select-pane -t my_session:0.0.0 -T 'mc'
tmux select-pane -t my_session:0.0.1 -T 'mmd'
tmux select-pane -t my_session:0.1.0 -T 'hpd'
tmux select-pane -t my_session:0.1.2 -T 'dtu'

# 重命名下面的横向窗格
tmux select-pane -t my_session:0.2 -T 'cli'

# 在第一个窗格运行命令
tmux send-keys -t my_session:0.0 './script/run_mc.sh' Enter

# 在第二个窗格运行命令
tmux send-keys -t my_session:0.1 './script/run_mmd.sh' Enter

# 在第三个窗格运行命令
tmux send-keys -t my_session:0.2 './script/run_hpd.sh' Enter

# 在第四个窗格运行命令
tmux send-keys -t my_session:0.3 './script/run_dtu.sh' Enter

# 在第五个窗格运行命令
tmux send-keys -t my_session:0.4 'cd x86_app' Enter
tmux send-keys -t my_session:0.4 'cargo run --bin pc_controller -- --probe 16c0:27dd' Enter

# 进入tmux会话
tmux attach-session -t my_session
