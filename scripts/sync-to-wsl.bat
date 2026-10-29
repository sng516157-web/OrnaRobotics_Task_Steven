REM Author: Steven (https://github.com/sng516157-web) — 2026-10-30
REM Sync the Windows copy of this repo into the WSL workspace and rebuild.
REM Edit the source path below if your checkout lives elsewhere.
@echo off
wsl -d Ubuntu-24.04 -- bash -c "rsync -a --delete '/mnt/c/Users/sng51/Documents/kimi/Workspaces/Orna Robotics Task/cable-insertion-sim/' ~/cable_ws/src/cable-insertion-sim/ && source /opt/ros/jazzy/setup.bash && cd ~/cable_ws && colcon build --symlink-install --parallel-workers 2"
pause
