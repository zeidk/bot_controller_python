
# Package for Lecture10 (ENPM809E 2022)

## Steps to clone and run packages

Assuming you have the catkin workspace `lecture10_ws` located in your home directory:
- `cd ~/lecture10_ws/src`
- `git clone https://github.com/zeidk/bot_controller_python.git`
- `cd ~/lecture10_ws`
- `rosdep install --from-paths . --ignore-src --rosdistro noetic -y` (replace `noetic` with `melodic` if you are using `melodic`)
- `catkin build`
- `source ~/lecture10_ws/devel/setup.bash` (add this line in your `.bashrc`)
- `source ~/.bashrc`

# Use the correct shebang line

If you are using Python2 and Melodic:
- Change the shebang line `#!/usr/bin/env python3` to `#!/usr/bin/env python` in the files located in `bot_controller/nodes`.
