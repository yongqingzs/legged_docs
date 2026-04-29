# ocs2_quadruped_controller start
## build
base
```bash
# ubuntu 24.04
# ros2-jazzy
mkdir -p ~/jazzy_ws/src
cd ~/jazzy_ws/src
git clone https://github.com/legubiao/quadruped_ros2_control.git
git checkout 5434c5810d1a7fe223bcfd04550e9d3bfdd4b458

cd ~/jazzy_ws
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j4" colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input --symlink-install

sudo apt-get install ros-jazzy-ros-gz
MAKEFLAGS="-j4" colcon build --packages-up-to gz_quadruped_playground --symlink-install
```

ocs2_ros2
```bash
cd ~/jazzy_ws/src
git clone https://github.com/legubiao/ocs2_ros2
cd ocs2_ros2
git checkout bac496f915f3b6a9c859b8989e3d6b6f56970271

git submodule update --init --recursive

cd ../../
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j4" colcon build --packages-up-to ocs2_quadruped_controller  --symlink-install
```

rl
```bash
cd ~
wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.5.0%2Bcpu.zip
unzip libtorch-cxx11-abi-shared-with-deps-2.5.0+cpu.zip

echo 'export Torch_DIR=~/libtorch' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/libtorch/lib' >> ~/.bashrc

cd ~/ros2_ws
MAKEFLAGS="-j4" colcon build --packages-up-to rl_quadruped_controller --symlink-install
```

## build mujoco
mujoco
```bash
sudo apt install libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libyaml-cpp-dev

# mujoco
cd ~/jazzy_ws/src
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco
git checkout d77ecb3f064678cc9227cdb8e0f5fccad97569fc
mkdir build && cd build
cmake ..
make -j4
sudo make install
simulate # mujoco test
```

unitree mujoco
```bash
# unitree_sdk2
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
git checkout 3d7f4a57c169420301cfa12479789e5142342d8e  # may useful
mkdir build && cd build
cmake ..
make -j4
sudo make install

# unitree_mujoco*
sudo apt update
sudo apt install libglfw3 libglfw3-dev

git clone https://github.com/legubiao/unitree_mujoco
git checkout 1fda65800287c369001da5e07ebd4652470f22f3
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4

# hardware_unitree_sdk2
MAKEFLAGS="-j4" colcon build --packages-up-to hardware_unitree_sdk2
```

for quick
```bash
# 需要将配置文件修改成绝对路径
echo "alias unitree_mujoco="~/jazzy_ws/src/unitree_mujoco/simulate/build/unitree_mujoco"" >> ~/.bashrc
```

## start
```bash
# 终端1
# base
ros2 launch unitree_guide_controller gazebo.launch.py
# mpc
ros2 launch ocs2_quadruped_controller gazebo.launch.py pkg_description:=go2_description
# rl
ros2 launch rl_quadruped_controller gazebo.launch.py pkg_description:=go2_description

# 终端2
ros2 run keyboard_input keyboard_input
```

## start mujoco
```bash
# 终端1 mujoco
unitree_mujoco

# 终端2 mpc
ros2 launch ocs2_quadruped_controller mujoco.launch.py

# 终端3
ros2 run keyboard_input keyboard_input
```

## start dds->ros2
```bash
ros2 run hardware_unitree_ros2 dds_ros2_bridge_node --ros-args -p domain:=19
```

demo
![](resource/demo.gif)

## arm build question
- hpipm_coclon
针对 lubancat rk3588、 ubuntu 24.04  
for 'hpipm_colcon', fix CMakeLists.txt: 
```bash
# fix1
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    message(STATUS "Building for Intel x86_64 architecture")
    set(TARGET "AVX")
    set(GIT_TAG "255ffdf38d3a5e2c3285b29568ce65ae286e5faf")
elseif(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
    message(STATUS "Building for ARM64 architecture")
    set(TARGET "GENERIC")
    set(GIT_TAG "255ffdf38d3a5e2c3285b29568ce65ae286e5faf")
else()
    message(STATUS "Building for unknown architecture")
    set(TARGET "GENERIC")
endif()

## fix2
set(TARGET "GENERIC" CACHE STRING "Target architecture" FORCE)
FetchContent_Declare(hpipmDownload
        GIT_REPOSITORY https://github.com/giaf/hpipm
        GIT_TAG ${GIT_TAG}
        UPDATE_COMMAND ""
        SOURCE_DIR ${HPIPM_DOWNLOAD_DIR}
        BINARY_DIR ${HPIPM_BUILD_DIR}
        BUILD_COMMAND $(MAKE)  TARGET=${TARGET}
        INSTALL_COMMAND "$(MAKE) install"
)
FetchContent_MakeAvailable(hpipmDownload)
```

build
```bash
MAKEFLAGS="-j1" colcon build --packages-up-to hpipm_colcon
```

- unitree_sdk2
uninstall
```bash
cd build
sudo xargs rm -f < install_manifest.txt
```
can't find '**.so'
```bash
# add to ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/unitree_robotics/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
```

- ocs2
```bash
# 正常
[ros2_control_node-3] [CppAdInterface] Loading Shared Library: /home/jazzy/ocs2_cpp_ad/go2/dynamics_systemFlowMap/cppad_generated/dynamics_systemFlowMap_lib.so

# 不正常
[ros2_control_node-3] [CppAdInterface] Compiling Shared Library: /home/cat/ocs2_cpp_ad/go2/dynamics_systemFlowMap/cppad_generated/dynamics_systemFlowMap_libcppadcg_tmp-89110158.so
```