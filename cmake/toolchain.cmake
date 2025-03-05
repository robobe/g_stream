set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)  # Change to match your target architecture

# Set cross-compiler paths
set(CMAKE_C_COMPILER /home/user/cross_compilers/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /home/user/cross_compilers/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-g++)
# set(CMAKE_SYSROOT /home/user/rootfs/ubuntuRootFS)  # Set to your root filesystem
set(CMAKE_LIBRARY_PATH /home/user/rootfs/ubuntuRootFS/usr/lib/aarch64-linux-gnu/)
# Set where CMake should look for libraries and headers
set(CMAKE_FIND_ROOT_PATH 
    /home/user/rootfs/ubuntuRootFS/usr/include/python3.10 
    /home/user/rootfs/ubuntuRootFS/usr/include
    /home/user/rootfs/ubuntuRootFS
    /home/user/rootfs/ubuntuRootFS/usr 
    /home/user/rootfs/ubuntuRootFS/usr/local 
    /home/user/cross_compilers/aarch64--glibc--stable-2022.08-1/aarch64-buildroot-linux-gnu/sysroot/usr/lib)

#linker 
# set(CMAKE_EXE_LINKER_FLAGS "-L/home/user/cross_compilers/aarch64--glibc--stable-2022.08-1/aarch64-buildroot-linux-gnu/sysroot/usr/lib")
# Only search inside the target rootfs
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# ROS 2 paths
set(AMENT_PREFIX_PATH /home/user/rootfs/ubuntuRootFS/opt/ros/humble)
set(CMAKE_PREFIX_PATH ${AMENT_PREFIX_PATH})