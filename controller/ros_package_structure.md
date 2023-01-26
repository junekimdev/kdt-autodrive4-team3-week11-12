# Package Structure

- `pkg-name/`
  - `include/`
    - `pkg-name/`
      - `include_file.hpp`
  - `launch/`
    - `launch_file.launch`
  - `msg/`
    - `msg.msg`
  - `pkg/`
    - `CMakefile.txt`
    - `package.xml`
  - `src/`
    - `src_file.cpp`

## How to create a ROS package

```shell
catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

And modify `CMakefile.txt` and `package.xml`

## How to build a ROS package

```shell
cd $HOME/$ws && catkin_make --only-pkg-with-deps <package_name>
```

## How to launch a ROS package

```shell
roslaunch <package_name> <launch_file>.launch
```

## Better way to run commands

```shell
# Inside a file: ".bashrc" or ".bash_alias"
## Build
alias cml="cd $HOME/$ws && catkin_make --only-pkg-with-deps <package_name>"
## Start
alias st="roslaunch <package_name> <launch_file>.launch"
```
