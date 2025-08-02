# move_arm

    docker build --pull --no-cache -t crane_x7_humble .

でビルドする必要あり、ROS2の鍵が2025年6月にリセットされたため。
https://zenn.dev/tiryoh/articles/2025-06-02-ros-noetic-apt-get-update

# ROS2 humble対応レポジトリ
humbleじゃないとROS2の方がうまく動かない
https://github.com/rt-net/crane_x7_ros/tree/humble


# ホスト側で X11 の接続を許可しておく
xhost +local:root

# コンテナ起動

    docker run --rm -it --network host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name crane_x7_dev crane_x7_humble

# setup.bash

    source /opt/ros/humble/setup.bash

# サンプルプログラム
https://github.com/rt-net/crane_x7_ros/blob/master/crane_x7_examples/README.md


