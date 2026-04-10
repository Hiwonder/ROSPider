#!/bin/bash
source $HOME/.hiwonderrc

# 无论设置的 ROS_HOSTNAME 和 ROS_MASTER_URI 是什么
# 都改为 localhost, 避免手机app切到局域网模式时因为 ip 地址改变而用不了
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export DISPLAY=:0.0
exec "$@"
