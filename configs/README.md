# Dora configuration

`dataflow_tracking_real.yml` 是在 Pollen Robotics 上游 `Demo` 目录中使用的
Dora 数据流配置。它依赖上游的 `HandTracking`、`AHSimulation`、`AHControl`
和 Rust workspace，本仓库没有重复收录这些内容。

使用时请把此文件复制到上游仓库的 `Demo` 目录，并根据自己的串口和
`AHControl/config/r_hand.toml` 调整参数。

