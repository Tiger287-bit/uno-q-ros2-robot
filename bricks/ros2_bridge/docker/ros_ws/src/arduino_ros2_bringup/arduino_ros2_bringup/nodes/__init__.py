"""节点层：一个文件一个节点，可被 `ros2 run` 单独拉起。

每个模块都必须提供 main()，并在 setup.py 的 entry_points 中登记，
这样 `ros2 run arduino_ros2_bringup <名字>` 与 launch 里的
`Node(package=..., executable=...)` 走的是同一入口。
"""
