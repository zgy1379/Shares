# Shares
用于分享交流知识和代码


使用前将astar_planner中的astar_param.yaml放入ucar_navigation的config目录下
并在move_base.launch中添加
```
<rosparam file="$(find ucar_navigation)/config/astar_param.yaml" command="load" />
```
