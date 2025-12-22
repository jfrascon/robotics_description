The script `create_wheel.py` creates the mesh `wheel.dae` that you can see in the png file [wheel.png](wheel.png).
However, the created mesh although is displayed in **RViz2** correctly with colors, **Gazebo** does not display the colors of the mesh.
To overcome this situation, all you have to do, is to open then file `wheel.dae` in **Blender** and export it again as `wheel.dae`.
**Blender** automatically fixes the mesh file so that **Gazebo** can display it with colors.
