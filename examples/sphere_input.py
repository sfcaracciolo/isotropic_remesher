import numpy as np 
import geometric_plotter
from open3d.geometry import TriangleMesh

sphere = TriangleMesh().create_sphere(radius=1, resolution=15)

geometric_plotter.set_export()

ax = geometric_plotter.figure(figsize=(5,5))

geometric_plotter.plot_trisurf(ax, np.asarray(sphere.vertices), np.asarray(sphere.triangles), color='k')

geometric_plotter.config_ax(ax, (50,50,0), 1.5)

geometric_plotter.execute(folder='E:\Repositorios\isotropic_remesher\export\\', name='sphere_input')