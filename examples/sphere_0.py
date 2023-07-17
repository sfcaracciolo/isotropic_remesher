import numpy as np 
import geometric_plotter
from half_edge import HalfEdgeModel
from src.isotropic_remesher import IsotropicRemesher
from open3d.geometry import TriangleMesh

sphere = TriangleMesh().create_sphere(radius=1, resolution=15)
mesh = HalfEdgeModel(sphere.vertices, sphere.triangles)

remesher = IsotropicRemesher(mesh)
remesher.isotropic_remeshing(
    .14, 
    iter=20, 
    explicit=False, 
    foldover=0,
    sliver=False
)

remesher.model.clean()

geometric_plotter.set_export()

ax = geometric_plotter.figure(figsize=(5,5))

geometric_plotter.plot_trisurf(ax, np.asarray(remesher.model.vertices), np.asarray(remesher.model.triangles), color='k')

geometric_plotter.config_ax(ax, (50,50,0), 1.5)

geometric_plotter.execute(folder='E:\Repositorios\isotropic_remesher\export\\', name='sphere_0')