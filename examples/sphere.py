from open3d.geometry import TriangleMesh, PointCloud
from open3d.visualization import draw_geometries
from half_edge import HalfEdgeModel
from src.isotropic_remesher import utils, IsotropicRemesher

sphere = TriangleMesh().create_sphere(radius=1, resolution=15)
mesh = HalfEdgeModel(sphere.vertices, sphere.triangles)

L_min, L_mean, L_max = utils.compute_len_stats(sphere.vertices)
print(f'L min = {L_min:.2f} L mean = {L_mean:.2f} L max = {L_max:.2f}')
remesher = IsotropicRemesher(mesh)
remesher.isotropic_remeshing(
    1.5*L_mean, 
    iter=20, 
    explicit=True, 
    foldover=10,
    sliver=False
)

# .8*L_mean
# iter=20, 
# explicit=False, 
# foldover=0,
# sliver=False

# 3.*L_mean
# iter=20, 
# explicit=False, 
# foldover=0
# sliver=False

remesher.model.clean()
_triangle_mesh = TriangleMesh(remesher.model.vertices, remesher.model.triangles)
new_vertices = PointCloud(remesher.model.vertices)
old_vertices = PointCloud(sphere.vertices)
draw_geometries([old_vertices, new_vertices, _triangle_mesh], mesh_show_back_face=True, mesh_show_wireframe=True) 