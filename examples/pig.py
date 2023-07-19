from open3d.geometry import TriangleMesh, PointCloud
from open3d.utility import Vector3dVector
from open3d.visualization import draw_geometries
from half_edge import HalfEdgeModel
from src.isotropic_remesher import utils, IsotropicRemesher
from geometric_tools import triangle_mesh_by_convex_hull_of_inner_sphere
import zarr

ZARR_PATH = 'E:\db.zarr'
root = zarr.open(ZARR_PATH, mode='r')
nodes = root['/sinus_pig/data/torso/electrodes'][:]
vertices = Vector3dVector(nodes)

sphere_triangle_mesh, indices = triangle_mesh_by_convex_hull_of_inner_sphere(vertices)
new_vertices = Vector3dVector(nodes[indices])
pig_triangle_mesh = TriangleMesh(new_vertices, sphere_triangle_mesh.triangles)

model = HalfEdgeModel(pig_triangle_mesh.vertices, pig_triangle_mesh.triangles)
remesher = IsotropicRemesher(model)

L_min, L_mean, L_max = utils.compute_len_stats(pig_triangle_mesh.vertices)
print(f'L min = {L_min:.2f} L mean = {L_mean:.2f} L max = {L_max:.2f}')

L = 1*L_mean
ITER = 3

remesher.isotropic_remeshing(
    L, 
    iter=ITER, 
    explicit=True, 
    foldover=20,
    sliver=True
)

model.clean()
_triangle_mesh = TriangleMesh(model.vertices, model.triangles)
draw_geometries([PointCloud(vertices), _triangle_mesh], mesh_show_back_face=True, mesh_show_wireframe=True) 