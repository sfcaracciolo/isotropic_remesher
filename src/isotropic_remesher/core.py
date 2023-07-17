import numpy as np 
from half_edge import HalfEdgeModel
import vector_tools
import geometric_tools
from open3d.utility import Vector3dVector
from open3d.t.geometry import RaycastingScene

class IsotropicRemesher:

    def __init__(self, model: HalfEdgeModel) -> None:
        self.model = model
        self.initial_surface = geometric_tools.scene_surface(model.vertices, model.triangles)

    def has_foldover_triangles(self, normals_pre, normals_pos, threshold=30):
        normals = np.empty((2,3), dtype=np.float32)
        for n0, n1 in zip(normals_pre, normals_pos):
            normals[0, :], normals[1, :] = n0, n1
            if vector_tools.angle(normals) > threshold:
                return True
        return False

    def has_collinear_edges(self, h0_index: int):

        _, h2_index, _, h5_index = self.model.adjacent_half_edges(h0_index)

        v3_index, v0_index = self.model.get_vertex_indices(h2_index)
        v1_index, v2_index = self.model.get_vertex_indices(h5_index)

        vertices = self.model.get_vertices_by_indices([v1_index, v2_index, v3_index])
        if vector_tools.are_collinear(vertices): return True
        
        vertices = self.model.get_vertices_by_indices([v0_index, v1_index, v3_index])
        if vector_tools.are_collinear(vertices): return True

        return False

    def has_small_edges(self, h_index:int, L_high:float):
        # check that new edges are smaller than high.
        v0_index = self.model.get_start_vertex_index(h_index)
        th_index = self.model.get_twin_index(h_index)
        for vi in self.model.vertex_ring(th_index):
            vs = self.model.get_vertices_by_indices([vi, v0_index])
            if vector_tools.distance(vs) > L_high:
                return False
        return True 

    def has_collapse_connectivity(self, h_index:int):
        v0_ring = set(self.model.vertex_ring(h_index))
        th_index = self.model.get_twin_index(h_index)
        v1_ring = set(self.model.vertex_ring(th_index))
        return len(v0_ring.intersection(v1_ring)) == 2

    def has_flip_connectivity(self, h0_index: int):

        if self.model.valence(h0_index) == 3:
            return False
        
        h3_index = self.model.get_twin_index(h0_index)

        if self.model.valence(h3_index) == 3:
            return False

        return True

    def split_long_edges(self, threshold: float):
        
        n = 0
        E = self.model.amount_of_half_edges()

        for h0_index in range(E):

            if h0_index in self.model.unreferenced_half_edges:
                continue

            if self.model.edge_len(h0_index) <= threshold:
                continue
            
            self.model.edge_split(h0_index)

            n += 1

        return n 

    def collapse_short_edges(self, L_low, L_high, explicit:bool=False, foldover:float=0):
        n = 0
        E = self.model.amount_of_half_edges()
        skip = []

        for h0_index in range(E):

            # skip if twin already tested

            h3_index = self.model.get_twin_index(h0_index)

            if h3_index in skip:
                continue
            
            skip.append(h3_index)

            if h0_index in self.model.unreferenced_half_edges:
                continue
            
            if explicit:
                v1_index = self.model.get_end_vertex_index(h0_index)
                if  v1_index < self.model.n_vertices:
                    continue

            if self.model.edge_len(h0_index) >= L_low:
                continue

            if not self.has_small_edges(h0_index, L_high):
                continue

            # https://stackoverflow.com/questions/27049163/mesh-simplification-edge-collapse-conditions
            if not self.has_collapse_connectivity(h0_index):
                continue

            # Compute normals before collapse 
            if foldover > 0: normals_pre = list(self.model.normal_ring(h3_index))[1:-1] # exclude f0 and f1

            p_ring = self.model.edge_collapse(h0_index)

            # Compute normals after collapse 
            if foldover > 0: 
                normals_pos = map(self.model.triangle_normal, p_ring)
                if self.has_foldover_triangles(normals_pre, normals_pos, threshold=foldover):
                    self.model.revert_edge_collapse(p_ring)
                    continue

            n += 1

        return n

    def equalize_valences(self, sliver:bool=False, foldover:float=0):

        n = 0
        E = self.model.amount_of_half_edges()
        skip = []

        for h0_index in range(E):

            # skip if twin already tested

            h3_index = self.model.get_twin_index(h0_index)

            if h3_index in skip:
                continue
            
            skip.append(h3_index)
            
            if h0_index in self.model.unreferenced_half_edges:
                continue
            
            if not self.has_flip_connectivity(h0_index):
                continue 

            if self.has_collinear_edges(h0_index):
                continue 

            # Compute normals before flip
            if foldover > 0: normals_pre = ( self.model.triangle_normal(h0_index), self.model.triangle_normal(h3_index) )

            if sliver: deviation_compactness_pre = self.compactness_deviation(h0_index)

            deviation_valence_pre = self.valence_deviation(h0_index)
            
            self.model.edge_flip(h0_index)
            
            if foldover > 0:
                normals_pos = ( self.model.triangle_normal(h0_index), self.model.triangle_normal(h3_index) )

                if self.has_foldover_triangles(normals_pre, normals_pos, threshold=foldover):
                    self.model.edge_flip(h0_index)
                    continue

            deviation_valence_pos = self.valence_deviation(h0_index)

            if deviation_valence_pre < deviation_valence_pos:
                self.model.edge_flip(h0_index)
                continue
            
            if sliver:
                deviation_compactness_pos = self.compactness_deviation(h0_index)

                if deviation_compactness_pre < deviation_compactness_pos:
                    self.model.edge_flip(h0_index)
                    continue

            n += 1

        return n

    def vertex_relocation(self, explicit:bool=False):
        E = self.model.amount_of_half_edges()
        skip = []
        new_vertices = np.asarray(self.model.vertices).copy()

        for h_index in range(E):

            if h_index in self.model.unreferenced_half_edges:
                continue
            
            v_index = self.model.get_start_vertex_index(h_index)

            if v_index in skip:
                continue 

            if v_index in self.model.unreferenced_vertices:
                continue

            if explicit:
                if  v_index < self.model.n_vertices:
                    continue
            
            new_vertices[v_index] = self.tangential_smoothing(h_index)
            
            skip.append(v_index)

        return new_vertices

    def isotropic_remeshing(self, L:float, iter:int=20, explicit:bool=False, foldover:float=0, sliver:bool=False):
        
        
        L_low, L_high = 4/5.*L, 4/3.*L
        print(f'L low = {L_low:.2f} L target = {L:.2f} L high = {L_high:.2f}')

        for m in range(iter): 

            print(10*'=' + f' ITER {m} ' + 10*'=')

            s = self.split_long_edges(L_high)
            print(f'split long edges ({s})')

            c = self.collapse_short_edges(L_low, L_high, explicit, foldover)
            print(f'collapse short edges ({c})')
            
            f = self.equalize_valences(sliver, foldover)
            print(f'flip edges ({f})')

            new_vertices = self.vertex_relocation(explicit)
            print(f'tangential smoothing')

            self.model.vertices = self.project_to_surface(new_vertices, self.initial_surface)
            print(f'back to surface')

    def project_to_surface(self, vertices: Vector3dVector, surface: RaycastingScene) -> Vector3dVector:
        projected_vertices = geometric_tools.project_to_surface(np.asarray(vertices), surface)
        return Vector3dVector(projected_vertices)
    
    def tangential_smoothing(self, h_index:int):
        n = self.model.vertex_normal(h_index)
        q = self.model.mean_vertex(h_index)
        v = self.model.get_start_vertex_by_edge(h_index)
        return q + np.inner(n, (v - q)) * n

    def compactness_deviation(self, h_index:int):
        return sum([1-self.model.mean_compactness(i) for i in self.model.adjacent_half_edges(h_index)])

    def valence_deviation(self, h_index:int):
        return sum([abs(self.model.valence(i)-6) for i in self.model.adjacent_half_edges(h_index)])
