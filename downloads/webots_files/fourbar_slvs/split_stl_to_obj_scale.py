import struct
import numpy as np
from pathlib import Path

class STLConverter:
    def __init__(self, filename, scale=0.001):  # 新增 scale 參數，預設 0.001 將 mm 轉換為 m
        self.filename = filename
        self.scale = scale
        self.is_binary = self._check_if_binary()
        
    def _check_if_binary(self):
        """檢查 STL 檔案是否為二進制格式"""
        with open(self.filename, 'rb') as f:
            header = f.read(5).decode('utf-8', errors='ignore')
            return not header.startswith('solid')
    
    def _read_binary_stl(self):
        """讀取二進制 STL 檔案"""
        with open(self.filename, 'rb') as f:
            f.seek(80)
            triangle_count = struct.unpack('I', f.read(4))[0]
            
            triangles = []
            normals = []
            
            for _ in range(triangle_count):
                nx, ny, nz = struct.unpack('fff', f.read(12))
                normals.append([nx, ny, nz])
                
                triangle = []
                for _ in range(3):
                    x, y, z = struct.unpack('fff', f.read(12))
                    # 在讀取頂點時進行縮放
                    triangle.append([x * self.scale, y * self.scale, z * self.scale])
                triangles.append(triangle)
                
                f.seek(2, 1)
                
        return np.array(triangles), np.array(normals)
    
    def _read_ascii_stl(self):
        """讀取 ASCII STL 檔案"""
        triangles = []
        normals = []
        current_triangle = []
        
        with open(self.filename, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                    
                parts = line.split()
                if not parts:
                    continue
                
                if parts[0] == 'facet' and parts[1] == 'normal':
                    normals.append([float(parts[2]), float(parts[3]), float(parts[4])])
                elif parts[0] == 'vertex':
                    # 在讀取頂點時進行縮放
                    current_triangle.append([
                        float(parts[1]) * self.scale,
                        float(parts[2]) * self.scale,
                        float(parts[3]) * self.scale
                    ])
                elif parts[0] == 'endfacet':
                    if current_triangle:
                        triangles.append(current_triangle)
                        current_triangle = []
        
        return np.array(triangles), np.array(normals)
    
    def _split_by_connected_components(self, triangles, normals):
        """使用連通分量分割模型"""
        vertex_to_triangle = {}
        for i, triangle in enumerate(triangles):
            for vertex in triangle:
                vertex_tuple = tuple(vertex)
                if vertex_tuple in vertex_to_triangle:
                    vertex_to_triangle[vertex_tuple].append(i)
                else:
                    vertex_to_triangle[vertex_tuple] = [i]
        
        visited = set()
        components = []
        
        def dfs(triangle_idx):
            component = []
            stack = [triangle_idx]
            
            while stack:
                current = stack.pop()
                if current not in visited:
                    visited.add(current)
                    component.append(current)
                    
                    for vertex in triangles[current]:
                        vertex_tuple = tuple(vertex)
                        for neighbor in vertex_to_triangle[vertex_tuple]:
                            if neighbor not in visited:
                                stack.append(neighbor)
            
            return component
        
        for i in range(len(triangles)):
            if i not in visited:
                component = dfs(i)
                components.append(component)
        
        return components
    
    def _write_binary_stl(self, filename, triangles, normals):
        """寫入二進制 STL 檔案"""
        with open(filename, 'wb') as f:
            f.write(b'\x00' * 80)
            f.write(struct.pack('I', len(triangles)))
            
            for triangle, normal in zip(triangles, normals):
                f.write(struct.pack('fff', *normal))
                for vertex in triangle:
                    f.write(struct.pack('fff', *vertex))
                f.write(struct.pack('H', 0))

    def _write_obj(self, filename, triangles, normals):
        """寫入 OBJ 檔案"""
        vertex_dict = {}
        vertex_list = []
        normal_list = []
        faces = []
        
        for triangle, normal in zip(triangles, normals):
            face_indices = []
            
            for vertex in triangle:
                vertex_tuple = tuple(vertex)
                if vertex_tuple not in vertex_dict:
                    vertex_dict[vertex_tuple] = len(vertex_list) + 1
                    vertex_list.append(vertex)
                face_indices.append(vertex_dict[vertex_tuple])
            
            normal_list.append(normal)
            faces.append(face_indices)
        
        with open(filename, 'w') as f:
            for v in vertex_list:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            
            for n in normal_list:
                f.write(f"vn {n[0]} {n[1]} {n[2]}\n")
            
            for i, face in enumerate(faces):
                f.write(f"f {face[0]}//{i+1} {face[1]}//{i+1} {face[2]}//{i+1}\n")
    
    def split_and_convert(self):
        """分割 STL 檔案並轉換為 OBJ 格式"""
        if self.is_binary:
            triangles, normals = self._read_binary_stl()
        else:
            triangles, normals = self._read_ascii_stl()
        
        components = self._split_by_connected_components(triangles, normals)
        
        output_dir = Path('split_parts')
        output_dir.mkdir(exist_ok=True)
        
        for i, component in enumerate(components):
            component_triangles = triangles[component]
            component_normals = normals[component]
            
            base_name = f"part_{i + 1}"
            stl_filename = output_dir / f"{base_name}.stl"
            obj_filename = output_dir / f"{base_name}.obj"
            
            #self._write_binary_stl(stl_filename, component_triangles, component_normals)
            self._write_obj(obj_filename, component_triangles, component_normals)
            
            print(f"已儲存零件 {i + 1} 到:")
            #print(f"  STL: {stl_filename}")
            print(f"  OBJ: {obj_filename}")
        
        return len(components)

# 使用範例
if __name__ == "__main__":
    # 設定 scale=0.001 將單位從 mm 轉換為 m
    converter = STLConverter("fourbar_assembly.stl", scale=0.01)
    num_parts = converter.split_and_convert()
    print(f"\n總共處理了 {num_parts} 個零件")