#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: 2025-02-09 15:40:51 UTC
Author: mdecycu
Description: STL file converter that splits STL files into multiple OBJ files with MTL materials
"""

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

    def _write_mtl(self, filename, material_name):
        """寫入 MTL 材質檔案"""
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(f"newmtl {material_name}\n")
            f.write("Ka 0.2 0.2 0.2\n")  # Ambient color
            f.write("Kd 0.8 0.8 0.8\n")  # Diffuse color
            f.write("Ks 0.5 0.5 0.5\n")  # Specular color
            f.write("Ns 50.0\n")         # Specular exponent
            f.write("d 1.0\n")           # Transparency (1.0 = opaque)
            f.write("illum 2\n")         # Illumination model

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
        
        # Generate material name from the obj filename
        material_name = filename.stem
        # Create MTL filename using Path
        mtl_filename = filename.with_suffix('.mtl')
        
        # Write OBJ file
        with open(filename, 'w', encoding='utf-8') as f:
            # Reference the MTL file
            f.write(f"mtllib {mtl_filename.name}\n")
            f.write(f"usemtl {material_name}\n\n")
            
            for v in vertex_list:
                f.write(f"v {v[0]} {v[1]} {v[2]}\n")
            
            for n in normal_list:
                f.write(f"vn {n[0]} {n[1]} {n[2]}\n")
            
            for i, face in enumerate(faces):
                f.write(f"f {face[0]}//{i+1} {face[1]}//{i+1} {face[2]}//{i+1}\n")
        
        # Write the corresponding MTL file
        self._write_mtl(mtl_filename, material_name)
    
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
            print(f"  MTL: {obj_filename.with_suffix('.mtl')}")
        
        return len(components)

# 直接轉換指定的 STL 檔案
try:
    # 指定要轉換的 STL 檔案名稱和縮放比例
    stl_file = "plotter_onshape.stl"
    scale = 0.002  #2 倍原始 mm 尺寸設計

    # 創建轉換器實例並執行轉換
    converter = STLConverter(stl_file, scale=scale)
    num_parts = converter.split_and_convert()
    print(f"\n總共處理了 {num_parts} 個零件")
except Exception as e:
    print(f"錯誤: {e}")