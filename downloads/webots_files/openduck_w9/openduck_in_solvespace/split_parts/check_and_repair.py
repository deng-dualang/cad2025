# pip install trimesh numpy networkx
import trimesh
import numpy as np
import os

def check_and_fix_obj(input_path, output_path):
    """
    檢查並修復非閉合的 .obj 文件，嘗試使其成為閉合模型。
    
    參數：
        input_path (str): 輸入的 .obj 文件路徑
        output_path (str): 修復後的 .obj 文件輸出路徑
    """
    try:
        # 載入 .obj 文件
        mesh = trimesh.load(input_path, file_type='obj', force='mesh')
        
        # 檢查基本屬性
        print(f"正在檢查文件: {input_path}")
        print(f"頂點數: {len(mesh.vertices)}")
        print(f"面數: {len(mesh.faces)}")
        print(f"是否閉合 (watertight): {mesh.is_watertight}")
        print(f"邊界框尺寸: {mesh.extents}")
        
        # 如果模型已經閉合，無需修復
        if mesh.is_watertight:
            print("模型已經閉合，無需修復，直接儲存到輸出路徑")
            mesh.export(output_path, file_type='obj')
            return
        
        # 如果模型非閉合，嘗試修復
        print("警告: 模型非閉合，正在嘗試修復...")
        
        # 方法 1: 嘗試填充孔洞
        try:
            mesh.fill_holes()
            print(f"填充孔洞後是否閉合: {mesh.is_watertight}")
        except Exception as e:
            print(f"填充孔洞失敗: {str(e)}")
        
        # 方法 2: 如果填充孔洞仍不閉合，嘗試修復非流形邊緣
        if not mesh.is_watertight:
            try:
                trimesh.repair.fix_non_manifold(mesh)
                print(f"修復非流形邊緣後是否閉合: {mesh.is_watertight}")
            except Exception as e:
                print(f"修復非流形邊緣失敗: {str(e)}")
        
        # 方法 3: 如果仍不閉合，嘗試統一法線並再次填充
        if not mesh.is_watertight:
            try:
                trimesh.repair.fix_normals(mesh)
                mesh.fill_holes()
                print(f"統一法線並再次填充孔洞後是否閉合: {mesh.is_watertight}")
            except Exception as e:
                print(f"統一法線並填充孔洞失敗: {str(e)}")
        
        # 儲存修復後的模型
        mesh.export(output_path, file_type='obj')
        print(f"修復後的模型已儲存至: {output_path}")
        
        # 檢查輸出文件是否成功生成
        if os.path.exists(output_path):
            print(f"輸出文件 {output_path} 已成功生成")
            # 重新載入修復後的模型並檢查是否閉合
            repaired_mesh = trimesh.load(output_path, file_type='obj')
            print(f"修復後模型是否閉合: {repaired_mesh.is_watertight}")
        else:
            print(f"錯誤: 無法生成輸出文件 {output_path}")
            
    except Exception as e:
        print(f"處理文件時發生錯誤: {str(e)}")
        print("建議使用 Blender 或 MeshLab 進行手動修復")

# 使用範例
if __name__ == "__main__":
    # 輸入和輸出文件路徑
    input_obj = "part_1.obj"  # 替換為你的 .obj 文件路徑
    output_obj = "part_1_repaired.obj"  # 修復後的輸出路徑
    
    # 執行檢查和修復
    check_and_fix_obj(input_obj, output_obj)