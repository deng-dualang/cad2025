# pip install trimesh numpy fast_simplification'
import trimesh
import numpy as np
import os

def check_and_simplify_obj(input_path, output_path, simplify_ratio=0.5):
    """
    檢查並簡化 .obj 文件。
    
    參數：
        input_path (str): 輸入的 .obj 文件路徑
        output_path (str): 簡化後的 .obj 文件輸出路徑
        simplify_ratio (float): 簡化比例（0 到 1，1 表示不簡化，0.5 表示減少到 50% 面數）
    """
    try:
        # 載入 .obj 文件
        mesh = trimesh.load(input_path, file_type='obj', force='mesh')
        
        # 檢查基本屬性
        print(f"正在檢查文件: {input_path}")
        print(f"頂點數: {len(mesh.vertices)}")
        print(f"面數: {len(mesh.faces)}")
        print(f"是否閉合 (watertight): {mesh.is_watertight}")
        print(f"是否為單一體積: {mesh.is_volume}")
        print(f"邊界框尺寸: {mesh.extents}")
        
        # 檢查模型是否有問題
        if not mesh.is_watertight:
            print("警告: 模型不是閉合的，可能導致 Webots 碰撞檢測問題！")
        
        # 簡化模型
        if simplify_ratio < 1.0:
            print(f"正在簡化模型，目標面數比例: {simplify_ratio}")
            target_face_count = max(10, int(len(mesh.faces) * simplify_ratio))  # 確保至少保留 10 個面
            
            # 使用 quadric 簡化演算法
            try:
                simplified_mesh = mesh.simplify_quadric_decimation(face_count=target_face_count)
                
                # 檢查簡化後的模型
                print(f"簡化後頂點數: {len(simplified_mesh.vertices)}")
                print(f"簡化後面數: {len(simplified_mesh.faces)}")
                print(f"簡化後是否閉合: {simplified_mesh.is_watertight}")
                
                # 如果簡化後模型非閉合，嘗試修復
                if not simplified_mesh.is_watertight:
                    print("警告: 簡化後模型非閉合，嘗試修復...")
                    simplified_mesh.fill_holes()
                    print(f"修復後是否閉合: {simplified_mesh.is_watertight}")
                
                # 儲存簡化後的模型
                simplified_mesh.export(output_path, file_type='obj')
                print(f"簡化後的模型已儲存至: {output_path}")
            except Exception as e:
                print(f"簡化模型時發生錯誤: {str(e)}")
                print("將儲存原始模型")
                mesh.export(output_path, file_type='obj')
        else:
            print("未進行簡化，將原始模型另存至輸出路徑")
            mesh.export(output_path, file_type='obj')
        
        # 檢查輸出文件是否成功生成
        if os.path.exists(output_path):
            print(f"輸出文件 {output_path} 已成功生成")
            # 重新載入並檢查
            repaired_mesh = trimesh.load(output_path, file_type='obj')
            print(f"最終輸出模型是否閉合: {repaired_mesh.is_watertight}")
        else:
            print(f"錯誤: 無法生成輸出文件 {output_path}")
            
    except Exception as e:
        print(f"處理文件時發生錯誤: {str(e)}")

# 使用範例
if __name__ == "__main__":
    # 輸入和輸出文件路徑
    input_obj = "part_1_repaired.obj"  # 你的 .obj 文件路徑
    output_obj = "part_1_simplified.obj"  # 簡化後的輸出路徑
    
    # 設置簡化比例（例如 0.5 表示減少到 50% 的面數）
    simplify_ratio = 0.5
    
    # 執行檢查和簡化
    check_and_simplify_obj(input_obj, output_obj, simplify_ratio)