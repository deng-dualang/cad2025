# pip install trimesh numpy networkx fast_simplification
import trimesh
import numpy as np
import os

def check_repair_simplify_obj(input_path, output_dir, temp_dir, simplify_ratio=0.5):
    """
    檢查、修復並簡化 .obj 文件，模擬分開版本的流程。
    
    參數：
        input_path (str): 輸入的 .obj 文件路徑
        output_dir (str): 修復且簡化後的輸出目錄
        temp_dir (str): 修復後的中間文件儲存目錄
        simplify_ratio (float): 簡化比例（0 到 1，1 表示不簡化，0.5 表示減少到 50% 面數）
    """
    try:
        # 確保輸出目錄和臨時目錄存在
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        if not os.path.exists(temp_dir):
            os.makedirs(temp_dir)
        
        # 獲取文件名
        filename = os.path.basename(input_path)
        temp_path = os.path.join(temp_dir, filename.replace(".obj", "_repaired.obj"))
        output_path = os.path.join(output_dir, filename.replace(".obj", "_repaired_simplified.obj"))
        
        # 載入 .obj 文件
        mesh = trimesh.load(input_path, file_type='obj', force='mesh')
        
        # 檢查基本屬性
        print(f"\n=== 正在檢查文件: {input_path} ===")
        print(f"頂點數: {len(mesh.vertices)}")
        print(f"面數: {len(mesh.faces)}")
        print(f"是否閉合 (watertight): {mesh.is_watertight}")
        print(f"是否為單一體積: {mesh.is_volume}")
        print(f"邊界框尺寸: {mesh.extents}")
        
        # 如果模型非閉合，嘗試修復
        if not mesh.is_watertight:
            print("警告: 模型非閉合，正在嘗試修復...")
            
            # 方法 1: 填充孔洞
            try:
                mesh.fill_holes()
                print(f"填充孔洞後是否閉合: {mesh.is_watertight}")
            except Exception as e:
                print(f"填充孔洞失敗: {str(e)}")
            
            # 方法 2: 修復非流形邊緣
            if not mesh.is_watertight:
                try:
                    trimesh.repair.fix_non_manifold(mesh)
                    print(f"修復非流形邊緣後是否閉合: {mesh.is_watertight}")
                except Exception as e:
                    print(f"修復非流形邊緣失敗: {str(e)}")
            
            # 方法 3: 統一法線並再次填充
            if not mesh.is_watertight:
                try:
                    trimesh.repair.fix_normals(mesh)
                    mesh.fill_holes()
                    print(f"統一法線並再次填充孔洞後是否閉合: {mesh.is_watertight}")
                except Exception as e:
                    print(f"統一法線並填充孔洞失敗: {str(e)}")
            
            # 儲存修復後的中間文件
            mesh.export(temp_path, file_type='obj')
            print(f"修復後的模型已儲存至中間文件: {temp_path}")
            
            # 重新載入修復後的模型
            if os.path.exists(temp_path):
                mesh = trimesh.load(temp_path, file_type='obj', force='mesh')
                print(f"重新載入修復後模型，是否閉合: {mesh.is_watertight}")
                if not mesh.is_watertight:
                    print("警告: 修復後模型仍非閉合，建議使用 Blender 或 MeshLab 進行手動修復")
            else:
                print(f"錯誤: 無法生成中間文件 {temp_path}")
                return
        else:
            print("模型已經閉合，無需修復，直接儲存到中間文件")
            mesh.export(temp_path, file_type='obj')
            mesh = trimesh.load(temp_path, file_type='obj', force='mesh')
        
        # 簡化模型
        if simplify_ratio < 1.0:
            print(f"正在簡化模型，目標面數比例: {simplify_ratio}")
            target_face_count = max(10, int(len(mesh.faces) * simplify_ratio))  # 確保至少保留 10 個面
            
            try:
                simplified_mesh = mesh.simplify_quadric_decimation(face_count=target_face_count)
                
                # 檢查簡化後的模型
                print(f"簡化後頂點數: {len(simplified_mesh.vertices)}")
                print(f"簡化後面數: {len(simplified_mesh.faces)}")
                print(f"簡化後是否閉合: {simplified_mesh.is_watertight}")
                
                # 如果簡化後非閉合，嘗試修復
                if not simplified_mesh.is_watertight:
                    print("警告: 簡化後模型非閉合，嘗試修復...")
                    try:
                        simplified_mesh.fill_holes()
                        print(f"修復後是否閉合: {simplified_mesh.is_watertight}")
                    except Exception as e:
                        print(f"簡化後修復失敗: {str(e)}")
                
                # 儲存簡化後的模型
                simplified_mesh.export(output_path, file_type='obj')
                print(f"修復且簡化後的模型已儲存至: {output_path}")
            except Exception as e:
                print(f"簡化模型時發生錯誤: {str(e)}")
                print("將儲存修復後的模型（未簡化）")
                mesh.export(output_path, file_type='obj')
        else:
            print("未進行簡化，將修復後的模型儲存至輸出路徑")
            mesh.export(output_path, file_type='obj')
        
        # 檢查最終輸出文件
        if os.path.exists(output_path):
            print(f"輸出文件 {output_path} 已成功生成")
            final_mesh = trimesh.load(output_path, file_type='obj', force='mesh')
            print(f"最終輸出模型是否閉合: {final_mesh.is_watertight}")
            print(f"最終頂點數: {len(final_mesh.vertices)}")
            print(f"最終面數: {len(final_mesh.faces)}")
        else:
            print(f"錯誤: 無法生成輸出文件 {output_path}")
            
    except Exception as e:
        print(f"處理文件時發生錯誤: {str(e)}")
        print("建議檢查文件路徑或使用 Blender/MeshLab 進行手動修復")

# 使用範例
if __name__ == "__main__":
    # 使用者輸入零件數量
    n_parts = 23

    
    # 輸入、臨時和輸出目錄
    input_dir = "./split_parts"  # 零件所在目錄
    temp_dir = "./repaired"  # 修復後的中間文件目錄
    output_dir = "./checked_repaired_simplified"  # 最終輸出目錄
    
    # 設置簡化比例
    simplify_ratio = 0.5
    
    # 處理 part_1.obj 到 part_n.obj
    for i in range(1, n_parts + 1):
        input_obj = os.path.join(input_dir, f"part_{i}.obj")
        if os.path.exists(input_obj):
            check_repair_simplify_obj(input_obj, output_dir, temp_dir, simplify_ratio)
        else:
            print(f"錯誤: 文件 {input_obj} 不存在，跳過處理")