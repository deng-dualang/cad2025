import bpy
import os
import json
from mathutils import Vector, Matrix
import time
import bmesh

# =================================================================
# === 使用者設定 (User Settings) ===
# =================================================================

# 設定要匯入的零件格式 ("stl" 或 "obj")
parts_format = "obj"

# 設定要匯出的格式 ("stl" 或 "obj")
export_format = "obj"

# 零件資料夾路徑，請根據你的需求設定
parts_folder = "C:\\Users\\yen\\Downloads\\portable_2026\\data\\tmp\\cad2025\\downloads\\blender\\split_parts"

# 匯出資料夾路徑，程式會自動建立
export_folder = "C:\\Users\\yen\\Downloads\\portable_2026\\data\\tmp\\cad2025\\downloads\\blender\\simplified_export"

# 可選：簡化凸包的細節（減少多邊形數量，0.0 表示不簡化，值越大簡化越多）
decimation_ratio = 0.2  # 範圍 0.0 到 1.0，1.0 表示不減少多邊形

# =================================================================
# === 核心功能函數 (Core Functions) ===
# =================================================================

def clear_scene():
    """
    清空當前場景中的所有網格物件。
    這個函數是為了確保每次運行程式時，場景都是乾淨的，避免舊的物件干擾。
    """
    # 確保程式處於物件模式，以便選擇和刪除物件
    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    print("場景已清空。")

def import_all_parts(folder_path, file_format):
    """
    從指定資料夾匯入所有指定格式的檔案。
    """
    imported_objects = []
    if not os.path.isdir(folder_path):
        print(f"錯誤：找不到資料夾 {folder_path}")
        return imported_objects

    for filename in os.listdir(folder_path):
        # 檢查檔案副檔名是否與設定的格式相符
        if filename.lower().endswith(f".{file_format}"):
            filepath = os.path.join(folder_path, filename)
            # 記錄匯入前的物件列表
            objects_before = set(bpy.data.objects)

            # 根據檔案格式執行不同的匯入操作
            if file_format == "stl":
                bpy.ops.import_mesh.stl(filepath=filepath)
            elif file_format == "obj":
                bpy.ops.wm.obj_import(filepath=filepath)
            
            # 找到新匯入的物件
            new_objs = list(set(bpy.data.objects) - objects_before)
            
            if new_objs:
                # 如果匯入的檔案包含了多個物件，則將它們合併成一個
                if len(new_objs) > 1:
                    bpy.ops.object.select_all(action='DESELECT')
                    for obj in new_objs:
                        obj.select_set(True)
                    bpy.context.view_layer.objects.active = new_objs[0]
                    bpy.ops.object.join()
                
                final_obj = bpy.context.view_layer.objects.active
                if final_obj:
                    # 為匯入的物件重新命名，以方便後續處理
                    final_obj.name = os.path.splitext(filename)[0]
                    imported_objects.append(final_obj)
                    print(f"成功匯入零件: {final_obj.name}")
    return imported_objects

def get_physics_properties(obj):
    """
    穩健地獲取物件的物理屬性，包含質量、位置和旋轉。
    這個函數的目的是在簡化模型之前，先記錄原始模型的物理屬性，以便簡化後能完全匹配。
    """
    if not obj:
        print(f"警告：物件無效，無法獲取物理屬性。")
        return 1.0, Vector((0, 0, 0)), Matrix.Identity(4)

    # 確保物件已添加剛體
    if not obj.rigid_body:
        bpy.ops.object.select_all(action='DESELECT')
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.rigidbody.object_add()

    # 設定為凸包，以確保物理引擎能正確計算其屬性
    obj.rigid_body.collision_shape = 'CONVEX_HULL'
    
    # 更新場景以確保物理屬性計算完成
    bpy.context.view_layer.update()
    
    mass = 1.0
    location = obj.location.copy()
    rotation = obj.matrix_world.copy()  # 獲取完整的變換矩陣（包含旋轉）
    
    try:
        mass = obj.rigid_body.mass
        print(f"  > 成功獲取 '{obj.name}' 的物理屬性。")
    except AttributeError:
        print(f"警告：無法獲取零件 '{obj.name}' 的物理屬性，將使用預設值。")
        
    return mass, location, rotation

def calculate_mesh_stats(obj):
    """
    計算物件的網格統計數據：頂點數、多邊形數、表面積、體積。
    此函數用於簡化前後的對比，讓使用者能清楚看到簡化效果。
    """
    if not obj or not obj.data:
        return 0, 0, 0.0, "無法計算 (無有效網格)"

    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')
    
    verts = len(obj.data.vertices)
    faces = len(obj.data.polygons)
    
    # 使用 BMesh 函式庫來計算更複雜的幾何屬性，例如面積和體積
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    
    # 計算總表面積
    area = sum(face.calc_area() for face in bm.faces)
    
    # 計算體積（需要網格是封閉的）
    try:
        volume = bm.calc_volume()
    except ValueError:
        volume = "無法計算 (網格可能未封閉)"
    
    bm.free()
    
    return verts, faces, area, volume

def compare_objects(original_obj, simplified_obj):
    """
    比較原始物件和簡化物件的統計數據並打印結果。
    """
    orig_verts, orig_faces, orig_area, orig_volume = calculate_mesh_stats(original_obj)
    simp_verts, simp_faces, simp_area, simp_volume = calculate_mesh_stats(simplified_obj)
    
    print(f"零件 '{original_obj.name}' 簡化前後比較：")
    print(f"    - 頂點數: 原始 {orig_verts} → 簡化 {simp_verts} (減少 {((orig_verts - simp_verts) / orig_verts * 100) if orig_verts > 0 else 0:.2f}%)")
    print(f"    - 多邊形數: 原始 {orig_faces} → 簡化 {simp_faces} (減少 {((orig_faces - simp_faces) / orig_faces * 100) if orig_faces > 0 else 0:.2f}%)")
    print(f"    - 表面積: 原始 {orig_area:.4f} → 簡化 {simp_area:.4f}")
    print(f"    - 體積: 原始 {orig_volume if isinstance(orig_volume, float) else orig_volume} → 簡化 {simp_volume if isinstance(simp_volume, float) else simp_volume}")

def create_and_match_primitive(original_obj_name, export_folder, export_format):
    """
    為每個原始物件創建一個簡化凸包，並匹配其物理屬性和旋轉。
    這是整個程式的核心步驟。
    """
    if not os.path.exists(export_folder):
        os.makedirs(export_folder)

    original_obj = bpy.data.objects.get(original_obj_name)
    if not original_obj:
        print(f"錯誤：找不到物件 '{original_obj_name}'，跳過處理。")
        return

    # 步驟1: 獲取原始物件的物理屬性
    original_mass, original_loc, original_rotation = get_physics_properties(original_obj)

    # 步驟2: 創建凸包
    bpy.ops.object.select_all(action='DESELECT')
    original_obj.select_set(True)
    bpy.context.view_layer.objects.active = original_obj
    bpy.ops.object.duplicate() # 複製原始物件
    simplified_obj = bpy.context.active_object
    simplified_obj.name = f"{original_obj_name}_simplified"

    # 進入編輯模式，並執行「凸包」運算
    bpy.ops.object.mode_set(mode='EDIT')
    try:
        bpy.ops.mesh.convex_hull()
    except Exception as e:
        print(f"無法生成凸包 '{simplified_obj.name}'：{str(e)}")
        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        original_obj.select_set(True)
        bpy.ops.object.delete() # 刪除原始物件，以避免重複
        return
    bpy.ops.object.mode_set(mode='OBJECT')

    # 步驟3: 可選的細節簡化
    if decimation_ratio < 1.0:
        bpy.ops.object.modifier_add(type='DECIMATE')
        simplified_obj.modifiers["Decimate"].ratio = decimation_ratio
        try:
            bpy.ops.object.modifier_apply(modifier="Decimate")
        except Exception as e:
            print(f"警告：無法應用簡化修飾器 '{simplified_obj.name}'：{str(e)}")

    # 步驟4: 匹配原始物件的物理屬性
    simplified_obj.location = original_loc
    simplified_obj.matrix_world = original_rotation

    # 賦予簡化物件剛體屬性，並設定其物理參數
    bpy.ops.rigidbody.object_add()
    simplified_obj.rigid_body.collision_shape = 'CONVEX_HULL'
    simplified_obj.rigid_body.mass = original_mass
    
    print(f"已為零件 '{original_obj_name}' 創建並匹配簡化凸包 '{simplified_obj.name}'。")

    # 步驟5: 簡化前後數據比較
    compare_objects(original_obj, simplified_obj)

    # 步驟6: 刪除原始物件
    bpy.ops.object.select_all(action='DESELECT')
    original_obj.select_set(True)
    bpy.ops.object.delete()

    # 步驟7: 匯出簡化後的物件
    export_path = os.path.join(export_folder, f"{simplified_obj.name}.{export_format}")
    bpy.ops.object.select_all(action='DESELECT')
    simplified_obj.select_set(True)
    
    try:
        if export_format == "obj":
            bpy.ops.wm.obj_export(
                filepath=export_path,
                export_selected_objects=True,
                check_existing=True,
                forward_axis='NEGATIVE_Z',
                up_axis='Y'
            )
        elif export_format == "stl":
            bpy.ops.export_mesh.stl(
                filepath=export_path,
                use_selection=True,
                check_existing=True
            )
        print(f"已匯出至: {export_path}")
    except Exception as e:
        print(f"匯出失敗 '{simplified_obj.name}'：{str(e)}")

# =================================================================
# === 主流程 (Main Execution) ===
# =================================================================

def run_full_pipeline():
    """執行完整的處理流程。"""
    print("開始執行完整處理流程...")
    
    clear_scene()
    
    # 依序執行各個步驟
    original_objs = import_all_parts(parts_folder, parts_format)
    
    if not original_objs:
        print("沒有匯入任何物件，流程中止。請檢查零件資料夾是否正確或內含檔案。")
        return

    print(f"匯入完成，共 {len(original_objs)} 個物件。")
    
    object_names = [obj.name for obj in original_objs]
    
    # 逐一處理每個匯入的物件
    for name in object_names:
        create_and_match_primitive(name, export_folder, export_format)
    
    print("所有流程已成功完成！")

# --- 執行程式 ---
# 確保程式只在直接執行時運行，而不是被其他程式匯入時運行
if __name__ == "__main__":
    run_full_pipeline()