import bpy
import os
import json

# =================================================================
# === 使用者設定 (User Settings) ===
# =================================================================

# 設定要匯入的零件格式 ("stl" 或 "obj")
parts_format = "obj"

# 設定要匯出的格式 ("stl" 或 "obj")
export_format = "obj"

# 簡化網格的比例 (0.0 到 1.0 之間，值越小，模型面數越少)
# 例如：0.3 表示將面數減少到原本的 30%
simplify_ratio = 0.3

# =================================================================
# === 自動路徑設定 (Automatic Path Configuration) ===
# =================================================================

# 檢查腳本是否在 Blender 內執行並已儲存
if bpy.data.filepath:
    # 自動獲取腳本所在的目錄
    script_dir = os.path.dirname(bpy.data.filepath)
else:
    # 如果腳本是直接貼在文字編輯器中未儲存，則使用一個預設路徑
    # 警告：為了讓腳本正常運作，建議先將此 .py 檔案儲存
    script_dir = "C:\\Users\\yen\\Downloads\\portable_2026\\data\\tmp\\cad2025\\downloads\\blender"
    print("⚠️ 警告：腳本未儲存，使用預設路徑。建議先儲存腳本檔。")


# 設定相關資料夾路徑
parts_folder = os.path.join(script_dir, "split_parts")
export_folder = os.path.join(script_dir, "simplified_export")
output_json = os.path.join(script_dir, "positions.json")

print(f"📂 腳本目錄: {script_dir}")
print(f"📂 零件資料夾: {parts_folder}")
print(f"📂 匯出資料夾: {export_folder}")


# =================================================================
# === 核心功能函數 (Core Functions) ===
# =================================================================

def clear_scene():
    """清空當前場景中的所有網格物件。"""
    if bpy.ops.object.mode_set.poll():
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    print("🧹 場景已清空。")

def import_all_stl(folder_path):
    """從指定資料夾匯入所有 STL 檔案。"""
    imported_objects = []
    if not os.path.isdir(folder_path):
        print(f"❌ 錯誤：找不到資料夾 {folder_path}")
        return imported_objects

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(".stl"):
            filepath = os.path.join(folder_path, filename)
            objects_before = set(bpy.data.objects)
            bpy.ops.import_mesh.stl(filepath=filepath)
            new_obj_set = set(bpy.data.objects) - objects_before
            if new_obj_set:
                new_obj = new_obj_set.pop()
                new_obj.name = os.path.splitext(filename)[0]
                imported_objects.append(new_obj)
    return imported_objects

def import_all_obj(folder_path):
    """從指定資料夾匯入所有 OBJ 檔案。如果單一 OBJ 檔包含多個物件，會將它們合併。"""
    imported_objects = []
    if not os.path.isdir(folder_path):
        print(f"❌ 錯誤：找不到資料夾 {folder_path}")
        return imported_objects

    for filename in os.listdir(folder_path):
        if filename.lower().endswith(".obj"):
            filepath = os.path.join(folder_path, filename)
            objects_before = set(bpy.data.objects)
            bpy.ops.wm.obj_import(filepath=filepath)
            new_objs = list(set(bpy.data.objects) - objects_before)
            if new_objs:
                if len(new_objs) > 1:
                    bpy.ops.object.select_all(action='DESELECT')
                    for obj in new_objs:
                        obj.select_set(True)
                    bpy.context.view_layer.objects.active = new_objs[0]
                    bpy.ops.object.join()
                final_obj = bpy.context.view_layer.objects.active
                final_obj.name = os.path.splitext(filename)[0]
                imported_objects.append(final_obj)
    return imported_objects

def save_positions_to_json(obj_list, json_path):
    """擷取物件清單中每個物件的中心點位置，並儲存為 JSON 檔案。"""
    positions = {}
    for obj in obj_list:
        loc = obj.location
        positions[obj.name] = [round(loc.x, 6), round(loc.y, 6), round(loc.z, 6)]
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(positions, f, indent=4, ensure_ascii=False)
    print(f"✅ 零件位置已儲存至 {json_path}")

def simplify_and_export(obj_list, ratio, format_type, output_folder):
    """
    【使用臨時場景策略】簡化物件網格，然後根據指定格式匯出。
    此方法可以繞過有問題的 'use_selection' 或 'export_scope' 參數。
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    print(f"✨ 開始簡化並匯出 {len(obj_list)} 個物件...")

    # 1. 儲存原始場景，以便後續切換回來
    original_scene = bpy.context.window.scene

    for obj in obj_list:
        if obj.type != 'MESH':
            continue
        
        # --- 網格簡化 (在原始場景中進行) ---
        bpy.context.view_layer.objects.active = obj
        if bpy.context.object.mode != 'OBJECT':
            bpy.ops.object.mode_set(mode='OBJECT')
        
        decimate_mod = obj.modifiers.new(name="DecimateMod", type='DECIMATE')
        decimate_mod.ratio = ratio
        bpy.ops.object.modifier_apply(modifier=decimate_mod.name)
        
        # --- 開始臨時場景匯出流程 ---
        export_path = os.path.join(output_folder, f"{obj.name}_simplified.{format_type}")
        
        # 2. 建立一個新的臨時場景
        temp_scene = bpy.data.scenes.new(name="TempExportScene")
        
        # 3. 將我們想匯出的單一物件連結到新場景的根集合中
        temp_scene.collection.objects.link(obj)
        
        # 4. 將視窗的當前場景切換到我們的臨時場景
        bpy.context.window.scene = temp_scene
        
        # 5. 執行匯出。因為此時場景中只有一個物件，所以等於只匯出該物件
        try:
            if format_type == "stl":
                # STL 匯出通常沒問題，但為求一致，也可用此安全模式
                bpy.ops.export_mesh.stl(filepath=export_path, use_selection=False) # 匯出整個場景 (其中只有一個物件)
            elif format_type == "obj":
                # 不再使用任何有問題的參數，直接匯出當前場景
                bpy.ops.wm.obj_export(filepath=export_path)
        finally:
            # 6. 無論成功或失敗，都必須切換回原始場景
            bpy.context.window.scene = original_scene
            # 7. 刪除臨時場景，清理環境
            bpy.data.scenes.remove(temp_scene)
            
    print(f"📦 已將所有簡化零件匯出為 {format_type.upper()} 至 {output_folder}")


# =================================================================
# === 主流程 (Main Execution) ===
# =================================================================

def run_full_pipeline():
    """執行完整的處理流程。"""
    print("🚀 開始執行完整處理流程...")
    
    clear_scene()
    
    objs = []
    if parts_format.lower() == "stl":
        objs = import_all_stl(parts_folder)
    elif parts_format.lower() == "obj":
        objs = import_all_obj(parts_folder)
    else:
        print(f"❌ 錯誤：不支援的匯入格式 '{parts_format}'。請使用 'stl' 或 'obj'。")
        return

    if not objs:
        print("🤷‍ 沒有匯入任何物件，流程中止。請檢查零件資料夾是否正確或內含檔案。")
        return

    print(f"📥 匯入完成，共 {len(objs)} 個物件。")

    save_positions_to_json(objs, output_json)
    
    simplify_and_export(objs, simplify_ratio, export_format.lower(), export_folder)
    
    print("🎉 所有流程已成功完成！")

# --- 執行腳本 ---
if __name__ == "__main__":
    run_full_pipeline()