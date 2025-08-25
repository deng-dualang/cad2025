from controller import Supervisor, Keyboard
import time
import random
import numpy as np

supervisor = Supervisor()
keyboard = Keyboard()
keyboard.enable(int(supervisor.getBasicTimeStep()))

sphere_radius = 0.1
waiting_ball_def = None  # 記錄目前等待擊出的球的 DEF 名稱

def axis_angle_to_rotation_matrix(axis, angle):
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c
    return np.array([
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c]
    ])

def generate_valid_def_name(base_name="Sphere"):
    timestamp = int(supervisor.getTime() * 1000)
    return f"{base_name}_{timestamp}"

def generate_random_color():
    r = random.random()
    g = random.random()
    b = random.random()
    return r, g, b

def youbot_local_to_world(local_pos):
    youbot_node = supervisor.getFromDef('youbot')
    if youbot_node is None:
        raise RuntimeError("找不到 DEF 為 youbot 的 Robot 物件")
    youbot_translation = np.array(youbot_node.getField('translation').getSFVec3f())
    youbot_rotation = youbot_node.getField('rotation').getSFRotation()
    youbot_axis = youbot_rotation[:3]
    youbot_angle = youbot_rotation[3]
    youbot_rot_mat = axis_angle_to_rotation_matrix(youbot_axis, youbot_angle)
    rotated = youbot_rot_mat @ np.array(local_pos)
    world_pos = youbot_translation + rotated
    return tuple(world_pos)

def create_static_ball(def_name, world_pos, r, g, b):
    # 不給 physics 節點 = static
    sphere_string = f"""
    DEF {def_name} Solid {{
      translation {world_pos[0]} {world_pos[1]} {world_pos[2]}
      contactMaterial "ball"
      children [
        Shape {{
          geometry Sphere {{
            radius {sphere_radius}
          }}
          appearance Appearance {{
            material Material {{
              diffuseColor {r} {g} {b}
            }}
          }}
        }}
      ]
      boundingObject Sphere {{
        radius {sphere_radius}
      }}
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, sphere_string)

def create_dynamic_ball(def_name, world_pos, r, g, b):
    # 有 physics 節點 = dynamic
    sphere_string = f"""
    DEF {def_name} Solid {{
      translation {world_pos[0]} {world_pos[1]} {world_pos[2]}
      contactMaterial "ball"
      children [
        Shape {{
          geometry Sphere {{
            radius {sphere_radius}
          }}
          appearance Appearance {{
            material Material {{
              diffuseColor {r} {g} {b}
            }}
          }}
        }}
      ]
      boundingObject Sphere {{
        radius {sphere_radius}
      }}
      physics Physics {{
        mass 0.01
        density -1
      }}
    }}
    """
    root = supervisor.getRoot()
    children_field = root.getField("children")
    children_field.importMFNodeFromString(-1, sphere_string)

def create_static_sphere(supervisor, x, y, z):
    global waiting_ball_def, waiting_ball_info
    def_name = generate_valid_def_name()
    waiting_ball_def = def_name  # 記錄 DEF
    r, g, b = generate_random_color()
    world_pos = youbot_local_to_world((x, y, z))
    # 存顏色與位置給動態球用
    waiting_ball_info = (world_pos, r, g, b)
    create_static_ball(def_name, world_pos, r, g, b)
    print(f"Static (non-physics) sphere created at youbot-relative {x,y,z} (world: {world_pos}) with DEF name: {def_name} and color: ({r:.2f}, {g:.2f}, {b:.2f})")

def activate_dynamic_ball():
    global waiting_ball_def, waiting_ball_info
    if waiting_ball_def is None or waiting_ball_info is None:
        print("No waiting ball to activate.")
        return
    # 刪除舊的 static 球
    ball_node = supervisor.getFromDef(waiting_ball_def)
    if ball_node is not None:
        ball_node.remove()
        supervisor.step(int(supervisor.getBasicTimeStep()))
    # 產生 dynamic 球
    world_pos, r, g, b = waiting_ball_info
    create_dynamic_ball(waiting_ball_def, world_pos, r, g, b)
    print(f"Ball {waiting_ball_def} is now dynamic!")
    waiting_ball_def = None
    waiting_ball_info = None

last_key_time = 0
debounce_time = 0.5
default_feed_pos = (-0.35, 0.0, 0.9)  # 可依需求調整
waiting_ball_info = None

print("按 A 產生一顆靜止的球（無 Physics），按 M 讓球變 dynamic 可以被擊出")

while supervisor.step(int(supervisor.getBasicTimeStep())) != -1:
    key = keyboard.getKey()
    current_time = time.time()
    if key == ord('A') and (current_time - last_key_time >= debounce_time):
        if waiting_ball_def is None:
            create_static_sphere(supervisor, *default_feed_pos)
        else:
            print("還有一顆球等待擊出，請先擊出再產生新球。")
        last_key_time = current_time
    if key == ord('M') and (current_time - last_key_time >= debounce_time):
        activate_dynamic_ball()
        last_key_time = current_time