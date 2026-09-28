#!/usr/bin/env python3
"""crane_x7_description の xacro から MuJoCo 用モデル (MJCF) を生成する.

使い方 (リポジトリのルートで):
    source /opt/ros/lyrical/setup.bash
    source ros2_ws/install/setup.bash
    .venv/bin/python mujoco/tools/urdf_to_mjcf.py

出力: mujoco/models/crane_x7/crane_x7.xml (ロボット単体), scene.xml (床・照明つき)
メッシュはコピーせず、crane_x7_description/meshes を相対パスで参照する.
"""

import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco

REPO_ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION_DIR = REPO_ROOT / 'ros2_ws/src/crane_x7_description'
XACRO_FILE = DESCRIPTION_DIR / 'urdf/crane_x7.urdf.xacro'
MESH_DIR = DESCRIPTION_DIR / 'meshes'
OUT_DIR = REPO_ROOT / 'mujoco/models/crane_x7'

ARM_JOINTS = [
    'crane_x7_shoulder_fixed_part_pan_joint',
    'crane_x7_shoulder_revolute_part_tilt_joint',
    'crane_x7_upper_arm_revolute_part_twist_joint',
    'crane_x7_upper_arm_revolute_part_rotate_joint',
    'crane_x7_lower_arm_fixed_part_joint',
    'crane_x7_lower_arm_revolute_part_joint',
    'crane_x7_wrist_joint',
]
GRIPPER_JOINT = 'crane_x7_gripper_finger_a_joint'
GRIPPER_MIMIC_JOINT = 'crane_x7_gripper_finger_b_joint'

# 手先の基準点 (crane_x7_gripper_base_link 座標系)
EE_BODY = 'crane_x7_gripper_base_link'
EE_OFFSET = [0.0, 0.0, 0.07]

# 位置制御アクチュエータのゲイン (Dynamixel の位置制御を近似)
ARM_KP = 100.0
GRIPPER_KP = 20.0
JOINT_DAMPING = 1.0
# 指は軽いので、アームと同じ減衰だと mimic の拘束越しに動きが極端に遅くなる
FINGER_DAMPING = 0.2
JOINT_ARMATURE = 0.01


def load_urdf() -> tuple[str, dict[str, float]]:
    urdf = subprocess.run(
        ['xacro', str(XACRO_FILE), 'use_mock_components:=true'],
        check=True, capture_output=True, text=True,
    ).stdout
    root = ET.fromstring(urdf)

    # アクチュエータの力の上限に使うため、URDF の effort 制限を控えておく
    efforts = {
        joint.get('name'): float(joint.find('limit').get('effort'))
        for joint in root.findall('joint')
        if joint.find('limit') is not None
    }

    # ros2_control / gazebo 用のタグは MuJoCo では不要
    for tag in ('ros2_control', 'gazebo'):
        for elem in root.findall(tag):
            root.remove(elem)

    # package:// を meshes からの相対パスに置き換える
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        mesh.set('filename', filename.replace('package://crane_x7_description/meshes/', ''))

    # MuJoCo の URDF 読み込み設定: 見た目用メッシュを残し、固定関節のリンクは結合しない
    ext = ET.SubElement(root, 'mujoco')
    ET.SubElement(ext, 'compiler', {
        'meshdir': str(MESH_DIR),
        'discardvisual': 'false',
        'fusestatic': 'false',
        'strippath': 'false',
        'balanceinertia': 'true',
    })
    return ET.tostring(root, encoding='unicode'), efforts


def build_spec() -> mujoco.MjSpec:
    urdf, efforts = load_urdf()
    spec = mujoco.MjSpec.from_string(urdf)
    spec.modelname = 'crane_x7'

    # URDF の visual と collision が両方 geom になるので、役割を分ける
    # visual: 衝突なし (group 2)、collision: 表示しない (group 3)
    # visual/wrist.stl と collision/wrist.stl のようにファイル名が同じだと、MuJoCo は
    # 1 つのメッシュにまとめてしまう。visual 由来の geom は density=0 になるのでそれで見分け、
    # collision 側には collision/ のメッシュを別名で割り当て直す
    for geom in spec.geoms:
        if geom.type != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        if geom.density == 0:
            geom.contype = 0
            geom.conaffinity = 0
            geom.group = 2
            continue
        geom.group = 3
        stem = Path(spec.mesh(geom.meshname).file).stem
        if not (MESH_DIR / 'collision' / f'{stem}.stl').exists():
            continue
        collision_name = f'collision_{stem}'
        if spec.mesh(collision_name) is None:
            mesh = spec.add_mesh()
            mesh.name = collision_name
            mesh.file = f'collision/{stem}.stl'
        geom.meshname = collision_name

    # 接触判定から外すリンクの組
    # - 2 本の指: 親子関係ではないので、閉じたときに指どうしが接触判定されてしまう
    # - 1 つ飛ばしのリンク: MuJoCo はメッシュを凸包で扱うため、関節を可動範囲の端まで曲げると
    #   実機では当たらないのに接触判定される
    for body1, body2 in [
        ('crane_x7_gripper_finger_a_link', 'crane_x7_gripper_finger_b_link'),
        ('crane_x7_upper_arm_revolute_part_link', 'crane_x7_lower_arm_revolute_part_link'),
        ('crane_x7_lower_arm_revolute_part_link', 'crane_x7_gripper_base_link'),
    ]:
        exclude = spec.add_exclude()
        exclude.bodyname1 = body1
        exclude.bodyname2 = body2

    for joint in spec.joints:
        is_finger = joint.name in (GRIPPER_JOINT, GRIPPER_MIMIC_JOINT)
        joint.damping[0] = FINGER_DAMPING if is_finger else JOINT_DAMPING
        joint.armature = JOINT_ARMATURE

    # アーム 7 軸 + グリッパーに位置制御アクチュエータを付ける
    for name in ARM_JOINTS + [GRIPPER_JOINT]:
        joint = spec.joint(name)
        kp = GRIPPER_KP if name == GRIPPER_JOINT else ARM_KP
        act = spec.add_actuator()
        act.name = name
        act.target = name
        act.trntype = mujoco.mjtTrn.mjTRN_JOINT
        act.set_to_position(kp=kp)
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.ctrlrange = joint.range
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.forcerange = [-efforts[name], efforts[name]]

    # 手先の基準点: 2 本の指先の中間 (ROS 側では crane_x7_gripper_base_link から同じオフセットで求める)
    ee_site = spec.body(EE_BODY).add_site()
    ee_site.name = 'ee_site'
    ee_site.pos = EE_OFFSET
    ee_site.size = [0.005, 0, 0]
    ee_site.rgba = [0, 1, 0, 1]
    ee_site.group = 4

    # URDF の mimic (finger_b は finger_a に連動) は MuJoCo の読み込み時に等式拘束へ変換される
    assert any(eq.name1 == GRIPPER_MIMIC_JOINT for eq in spec.equalities), 'mimic が変換されていない'
    return spec


def write_scene():
    scene = """<mujoco model="crane_x7_scene">
  <include file="crane_x7.xml"/>

  <statistic center="0.2 0 0.4" extent="1.0"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="150" elevation="-25"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4"
      rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <!-- 到達タスクの目標位置の表示用 (当たり判定なし、コードから mocap_pos で動かす) -->
    <body name="goal" mocap="true" pos="0.3 0 0.3">
      <geom type="sphere" size="0.015" rgba="1 0.8 0 0.6" contype="0" conaffinity="0" group="1"/>
    </body>
  </worldbody>
</mujoco>
"""
    (OUT_DIR / 'scene.xml').write_text(scene)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    spec = build_spec()
    model = spec.compile()
    # 出力ファイルからの相対パスでメッシュを参照させる (他の PC でもそのまま使えるように)
    # to_xml() は内部で再コンパイルするので、書き出した後に meshdir だけ置き換える
    xml = spec.to_xml().replace(
        f'meshdir="{MESH_DIR}/"', f'meshdir="{os.path.relpath(MESH_DIR, OUT_DIR)}/"')
    (OUT_DIR / 'crane_x7.xml').write_text(xml)
    write_scene()
    print(f'wrote {OUT_DIR}/crane_x7.xml and scene.xml')
    print(f'  bodies={model.nbody} joints={model.njnt} actuators={model.nu} geoms={model.ngeom}')


if __name__ == '__main__':
    main()
