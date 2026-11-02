import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_active_bridge_config(
    payload_file: str,
    bridges_dir: str = "",
    entity_name: str = "rover",
    world_name: str = "mars_yard",
) -> str:
    """
    Build a dynamic ros_gz_bridge configuration file by assembling:
      1. Base chassis bridge (_bridge_rover.yaml)
      2. Active submodule bridges (_bridge_arm.yaml, _bridge_autonomy.yaml, _bridge_crossbar.yaml)
      3. Active sensor templates (bridges/sensors/<model>.yaml) with {ID} token replacement.

    Parameters:
        payload_file (str): Path to payload.yaml defining mounted submodules and sensors.
        bridges_dir (str): Path to rover_description/config/bridges directory.
                           If empty, automatically resolved via ament_index.
        entity_name (str): Name of the robot entity in Gazebo.
        world_name (str): Name of the Gazebo world.

    Returns:
        str: Absolute path to the generated composite YAML file in /tmp.
    """
    if not bridges_dir:
        pkg_desc = get_package_share_directory("rover_description")
        bridges_dir = os.path.join(pkg_desc, "config", "bridges")

    entries = []
    seen_topics = set()

    def add_entries(raw_entries):
        for entry in raw_entries:
            if not isinstance(entry, dict):
                continue
            ros_topic = entry.get("ros_topic_name")
            direction = entry.get("direction")
            key = (ros_topic, direction)
            if key not in seen_topics:
                seen_topics.add(key)
                entries.append(entry)

    # 1. Base rover chassis bridge
    chassis_bridge_path = os.path.join(bridges_dir, "_bridge_rover.yaml")
    if os.path.exists(chassis_bridge_path):
        with open(chassis_bridge_path, "r") as f:
            data = yaml.safe_load(f) or []
            if isinstance(data, list):
                add_entries(data)

    # 2. Parse payload.yaml
    payload_data = {}
    if os.path.exists(payload_file):
        try:
            with open(payload_file, "r") as f:
                payload_data = yaml.safe_load(f) or {}
        except Exception as e:
            print(f"[rover_description.bridge_generator] Error loading payload_file {payload_file}: {e}")

    submodules = payload_data.get("submodules") or []
    submodule_ids = {s.get("id") for s in submodules if isinstance(s, dict) and "id" in s}
    sensors = payload_data.get("sensors") or []

    # Robotic arm submodule bridge
    if "robotic_arm" in submodule_ids:
        arm_bridge_path = os.path.join(bridges_dir, "_bridge_arm.yaml")
        if os.path.exists(arm_bridge_path):
            with open(arm_bridge_path, "r") as f:
                data = yaml.safe_load(f) or []
                if isinstance(data, list):
                    add_entries(data)

    # Autonomy submodule bridge
    if "autonomy" in submodule_ids:
        autonomy_bridge_path = os.path.join(bridges_dir, "_bridge_autonomy.yaml")
        if os.path.exists(autonomy_bridge_path):
            with open(autonomy_bridge_path, "r") as f:
                data = yaml.safe_load(f) or []
                if isinstance(data, list):
                    add_entries(data)

    # Crossbar submodule bridge
    if "crossbar" in submodule_ids:
        crossbar_bridge_path = os.path.join(bridges_dir, "_bridge_crossbar.yaml")
        if os.path.exists(crossbar_bridge_path):
            with open(crossbar_bridge_path, "r") as f:
                data = yaml.safe_load(f) or []
                if isinstance(data, list):
                    add_entries(data)

    # 3. Dynamic sensor templates
    for sensor in sensors:
        if not isinstance(sensor, dict):
            continue
        sensor_id = sensor.get("id")
        model = sensor.get("model")
        parent = str(sensor.get("parent", ""))
        if not sensor_id or not model:
            continue

        # Skip sensors whose parent submodule is inactive
        if ("sensor_mount_06" in parent or "wrist" in parent) and "robotic_arm" not in submodule_ids:
            continue
        if ("sensor_mount_00" in parent or "sensor_mount_01" in parent) and "autonomy" not in submodule_ids:
            continue
        if "sensor_mount_02" in parent and "crossbar" not in submodule_ids:
            continue

        template_file = os.path.join(bridges_dir, "sensors", f"{model}.yaml")
        if os.path.exists(template_file):
            try:
                with open(template_file, "r") as tf:
                    content = tf.read()
                content = content.replace("{ID}", sensor_id)
                content = content.replace("{ROBOT_NAME}", entity_name)
                content = content.replace("{WORLD_NAME}", world_name)
                sensor_entries = yaml.safe_load(content) or []
                if isinstance(sensor_entries, list):
                    add_entries(sensor_entries)
            except Exception as e:
                print(f"[rover_description.bridge_generator] Error processing template for sensor {sensor_id}: {e}")

    # 4. Save composite bridge file
    output_file = os.path.join("/tmp", f"active_bridge_{entity_name}.yaml")
    try:
        with open(output_file, "w") as out_f:
            yaml.dump(entries, out_f, default_flow_style=False)
        return output_file
    except Exception as e:
        print(f"[rover_description.bridge_generator] Error writing {output_file}: {e}")
        return chassis_bridge_path
