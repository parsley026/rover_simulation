import os
import glob
import yaml

config_dir = '/home/raptors/Desktop/rover_simulation_ai/src/rover_autonomy/config/navigation'
yaml_files = glob.glob(os.path.join(config_dir, 'costmaps_*.yaml'))
yaml_files.append(os.path.join(config_dir, 'navigation.yaml')) # Just in case

class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True

for file in yaml_files:
    if not os.path.exists(file): continue
    
    with open(file, 'r') as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            print(f"Error parsing {file}: {e}")
            continue
            
    if data is None:
        continue
        
    with open(file, 'w') as f:
        yaml.dump(data, f, Dumper=NoAliasDumper, default_flow_style=False, sort_keys=False)

print("Removed aliases from yaml files")
