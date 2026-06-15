#!/usr/bin/env python3
import yaml
import os
import sys

def validate_db(db_path):
    print(f"Validating {db_path}...")
    
    if not os.path.exists(db_path):
        print(f"❌ Error: Database file not found at {db_path}")
        sys.exit(1)
        
    try:
        with open(db_path, 'r') as f:
            db = yaml.safe_load(f)
    except Exception as e:
        print(f"❌ Error: Failed to parse YAML. Details: {e}")
        sys.exit(1)
        
    if 'semantic_classes' not in db:
        print("❌ Error: Missing root key 'semantic_classes'")
        sys.exit(1)
        
    config_dir = os.path.dirname(db_path)
    errors = 0
    
    for class_name, rules in db['semantic_classes'].items():
        print(f"\nChecking class: '{class_name}'")
        
        # Check required sections
        if 'costmap_injection' not in rules:
            print("  ⚠️ Warning: Missing 'costmap_injection' section. Node will use defaults.")
        else:
            if rules['costmap_injection'].get('forces_reroute', False):
                print("  ⚠️ Notice: This class is configured to force a global reroute (hallway block).")
        if 'behavior_rules' not in rules:
            print("  ⚠️ Warning: Missing 'behavior_rules' section.")
            
        # Check image paths
        images = rules.get('reference_images', [])
        for img_path in images:
            # We assume paths in the YAML are relative to the package share, e.g. "config/images/bed.jpg"
            # Since this script is inside config/, if the user wrote "config/images/...", we can check from the parent dir.
            # However, for simplicity, let's check relative to the workspace root if needed, or just relative to the config dir itself.
            # If the user put "config/images/bed.jpg", and we are in "config/", the path is "../config/images/bed.jpg" or we can just run from the package root.
            
            # Let's resolve the path assuming it is relative to the package root, which is one level above config.
            package_root = os.path.dirname(config_dir)
            full_img_path = os.path.join(package_root, img_path)
            
            if not os.path.exists(full_img_path):
                print(f"  ❌ Error: Image not found at {full_img_path}")
                errors += 1
            else:
                print(f"  ✅ Image verified: {img_path}")
                
    if errors > 0:
        print(f"\n❌ Validation failed with {errors} errors.")
        sys.exit(1)
    else:
        print("\n✅ Database validation passed successfully!")

if __name__ == '__main__':
    script_dir = os.path.dirname(os.path.realpath(__file__))
    db_file = os.path.join(script_dir, 'semantic_objects_db.yaml')
    validate_db(db_file)
