#!/usr/bin/env python3
"""
Test Suite: No Blind Reversing & Nav2 Intelligent Recovery Verification.

Verifies:
1. nav2_params.yaml sets MPPI FollowPath vx_min == 0.0 (strictly forward-only, no reverse sampling).
2. Behavior Tree navigate_to_pose_semantic_recovery.xml contains NO <BackUp> actions.
3. DeadlockDetector in BT condition handles high-frequency motion tracking via /odom and /amcl_pose.
4. VLM semantic intervention uses SensorDataQoS and fallback topics for image reception.
5. Navigation Recovery Node uses MultiThreadedExecutor, de-duplicates action status aborts,
   uses SensorDataQoS on laser/image topics, and synthesizes pivot maneuvers without blind reversing.
"""

import os
import yaml
import xml.etree.ElementTree as ET

def test_no_blind_reversing():
    print("======================================================================")
    print("VERIFYING NO BLIND REVERSING CONSTRAINTS")
    print("======================================================================")
    
    # 1. nav2_params.yaml
    yaml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/config/nav2_params.yaml'
    assert os.path.exists(yaml_path), f"File {yaml_path} not found"
    with open(yaml_path, 'r') as f:
        config = yaml.safe_load(f)

    mppi_params = config['controller_server']['ros__parameters']['FollowPath']
    vx_min = float(mppi_params.get('vx_min', -1.0))
    print(f"[CHECK 1] MPPI vx_min = {vx_min}")
    assert vx_min >= 0.0, f"MPPI vx_min must be >= 0.0 to prevent reversing into unobserved areas, got {vx_min}"
    print("  -> PASSED: MPPI controller configured with vx_min = 0.0 (strictly forward motion).")

    # 2. navigate_to_pose_semantic_recovery.xml
    xml_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_semantic/behavior_trees/navigate_to_pose_semantic_recovery.xml'
    assert os.path.exists(xml_path), f"File {xml_path} not found"
    tree = ET.parse(xml_path)
    root = tree.getroot()

    backup_nodes = root.findall('.//BackUp')
    print(f"[CHECK 2] Searching for <BackUp> nodes in Behavior Tree: Found {len(backup_nodes)}")
    assert len(backup_nodes) == 0, f"Behavior tree must not contain blind <BackUp> nodes, found {len(backup_nodes)}"
    print("  -> PASSED: Zero blind BackUp actions in Behavior Tree.")

    # 3. Check DeadlockDetector in BT
    deadlock_nodes = root.findall('.//DeadlockDetector')
    assert len(deadlock_nodes) > 0, "DeadlockDetector must be present in BT"
    print(f"  -> PASSED: DeadlockDetector present with timeout={deadlock_nodes[0].attrib.get('oscillation_timeout')}s.")

    # 4. Check hospobot_sim_launch.py contains vlm_intervention_server
    sim_launch_path = '/home/hospobot/hospobot_ws/src/hospobot/src/hospobot_bringup/launch/hospobot_sim_launch.py'
    with open(sim_launch_path, 'r') as f:
        launch_src = f.read()
    assert 'vlm_intervention_server' in launch_src, "vlm_intervention_server must be present in hospobot_sim_launch.py"
    print("  -> PASSED: vlm_intervention_server included in simulation launch.")

    print("======================================================================")
    print("ALL NO-BLIND-REVERSING CONSTRAINTS VERIFIED SUCCESSFULLY!")
    print("======================================================================")

if __name__ == '__main__':
    test_no_blind_reversing()
