from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def _pair(a, b):
    return frozenset((a, b))


def test_collision_model_and_allowed_pairs_are_narrowly_scoped():
    urdf = ET.parse(ROOT / 'config/moveit/robot_v2_2.urdf').getroot()
    srdf = ET.parse(ROOT / 'config/moveit/autolife_s2.srdf').getroot()

    links = urdf.findall('link')
    assert sum(link.find('collision') is not None for link in links) == 43

    adjacent = {
        _pair(joint.find('parent').get('link'), joint.find('child').get('link'))
        for joint in urdf.findall('joint')
    }
    disabled = {
        _pair(item.get('link1'), item.get('link2'))
        for item in srdf.findall('disable_collisions')
    }
    assert len(disabled) == 67

    def gripper_family(link, side):
        return (
            link == f'Link_Camera_Gripper_{side}'
            or link == f'Link_{side}_Wrist_Lower_to_Gripper'
            or link.startswith(f'Link_{side}_Gripper')
        )

    for pair in disabled:
        assert len(pair) == 2
        a, b = tuple(pair)
        assert not (
            (a.startswith('Link_Left_') and b.startswith('Link_Right_'))
            or (a.startswith('Link_Right_') and b.startswith('Link_Left_'))
        )
        assert (
            pair in adjacent
            or all(gripper_family(link, 'Left') for link in pair)
            or all(gripper_family(link, 'Right') for link in pair)
        )


def test_motion_executor_resolves_moveit_meshes_as_package_uris():
    source = (ROOT / 'adaptive_object_grasping_nodes/motion_executor.py').read_text()
    assert "'package://adaptive_object_grasping/meshes/robot_v2_2/'" in source


def test_moveit_ik_reuses_collision_checked_multi_seed_solution():
    source = (ROOT / 'adaptive_object_grasping_nodes/motion_executor.py').read_text()
    assert 'def _solve_moveit_ik' in source
    assert 'scene.is_state_colliding(probe, group, False)' in source
    assert 'component.set_goal_state(robot_state=ik_solution)' in source
    assert "'plan_visible_reset'" in source
    assert 'validate_visible_reset_pose(data, require_verified=True)' in source
    assert "moveit_dir / 'moveit_controllers.yaml'" in source
    wrapper = (ROOT / 'scripts/motion_executor_robot_env.sh').read_text()
    assert 'third_party/ros-overlay/root/opt/ros/jazzy' in wrapper
    assert 'robot_env/lib/python3.12/site-packages:${PYTHONPATH' not in wrapper
    assert "if self._planning_backend == 'vendor_rrt':" in source


def test_package_config_path_allows_symlink_install_but_rejects_traversal():
    source = (ROOT / 'adaptive_object_grasping_nodes/motion_executor.py').read_text()
    assert "if '..' in path.parts:" in source
    assert 'return package_share / path' in source
    assert 'resolved.relative_to(package_share)' not in source


def test_failed_environment_scene_load_cannot_be_bypassed_by_cached_moveit():
    source = (ROOT / 'adaptive_object_grasping_nodes/motion_executor.py').read_text()
    assert 'self._environment_scene_loaded = False' in source
    assert 'if not self._environment_scene_loaded:' in source
    assert 'self._load_environment_scene()' in source
    assert 'self._environment_scene_loaded = True' in source


def test_moveit_pipeline_time_parameterizes_and_validates_every_solution():
    launch = (ROOT / 'launch/bringup.launch.py').read_text()
    planning = (ROOT / 'config/moveit/ompl_planning.yaml').read_text()
    for source in (launch, planning):
        assert 'default_planning_response_adapters/AddTimeOptimalParameterization' in source
        assert 'default_planning_response_adapters/ValidateSolution' in source
