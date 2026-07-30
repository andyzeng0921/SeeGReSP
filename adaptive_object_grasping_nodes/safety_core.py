"""Pure hardware-execution safety checks.

The CLI and ROS executor both call these rules. Keeping the authoritative
checks below the command-line client prevents direct ROS calls from bypassing
the site-acceptance gate.
"""

from collections.abc import Mapping


HARDWARE_EXECUTION_CONFIRMATION = 'I_HAVE_CHECKED_ESTOP_AND_WORKSPACE'

REQUIRED_SITE_ACCEPTANCE = (
    ('controller_adapter', '厂商轨迹适配'),
    ('camera_extrinsic', '相机外参实测标定'),
    ('left_tcp', '左夹爪 TCP 实物标定'),
    ('right_tcp', '右夹爪 TCP 实物标定'),
    ('gripper_mapping', '夹爪反馈到 URDF 的实物标定'),
    ('emergency_stop', '实体急停按下/复位测试'),
    ('workspace', '工作区清空及急停监护人'),
)


def execution_confirmation_valid(provided, expected=HARDWARE_EXECUTION_CONFIRMATION):
    """Return true only for an explicit, exact operator acknowledgement."""

    return str(provided or '') == str(expected)


def site_acceptance_blockers(data):
    """Return all unmet site-acceptance requirements without side effects."""

    if not isinstance(data, Mapping):
        return ['现场验收文件顶层必须是映射']

    blockers = []
    for key, label in REQUIRED_SITE_ACCEPTANCE:
        record = data.get(key) or {}
        if not isinstance(record, Mapping):
            blockers.append(f'{label}（记录格式错误）')
            continue
        if record.get('verified') is not True:
            blockers.append(label)
            continue
        if key != 'controller_adapter':
            if not str(record.get('operator') or '').strip():
                blockers.append(f'{label}（缺现场人员姓名）')
            if not str(record.get('timestamp') or '').strip():
                blockers.append(f'{label}（缺时间）')

    workspace = data.get('workspace') or {}
    if not isinstance(workspace, Mapping):
        workspace = {}
    if workspace.get('collision_zone_cleared') is not True:
        blockers.append('工作区尚未确认清空')
    if workspace.get('observer_at_emergency_stop') is not True:
        blockers.append('急停旁尚无现场监护人')

    emergency_stop = data.get('emergency_stop') or {}
    if not isinstance(emergency_stop, Mapping):
        emergency_stop = {}
    if emergency_stop.get('physical_button_stopped_motion_test') is not True:
        blockers.append('未记录实体急停停止动作测试')
    if emergency_stop.get('reset_and_reenable_test') is not True:
        blockers.append('未记录急停复位与重新使能测试')

    return list(dict.fromkeys(blockers))
