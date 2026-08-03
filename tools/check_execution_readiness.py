#!/usr/bin/env python3
import sys
from pathlib import Path

import yaml

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from adaptive_object_grasping_nodes.safety_core import site_acceptance_blockers


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else 'config/site_acceptance.yaml')
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception as exc:
        print(f'拒绝真实运动：无法读取现场验收记录 {path}: {exc}', file=sys.stderr)
        return 3

    blockers = site_acceptance_blockers(data)

    if blockers:
        print('拒绝真实运动，未完成：', file=sys.stderr)
        for blocker in dict.fromkeys(blockers):
            print(f'  - {blocker}', file=sys.stderr)
        print(f'现场完成后填写：{path}', file=sys.stderr)
        return 3

    print(f'现场验收记录通过：{path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
