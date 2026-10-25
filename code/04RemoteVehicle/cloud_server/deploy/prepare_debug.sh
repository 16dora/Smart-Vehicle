#!/bin/bash
set -euo pipefail
# 为 ubuntu 创建独立调试数据库；不改变生产账号，已有调试数据不覆盖。
install -d -o ubuntu -g ubuntu -m 700 /home/ubuntu/smart-vehicle-cloud/debug
python3 - <<'PY'
from pathlib import Path
import sqlite3
root = Path('/home/ubuntu/smart-vehicle-cloud/debug')
target = root / 'users.db'
if not target.exists():
    source = sqlite3.connect('file:/var/lib/smart-vehicle/users.db?mode=ro', uri=True)
    dest = sqlite3.connect(target)
    source.backup(dest)
    dest.close()
    source.close()
settings = {}
for line in Path('/etc/smart-vehicle/cloud.env').read_text().splitlines():
    if '=' in line:
        key, value = line.split('=', 1)
        settings[key] = value
settings['SV_DATABASE'] = str(target)
settings['SV_WEB_ROOT'] = '/home/ubuntu/smart-vehicle-cloud/src/web'
(root / 'debug.env').write_text(''.join(key+'='+value+'\n' for key,value in settings.items()))
PY
chown ubuntu:ubuntu /home/ubuntu/smart-vehicle-cloud/debug/users.db /home/ubuntu/smart-vehicle-cloud/debug/debug.env
chmod 600 /home/ubuntu/smart-vehicle-cloud/debug/users.db /home/ubuntu/smart-vehicle-cloud/debug/debug.env
