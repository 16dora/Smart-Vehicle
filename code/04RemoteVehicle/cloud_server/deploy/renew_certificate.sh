#!/bin/bash
set -euo pipefail
# 公开 IP 的短期证书续期后原子安装，设备信任根证书无需跟随叶证书更新。
install -m 644 /etc/letsencrypt/live/124.223.192.76/fullchain.pem /etc/smart-vehicle/server_ca.pem
install -m 600 /etc/letsencrypt/live/124.223.192.76/privkey.pem /etc/smart-vehicle/server.key
nginx -t
systemctl reload nginx
