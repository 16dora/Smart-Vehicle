#!/bin/bash
set -euo pipefail
# 输入：源码目录和可执行文件；作用：安装独立服务，不触碰运行数据库和已有设备密钥。
src=${1:?source directory required}
binary=${2:?executable required}
id smart-vehicle >/dev/null 2>&1 || useradd --system --home-dir /var/lib/smart-vehicle --shell /usr/sbin/nologin smart-vehicle
install -d -m 755 /opt/smart-vehicle/bin /opt/smart-vehicle/web
install -d -m 700 /etc/smart-vehicle
install -d -o smart-vehicle -g smart-vehicle -m 700 /var/lib/smart-vehicle
install -m 755 "$binary" /opt/smart-vehicle/bin/smart_vehicle_cloud
install -m 644 "$src"/web/*.html /opt/smart-vehicle/web/
if [ ! -f /etc/smart-vehicle/server.key ]; then
    # IP 预览使用自签证书；私钥只保留在服务器，固件仅嵌入公开证书。
    openssl req -x509 -newkey rsa:3072 -nodes -days 365 \
        -keyout /etc/smart-vehicle/server.key -out /etc/smart-vehicle/server_ca.pem \
        -subj /CN=124.223.192.76 -addext subjectAltName=IP:124.223.192.76 \
        -addext basicConstraints=critical,CA:TRUE
    chmod 600 /etc/smart-vehicle/server.key
    chmod 644 /etc/smart-vehicle/server_ca.pem
fi
install -m 644 "$src/deploy/smart-vehicle.service" /etc/systemd/system/smart-vehicle.service
if [ -f /etc/letsencrypt/live/124.223.192.76/fullchain.pem ]; then
    install -d /usr/local/libexec
    install -m 755 "$src/deploy/renew_certificate.sh" /usr/local/libexec/smart-vehicle-renew
    install -m 644 "$src/deploy/smart-vehicle-cert.service" /etc/systemd/system/
    install -m 644 "$src/deploy/smart-vehicle-cert.timer" /etc/systemd/system/
    /usr/local/libexec/smart-vehicle-renew
fi
if [ -L /etc/nginx/sites-enabled/default ]; then
    # 首次替换前保留原站点，便于管理员恢复。
    cp -an /etc/nginx/sites-available/default /etc/smart-vehicle/nginx-default.backup
    unlink /etc/nginx/sites-enabled/default
fi
install -m 644 "$src/deploy/nginx.conf" /etc/nginx/sites-available/smart-vehicle
ln -sfn /etc/nginx/sites-available/smart-vehicle /etc/nginx/sites-enabled/smart-vehicle
nginx -t
systemctl daemon-reload
if [ -f /etc/systemd/system/smart-vehicle-cert.timer ]; then systemctl enable --now smart-vehicle-cert.timer; fi
systemctl enable smart-vehicle
systemctl restart smart-vehicle
systemctl reload nginx
