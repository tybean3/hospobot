#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "sudo $0"
  exit 1
fi

echo "Updating /etc/wireguard/wg0.conf with the server-provided configuration..."

cat <<'EOF' > /etc/wireguard/wg0.conf
[Interface]
Address = 10.0.0.3/24,fd00:db8:0:abc::3/64
PrivateKey = CD00uLieVzZR8wOKSFOakw8RvCHtLRWOD4nIO8OK8nY=
DNS = 10.0.0.1,fd00:db8:0:abc::1
MTU = 1420

[Peer]
AllowedIPs = 0.0.0.0/0,::/0
Endpoint = 124.183.7.111:51820
PersistentKeepalive = 25
PublicKey = 5Y2LCpLaqDCVHmjAicTL2rHyuUz2Ebei55Bb5JZSPQ4=
PresharedKey = Vo0A/dDeMXitk4t05SWyOvVVGG6ydwZj8EY8FLcfMFk=
EOF

chmod 600 /etc/wireguard/wg0.conf

echo "=========================================================="
echo "Configuration successfully updated!"
echo "You can now connect to your home network."
echo ""
echo "To start the connection, run:"
echo "  wg-quick up wg0"
echo ""
echo "To check the connection status, run:"
echo "  wg show"
echo "=========================================================="
