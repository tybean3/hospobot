#!/bin/bash

# Ensure the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo:"
  echo "sudo $0"
  exit 1
fi

# Install wireguard-tools if not present
if ! command -v wg &> /dev/null; then
    echo "Installing wireguard-tools..."
    apt-get update
    apt-get install -y wireguard-tools resolvconf
fi

echo "Generating WireGuard keys..."
PRIVATE_KEY=$(wg genkey)
PUBLIC_KEY=$(echo "$PRIVATE_KEY" | wg pubkey)

# Prompt for the assigned client IP on the VPN
echo ""
read -p "Enter the internal VPN IP address assigned to this client (e.g., 10.8.0.2/24): " CLIENT_IP

# Prompt for AllowedIPs
echo ""
echo "Which IPs should be routed through the VPN?"
echo " - Enter '0.0.0.0/0' to route ALL internet traffic through your home network."
echo " - Enter your home subnet (e.g., '192.168.1.0/24, 10.8.0.0/24') to only route home network traffic."
read -p "AllowedIPs: " ALLOWED_IPS

# Create wg0.conf
echo "Creating /etc/wireguard/wg0.conf..."
cat <<EOF > /etc/wireguard/wg0.conf
[Interface]
PrivateKey = $PRIVATE_KEY
Address = $CLIENT_IP

[Peer]
PublicKey = 5Y2LCpLaqDCVHmjAicTL2rHyuUz2Ebei55Bb5JZSPQ4=
Endpoint = 124.183.7.111:51820
AllowedIPs = $ALLOWED_IPS
PersistentKeepalive = 25
EOF

chmod 600 /etc/wireguard/wg0.conf

echo ""
echo "=========================================================="
echo "WireGuard configuration has been generated successfully!"
echo "Saved to: /etc/wireguard/wg0.conf"
echo ""
echo ">>> CRITICAL NEXT STEP <<<"
echo "You MUST add this machine's public key to your home server's WireGuard configuration as a new [Peer]:"
echo ""
echo "PublicKey = $PUBLIC_KEY"
echo ""
echo "=========================================================="
echo ""
echo "Commands to manage the VPN connection:"
echo "  Start:  wg-quick up wg0"
echo "  Stop:   wg-quick down wg0"
echo "  Status: wg show"
echo "=========================================================="
