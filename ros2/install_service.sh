#!/bin/bash

# Robot Auto Service Installation Script
echo "🔧 Installing Robot Auto Service..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ This script must be run as root (use sudo)"
    exit 1
fi

# Copy service file to systemd directory
echo "📁 Installing systemd service..."
cp robot-auto.service /etc/systemd/system/

# Reload systemd daemon
echo "🔄 Reloading systemd daemon..."
systemctl daemon-reload

# Enable the service for automatic startup
echo "✅ Enabling service for automatic startup..."
systemctl enable robot-auto.service

echo ""
echo "🎯 Installation Complete!"
echo "========================"
echo "📋 Service Commands:"
echo "- Start service: sudo systemctl start robot-auto"
echo "- Stop service: sudo systemctl stop robot-auto"
echo "- Check status: sudo systemctl status robot-auto"
echo "- View logs: sudo journalctl -u robot-auto -f"
echo "- Disable auto-start: sudo systemctl disable robot-auto"
echo ""
echo "🚀 The robot will now start automatically on boot!"
echo "To start it now, run: sudo systemctl start robot-auto" 