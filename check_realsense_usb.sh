#!/bin/bash
# Script to detect Intel RealSense camera USB connection speed
# Returns: "3.0", "2.0", or "unknown"

# Intel RealSense vendor ID
REALSENSE_VENDOR_ID="8086"

# Search for RealSense device in sysfs
USB_SPEED=""
for dev in /sys/bus/usb/devices/*; do
    if [ -f "$dev/idVendor" ]; then
        VENDOR=$(cat "$dev/idVendor" 2>/dev/null)
        if [ "$VENDOR" = "$REALSENSE_VENDOR_ID" ]; then
            # Found RealSense device, check its speed
            if [ -f "$dev/speed" ]; then
                USB_SPEED=$(cat "$dev/speed" 2>/dev/null)
                break
            fi
        fi
    fi
done

# Parse speed and return appropriate version
if [ -n "$USB_SPEED" ]; then
    # USB 3.2 Gen 2x2 = 20000 Mbps
    # USB 3.1 Gen 2 / USB 3.2 Gen 2 = 10000 Mbps
    # USB 3.0 / USB 3.1 Gen 1 / USB 3.2 Gen 1 = 5000 Mbps
    # USB 2.0 = 480 Mbps
    # USB 1.1 = 12 Mbps
    if [ "$USB_SPEED" -ge 20000 ]; then
        echo "3.2"
        exit 0
    elif [ "$USB_SPEED" -ge 10000 ]; then
        echo "3.1"
        exit 0
    elif [ "$USB_SPEED" -ge 5000 ]; then
        echo "3.0"
        exit 0
    elif [ "$USB_SPEED" -ge 480 ]; then
        echo "2.0"
        exit 0
    else
        echo "2.0"
        exit 0
    fi
fi

echo "unknown"
exit 1
