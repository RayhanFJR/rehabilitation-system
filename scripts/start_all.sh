#==================================================================
# FILE 6: scripts/start_all.sh
# Start complete system (3 terminals or background)
#==================================================================

#!/bin/bash

echo "=========================================="
echo "  Starting Complete System"
echo "=========================================="
echo ""

# Check if running in background mode
BACKGROUND=false
if [ "$1" == "--background" ]; then
    BACKGROUND=true
fi

echo "Background mode: $BACKGROUND"
echo ""

# ============ FIRMWARE ============
echo "[System] Starting firmware monitor..."

if $BACKGROUND; then
    # Create log directory
    mkdir -p logs
    
    # Start in background with logging
    (cd firmware && pio device monitor > logs/firmware.log 2>&1) &
    FIRMWARE_PID=$!
    echo "✓ Firmware monitor started (PID: $FIRMWARE_PID)"
else
    echo "Please run in separate terminal:"
    echo "  cd firmware && pio device monitor"
fi

echo ""

# ============ SERVER ============
echo "[System] Starting server..."

if $BACKGROUND; then
    (cd server/build && ./rehab_server > ../../logs/server.log 2>&1) &
    SERVER_PID=$!
    sleep 2
    echo "✓ Server started (PID: $SERVER_PID)"
else
    echo "Please run in separate terminal:"
    echo "  cd server/build && ./rehab_server"
fi

echo ""

# ============ VISION ============
echo "[System] Starting vision system..."

if $BACKGROUND; then
    (cd vision && python3 src/main.py > ../logs/vision.log 2>&1) &
    VISION_PID=$!
    echo "✓ Vision system started (PID: $VISION_PID)"
else
    echo "Please run in separate terminal:"
    echo "  cd vision && python3 src/main.py"
fi

echo ""

if $BACKGROUND; then
    echo "=========================================="
    echo "  System Running in Background"
    echo "=========================================="
    echo ""
    echo "Process IDs:"
    echo "  Firmware: $FIRMWARE_PID"
    echo "  Server:   $SERVER_PID"
    echo "  Vision:   $VISION_PID"
    echo ""
    echo "Log files in: logs/"
    echo ""
    echo "To stop:"
    echo "  kill $FIRMWARE_PID $SERVER_PID $VISION_PID"
    echo ""
    
    # Keep running
    wait
else
    echo "=========================================="
    echo "  Start components in separate terminals"
    echo "=========================================="
fi