#==================================================================
# FILE 7: scripts/clean.sh
# Clean build artifacts
#==================================================================

#!/bin/bash

echo "Cleaning build artifacts..."
echo ""

# Firmware
if [ -d "firmware" ]; then
    echo "Cleaning firmware..."
    cd firmware
    pio run -t clean
    cd ..
fi

# Server
if [ -d "server/build" ]; then
    echo "Cleaning server..."
    cd server/build
    make clean
    cd ../..
fi

# Python
echo "Cleaning Python cache..."
find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null
find . -name "*.pyc" -delete 2>/dev/null

echo ""
echo "✓ Cleanup complete"