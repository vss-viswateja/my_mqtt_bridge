#!/bin/bash
# Simple HTTP server for testing MQTT HTML files
cd "$(dirname "$0")"
echo "Starting HTTP server at http://localhost:8000"
echo "Open: http://localhost:8000/mqtt_test.html"
echo "Or: http://localhost:8000/index.html"
echo ""
python3 -m http.server 8000
