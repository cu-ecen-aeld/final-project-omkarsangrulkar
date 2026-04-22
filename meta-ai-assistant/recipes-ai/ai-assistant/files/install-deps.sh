#!/bin/sh
if [ ! -f /opt/ai-assistant/.deps-installed ]; then
    pip3 install --break-system-packages google-genai && \
    touch /opt/ai-assistant/.deps-installed
fi
