#!/bin/bash
# Fixes the placeholder values in the YouBot SDF (see README).
# Usage: ./apply_sdf_fix.sh path/to/youbot_description/sdf/Youbot/model.sdf
set -e
F="${1:?usage: apply_sdf_fix.sh path/to/model.sdf}"
sed -i 's|<body_name>pen</body_name>|<body_name>base_footprint</body_name>|' "$F"
sed -i 's|<frame_name>ground_plane</frame_name>|<frame_name>odom</frame_name>|' "$F"
echo "p3d plugin fixed in $F"
