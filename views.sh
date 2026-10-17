#!/usr/bin/env bash
# Jump the Gazebo camera to a standard view.  usage:  bash views.sh front|back|left|right|top|iso|nozzle
# Run it in an Ubuntu terminal while the simulation window is open.
v="${1:-iso}"
case "$v" in
  front) POSE="position: {x: 0, y: -1.5, z: 0.55} orientation: {x: -0.04933, y: 0.04933, z: 0.70538, w: 0.70538}" ;;
  back) POSE="position: {x: 0, y: 1.5, z: 0.55} orientation: {x: 0.04933, y: 0.04933, z: -0.70538, w: 0.70538}" ;;
  left) POSE="position: {x: -1.5, y: 0, z: 0.55} orientation: {x: 0.00000, y: 0.06976, z: 0.00000, w: 0.99756}" ;;
  right) POSE="position: {x: 1.5, y: 0, z: 0.55} orientation: {x: -0.06976, y: 0.00000, z: 0.99756, w: 0.00000}" ;;
  top) POSE="position: {x: 0, y: 0, z: 1.9} orientation: {x: -0.50000, y: 0.50000, z: 0.50000, w: 0.50000}" ;;
  iso) POSE="position: {x: 1.0, y: -1.1, z: 0.95} orientation: {x: -0.19996, y: 0.08283, z: 0.90198, w: 0.37361}" ;;
  nozzle) POSE="position: {x: 0, y: -0.55, z: 0.58} orientation: {x: -0.11062, y: 0.11062, z: 0.69840, w: 0.69840}" ;;
  *) echo "unknown view '$v' - use: front back left right top iso nozzle"; exit 1 ;;
esac
source /opt/ros/jazzy/setup.bash 2>/dev/null
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean --timeout 3000 --req "pose: {$POSE}"
