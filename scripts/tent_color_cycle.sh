#!/usr/bin/env bash
set -euo pipefail

# Cycle tent color in Gazebo Sim with random colors.
# Removes tent_target from the world if present, then spawns its own
# 'tent_color' model and cycles through random RGB colors.
#
# REQUIRES: tent_target commented out in terrain.sdf
#           (the script manages the tent lifecycle)
#
# Usage:
#   ./scripts/tent_color_cycle.sh [world] [total_changes] [container]
#
# Examples:
#   ./scripts/tent_color_cycle.sh                          # defaults: terrain, 60 changes
#   ./scripts/tent_color_cycle.sh terrain 100
#   ./scripts/tent_color_cycle.sh terrain 60 knr_drone_px4

WORLD_NAME="${1:-terrain}"
# Fixed step seconds to exactly 5s as requested by user
STEP_SECONDS="5"
TOTAL_CHANGES="${2:-60}"
CONTAINER_NAME="${3:-${TENT_COLOR_CONTAINER:-knr_drone_px4}}"

# Validate basic inputs
if ! [[ "$TOTAL_CHANGES" =~ ^[0-9]+$ ]]; then
  echo "[ERR] Drugi argument (\$2) 'TOTAL_CHANGES' musi być liczbą, u ciebie jest: '${TOTAL_CHANGES}'." >&2
  echo "[ERR] Poprawna komenda: ./tent_color_cycle.sh terrain 60" >&2
  exit 1
fi
SCRIPT_IN_CONTAINER="/root/Dron_symulacja/scripts/tent_color_cycle.sh"

MODEL="tent_color"

# ---------- Auto-exec inside Docker if needed ----------
if [[ "${TENT_COLOR_IN_CONTAINER:-0}" != "1" && ! -f "/.dockerenv" ]]; then
  if command -v docker >/dev/null 2>&1; then
    if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}"; then
      echo "[INFO] Re-running inside Docker: ${CONTAINER_NAME}"
      exec docker exec -i "${CONTAINER_NAME}" bash -lc \
        "TENT_COLOR_IN_CONTAINER=1 '${SCRIPT_IN_CONTAINER}' '${WORLD_NAME}' '${TOTAL_CHANGES}' '${CONTAINER_NAME}'"
    fi
  fi
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "[ERR] 'gz' not found." >&2; exit 1
fi

SDF_TMP=$(mktemp /tmp/tent_color_XXXXXX.sdf)

cleanup() {
  # Prevent multiple traps executing the slow cleanup simultaneously if user presses Ctrl+C again
  trap '' EXIT INT TERM
  echo ""; echo "[INFO] Cleaning up ${MODEL} and debris..."
  remove_model
  remove_debris
  rm -f "${SDF_TMP}"
  echo "[INFO] Done cleaning up. Restart sim to restore original tent."
}
trap cleanup EXIT INT TERM

# Pose from terrain.sdf: dataset_anchor + tent yaw 0.55 rad
POSE='position: { x: 15.19 y: -197.69 z: 2.29 } orientation: { x: 0 y: 0 z: 0.2717606 w: 0.9623461 }'

write_and_spawn() {
  local r="$1" g="$2" b="$3"

  # Write SDF
  cat > "${SDF_TMP}" <<EOF
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="${MODEL}">
    <static>true</static>
    <link name="base_link">
      <visual name="visual">
        <geometry>
          <mesh><uri>model://tent/mesh/tent.dae</uri></mesh>
        </geometry>
        <material>
          <ambient>${r} ${g} ${b} 1.0</ambient>
          <diffuse>${r} ${g} ${b} 1.0</diffuse>
          <specular>0.5 0.5 0.5 1.0</specular>
          <pbr>
            <metal>
              <albedo_map>model://tent/mesh/tent.png</albedo_map>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF

  # Spawn (no 'name' override — uses model name from SDF)
  gz service -s "/world/${WORLD_NAME}/create" --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req "sdf_filename: \"${SDF_TMP}\" pose: { ${POSE} }" >/dev/null 2>&1
}

remove_model() {
  gz service -s "/world/${WORLD_NAME}/remove" --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 500 \
    --req "name: \"${MODEL}\" type: MODEL" >/dev/null 2>&1 || true
}

NUM_DEBRIS=15
TENT_X=15.19
TENT_Y=-197.69
TENT_Z=2.29
RADIUS=20.0

remove_debris() {
  for j in $(seq 1 $NUM_DEBRIS); do
    gz service -s "/world/${WORLD_NAME}/remove" --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean --timeout 500 \
      --req "name: \"debris_${j}\" type: MODEL" >/dev/null 2>&1 || true
  done
}

spawn_debris() {
  python3 -c "
import random, math, subprocess, tempfile, os
models = [
  'Ambulance', 'Cabinet', 'Cardboard box', 'Car Wheel', 'Casual female',
  'Construction Barrel', 'DRC Practice_ Orange Jersey Barrier', 'Dumpster',
  'Euro pallet', 'fence', 'Fire hydrant', 'FIRST 2015 trash can',
  'NIST fiducial barrel', 'OpScrubs', 'pallet', 'Pickup',
  'Reflective table', 'Scrubs', 'Standing person'
]
bushes = ['bush_0', 'bush_1', 'bush_2', 'bush_3', 'bush_4', 'bush_5']
trees = ['tree_1', 'tree_2', 'tree_3', 'tree_4', 'tree_5', 'tree_6', 'tree_7', 'tree_8', 'tree_9', 'palm_tree', 'oak_tree', 'pine_tree']

world = '${WORLD_NAME}'
plants = bushes + trees
selected_models = random.sample(models, 10)
selected_plants = random.sample(plants, 5)
selected = selected_models + selected_plants
random.shuffle(selected)
n_items = len(selected)

for j, model in enumerate(selected):
    angle = random.uniform(0, 2*math.pi)
    dist = random.uniform(4.0, ${RADIUS}) # at least 4m from tent
    x = ${TENT_X} + dist * math.cos(angle)
    y = ${TENT_Y} + dist * math.sin(angle)
    yaw = random.uniform(-math.pi, math.pi)
    
    sdf = f'''<?xml version=\"1.0\"?>
<sdf version=\"1.6\">
  <model name=\"debris_{j+1}\">
    <include><uri>model://{model}</uri></include>
  </model>
</sdf>'''
    
    path = f'/tmp/tmp_debris_{j+1}.sdf'
    with open(path, 'w') as f:
        f.write(sdf)
    
    pose = f'position: {{ x: {x:.2f} y: {y:.2f} z: 3.0 }} orientation: {{ x: 0 y: 0 z: {math.sin(yaw/2):.4f} w: {math.cos(yaw/2):.4f} }}'
    req = f'sdf_filename: \"{path}\" pose: {{ {pose} }}'
    print(f'  -> Zrzucam z nieba [{j+1}/{n_items}]: {model}', flush=True)
    cmd = ['gz', 'service', '-s', f'/world/{world}/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean', '--timeout', '10000', '--req', req]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
"
}

# ---------- Main ----------
echo "[INFO] World: ${WORLD_NAME} | Step: ${STEP_SECONDS}s | Changes: ${TOTAL_CHANGES}"
echo "[INFO] Ctrl+C to stop."

# Clean any leftover
remove_model
remove_debris
sleep 1

# Pre-generate random colors
mapfile -t COLORS < <(python3 -c "
import colorsys, random
for _ in range(${TOTAL_CHANGES}):
    h = random.random()
    s = random.uniform(0.4, 0.9)
    v = random.uniform(0.3, 0.6)
    r, g, b = colorsys.hsv_to_rgb(h, s, v)
    print(f'{r:.4f} {g:.4f} {b:.4f}')
")

echo "[INFO] Starting ${TOTAL_CHANGES} color changes..."
DEBRIS_INTERVAL=12 # 12 * 5s = 60s
for i in $(seq 0 $((TOTAL_CHANGES - 1))); do
  read -r r g b <<< "${COLORS[$i]}"

  # 1) Tent always changes color (every 5s)
  remove_model
  write_and_spawn "${r}" "${g}" "${b}" || true

  # 2) Debris only refreshes once per minute (every 12 tent iterations)
  if (( i % DEBRIS_INTERVAL == 0 )); then
    echo "  -> Odświeżam przeszkody (raz na minutę)..."
    remove_debris
    spawn_debris || true
  fi

  echo "  [$((i+1))/${TOTAL_CHANGES}]  rgb(${r}, ${g}, ${b})"
  sleep "${STEP_SECONDS}"
done

echo "[INFO] Done!"
