#!/usr/bin/env bash
# "WebODM terminal" of the demo: upload the geotagged photos and build the map.
source "$(dirname "${BASH_SOURCE[0]}")/demo_rc.sh"
set -a; . ./webodm.env; set +a   # local test account (WEBODM_URL / USER / PASSWORD)

say "STEP 3 - build the map: upload the geotagged photos to WebODM"
run 'ls photos/*.jpg | wc -l'
run head -4 photos/geo.txt
cat > webodm_options.json <<'EOF'
[{"name": "auto-boundary", "value": true},
 {"name": "dsm", "value": true},
 {"name": "orthophoto-resolution", "value": 3},
 {"name": "pc-quality", "value": "medium"}]
EOF
run cat webodm_options.json
run 'python3 project/webodm_upload.py photos --project-name "Survey simulation" --task-name block_1 --options "$(cat webodm_options.json)" --download orthophoto.tif dsm.tif textured_model.zip report.pdf --out results'
touch webodm.done
exec bash
