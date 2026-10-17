set -e
TAG=$1
S=/d/cftuv-angular-gates-20261017
cd /d/CFTUV-export-angular-20261016
test -z "$(git status --porcelain)" || { echo DIRTY; exit 1; }
sha256sum /c/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend | cut -c1-8
PYTHONPATH="C:/Users/helmd/.cftuv-native/venv-corpus/Lib/site-packages" "/c/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" -B tools/native_angular_pins.py --profile light --out D:/cftuv-angular-gates-20261017/cand/light-$TAG.json
mkdir -p $S/bcfg $S/bscripts $S/btmp
BLENDER_USER_CONFIG=D:/cftuv-angular-gates-20261017/bcfg BLENDER_USER_SCRIPTS=D:/cftuv-angular-gates-20261017/bscripts TMP=D:/cftuv-angular-gates-20261017/btmp TEMP=D:/cftuv-angular-gates-20261017/btmp \
  "/c/Program Files/Blender Foundation/Blender 4.5/blender.exe" -b "C:/Users/helmd/AppData/Local/Temp/cftuv_field_parity_20261008/frozen/testscene.blend" --python-exit-code 1 --python D:/CFTUV-export-20261015-evidence/angular-r2/full_pin_blender_v3.py -- D:/cftuv-angular-gates-20261017/cand/full-$TAG.json > $S/cand/full-$TAG.out 2> $S/cand/full-$TAG.err
cat $S/cand/full-$TAG.json.receipt.json
python $S/scripts/cand_values.py $S/cand/light-$TAG.json $S/cand/full-$TAG.json 2>&1 | tail -20
