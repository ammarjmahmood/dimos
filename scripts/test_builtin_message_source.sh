#!/usr/bin/env bash
# Compiler-free installation acceptance for checked-in built-in message sources.
set -euo pipefail
cd "$(dirname "$0")/.."
output="$PWD/build/message-codegen/builtin-source"
uv build --python .venv/bin/python --out-dir "$output/dist" packages/dimos-generated
uv venv --python .venv/bin/python "$output/venv"
uv pip install --python "$output/venv/bin/python" "$output"/dist/*.whl
uv pip install --python "$output/venv/bin/python" setuptools wheel
CC=/bin/false CXX=/bin/false uv pip install --no-build-isolation \
  --python "$output/venv/bin/python" packages/dimos-generated
CC=/bin/false CXX=/bin/false uv pip install --no-build-isolation \
  --python "$output/venv/bin/python" -e packages/dimos-generated
env -u PYTHONPATH "$output/venv/bin/python" -I - <<'PY'
from pathlib import Path
import pickle
import numpy as np
import dimos_generated
from dimos_generated_schemas.provider import message_types
from dimos_generated.std_msgs.msg import Header
from dimos_generated.sensor_msgs.msg import Image

assert Path(dimos_generated.__file__).suffix == '.py'
classes = message_types()
for name, cls in classes.items():
    value = cls()
    for little_endian in (False, True):
        assert cls.decode(value.encode(little_endian)) == value, name
image = Image(header=Header(frame_id='camera'), height=480, width=640,
              encoding='rgb8', step=1920, data=np.arange(921600, dtype=np.uint8))
restored = Image.decode(image.encode())
assert type(restored.header) is Header
assert bytes(restored.data.view()) == bytes(image.data.view())
assert pickle.loads(pickle.dumps(image)) == image
print(f'PASS: compiler-free wheel/source/editable installation; {len(classes)} types '
      'in both byte orders; 921600-byte Image, Header identity and worker pickle')
PY
