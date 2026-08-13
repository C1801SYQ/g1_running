#!/usr/bin/env bash

echo "=============================="
echo "1. System Info"
echo "=============================="
date
echo
uname -a
echo
if command -v lsb_release >/dev/null 2>&1; then
    lsb_release -a
else
    cat /etc/os-release
fi

echo
echo "=============================="
echo "2. CPU / RAM / Disk"
echo "=============================="
lscpu | grep -E "Model name|CPU\\(s\\)|Thread|Core|Socket" || true
echo
free -h
echo
df -h ~ .

echo
echo "=============================="
echo "3. NVIDIA GPU / Driver"
echo "=============================="
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi
else
    echo "ERROR: nvidia-smi not found. NVIDIA driver may not be installed."
fi

echo
echo "=============================="
echo "4. CUDA Toolkit"
echo "=============================="
if command -v nvcc >/dev/null 2>&1; then
    nvcc --version
else
    echo "nvcc not found. This is not always fatal for Isaac Lab, but useful to know."
fi

echo
echo "=============================="
echo "5. Python / Conda"
echo "=============================="
which python || true
python --version || true
which python3 || true
python3 --version || true
which conda || true
conda info --envs 2>/dev/null || true

echo
echo "=============================="
echo "6. Python Packages"
echo "=============================="
python3 - <<'PY'
import importlib.util

mods = [
    "torch",
    "torchvision",
    "numpy",
    "gymnasium",
    "rsl_rl",
    "isaacsim",
    "isaaclab",
    "omni",
]

for m in mods:
    spec = importlib.util.find_spec(m)
    print(f"{m:12s}: {'FOUND' if spec else 'NOT FOUND'}")

try:
    import torch
    print("\nTorch version:", torch.__version__)
    print("Torch CUDA available:", torch.cuda.is_available())
    if torch.cuda.is_available():
        print("Torch CUDA version:", torch.version.cuda)
        print("GPU:", torch.cuda.get_device_name(0))
except Exception as e:
    print("Torch check error:", repr(e))
PY

echo
echo "=============================="
echo "7. Isaac Lab / Unitree Repos"
echo "=============================="
echo "Searching common repo locations..."
find "$HOME" -maxdepth 4 -type f -name "isaaclab.sh" 2>/dev/null | head -20
find "$HOME" -maxdepth 4 -type d -name "unitree_rl_lab" 2>/dev/null | head -20
find "$HOME" -maxdepth 4 -type d -name "unitree_rl_gym" 2>/dev/null | head -20
find "$HOME" -maxdepth 4 -type d -name "IsaacLab" 2>/dev/null | head -20

echo
echo "=============================="
echo "8. RealSense Optional Check"
echo "=============================="
if command -v realsense-viewer >/dev/null 2>&1; then
    echo "realsense-viewer found:"
    realsense-viewer --version || true
else
    echo "realsense-viewer not found."
fi

if command -v rs-enumerate-devices >/dev/null 2>&1; then
    echo
    echo "RealSense devices:"
    rs-enumerate-devices -s || true
else
    echo "rs-enumerate-devices not found."
fi

echo
echo "=============================="
echo "9. Git"
echo "=============================="
git --version || true

echo
echo "Environment check finished."
