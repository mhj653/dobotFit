# -*- mode: python ; coding: utf-8 -*-
import os
import sys

from PyInstaller.utils.hooks import collect_all

conda_prefix = os.environ.get('CONDA_PREFIX', sys.prefix)
conda_bin = os.path.join(conda_prefix, 'Library', 'bin')

datas = [('ui/styles', 'ui/styles')]
binaries = [
    (os.path.join(conda_bin, 'liblzma.dll'), '.'),
    (os.path.join(conda_bin, 'libbz2.dll'), '.'),
    (os.path.join(conda_bin, 'libcrypto-3-x64.dll'), '.'),
]
hiddenimports = ['pybullet', 'numpy', 'pyrealsense2', 'cv2']
tmp_ret = collect_all('numpy')
datas += tmp_ret[0]; binaries += tmp_ret[1]; hiddenimports += tmp_ret[2]


a = Analysis(
    ['main.py'],
    pathex=[],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=0,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='RobotAutomationStudio',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='RobotAutomationStudio',
)
