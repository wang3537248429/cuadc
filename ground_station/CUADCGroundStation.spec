# -*- mode: python ; coding: utf-8 -*-

from PyInstaller.utils.hooks import collect_all


pymavlink_datas, pymavlink_binaries, pymavlink_hiddenimports = collect_all("pymavlink")

a = Analysis(
    ["windows_entry.py"],
    pathex=[str(SPECPATH)],
    binaries=pymavlink_binaries,
    datas=pymavlink_datas,
    hiddenimports=pymavlink_hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    noarchive=False,
    optimize=1,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="CUADC中文靶标监控",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    console=False,
    version="windows_version_info.txt",
)

coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name="CUADC中文靶标监控",
)
