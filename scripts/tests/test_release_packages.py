from __future__ import annotations

from pathlib import Path
import os
import subprocess
import sys
import tarfile

from scripts.build_release_packages import build, ignored, main, export_directory


ROOT = Path(__file__).resolve().parents[2]


def test_private_fixed_arena_configuration_is_never_exported():
    from scripts.release_policy import excluded
    assert excluded(Path('gazebo_sim/config/arena.local.json'))
    assert not excluded(Path('gazebo_sim/config/arena.json'))


def test_workspace_keeps_simulation_and_onboard_separate():
    simulation = ROOT / "gazebo_sim"
    onboard = ROOT / "onboard"
    assert (simulation / "pyproject.toml").is_file()
    assert (simulation / "src/cuadc_sim").is_dir()
    assert (simulation / "tests").is_dir()
    assert (onboard / "pyproject.toml").is_file()
    assert (onboard / "src/cuadc_onboard").is_dir()
    assert (onboard / "tests").is_dir()
    for shared_root in ("src", "tests", "assets", "models", "tools"):
        assert not (ROOT / shared_root).exists()


def test_onboard_source_does_not_import_simulation_namespace():
    for source in (ROOT / "onboard/src").rglob("*.py"):
        text = source.read_text(encoding="utf-8")
        assert "cuadc_sim" not in text, source


def test_each_project_owns_its_runtime_resources():
    for project in (ROOT / "gazebo_sim", ROOT / "onboard"):
        assert (project / "assets/targets/catalog.json").is_file()
        assert (project / "models/vision/MODEL_SHA256SUMS").is_file()
        for task in ("task1_v4m_best", "task2_sim_digits_v1"):
            assert (project / f"models/vision/{task}.pt.gz").is_file()
    assert (ROOT / "gazebo_sim/config/ardupilot/plane.parm").is_file()
    assert (ROOT / "gazebo_sim/worlds/cuadc_world.sdf.in").is_file()
    assert (ROOT / "onboard/config/onboard.yaml").is_file()
    assert (ROOT / "onboard/systemd/cuadc-onboard.service.in").is_file()


def test_all_three_projects_are_open_source():
    simulation = ROOT / "gazebo_sim"
    onboard = ROOT / "onboard"
    assert (simulation / "LICENSE").read_text(encoding="utf-8").startswith(
        "MIT License"
    )
    onboard_license = (onboard / "LICENSE").read_text(encoding="utf-8")
    assert onboard_license.startswith("MIT License")
    assert (ROOT / "ground_station/LICENSE").read_text().startswith("MIT License")
    for required in ("LICENSE", "THIRD_PARTY_NOTICES.md", "pyproject.toml"):
        assert (simulation / required).is_file(), required
    for required in ("LICENSE", "THIRD_PARTY_NOTICES.md", "pyproject.toml"):
        assert (onboard / required).is_file(), required
    assert not (onboard / "CODE_OF_CONDUCT.md").exists()
    assert not (onboard / ".github/ISSUE_TEMPLATE").exists()
    assert not (onboard / ".github/PULL_REQUEST_TEMPLATE.md").exists()
    assert (ROOT / ".github/workflows/ci.yml").is_file()


def test_cli_builds_each_project_into_a_simple_named_directory(tmp_path: Path):
    assert main(["simulation", "--output", str(tmp_path)]) == 0
    assert main(["onboard", "--output", str(tmp_path)]) == 0
    assert list((tmp_path / "gazebo_sim").glob("*.tar.gz"))
    assert list((tmp_path / "onboard").glob("*.tar.gz"))
    assert (tmp_path / "gazebo_sim/SHA256SUMS").is_file()
    assert (tmp_path / "onboard/SHA256SUMS").is_file()


def test_release_archives_remain_strictly_separated(tmp_path: Path):
    onboard_archive = build("onboard", tmp_path)
    simulation_archive = build("simulation", tmp_path)
    with tarfile.open(onboard_archive) as stream:
        onboard_names = {Path(name).parts[1:] for name in stream.getnames()}
    with tarfile.open(simulation_archive) as stream:
        simulation_names = {Path(name).parts[1:] for name in stream.getnames()}
    assert ("src", "cuadc_onboard", "runtime.py") in onboard_names
    assert ("src", "cuadc_sim", "companion.py") not in onboard_names
    assert ("worlds", "cuadc_world.sdf.in") not in onboard_names
    assert ("src", "cuadc_sim", "companion.py") in simulation_names
    assert ("src", "cuadc_onboard", "runtime.py") not in simulation_names
    assert ("config", "onboard.yaml") not in simulation_names
    assert ("README.md",) in simulation_names
    assert ("tests", "test_simulation_contract.py") in simulation_names
    assert (".github", "workflows", "ci.yml") not in simulation_names
    assert (".gitignore",) in simulation_names
    assert (".gitattributes",) in simulation_names
    hidden_directories = {
        ".git",
        ".pytest_cache",
        ".venv",
        ".deps",
        ".runtime",
        "runs",
        "dist",
        "build",
    }
    assert not any(hidden_directories.intersection(parts) for parts in simulation_names)
    assert not any(
        Path(parts[-1]).suffix in {".pdf", ".log", ".tlog", ".ulg", ".bag", ".mp4", ".mov", ".pt"}
        for parts in simulation_names
        if parts
    )
    assert ("models", "vision", "task1_v4m_best.pt.gz") in simulation_names
    assert ("models", "vision", "task2_sim_digits_v1.pt.gz") in simulation_names
    assert ("models", "vision", "task1_v4m_best.pt.gz") in onboard_names
    assert ("models", "vision", "task2_sim_digits_v1.pt.gz") in onboard_names
    assert not any("yolov8n_pretraining.pt.gz" in parts for parts in simulation_names)
    assert not any("sichuan" in part.lower() for parts in simulation_names for part in parts)
    assert ("README.md",) in onboard_names
    assert ("tests", "test_onboard.py") in onboard_names


def test_package_filter_excludes_local_secrets_but_keeps_public_env_example():
    names = [
        ".env",
        ".env.local",
        ".env.example",
        "session.tlog",
        "recording.mp4",
        ".pytest_cache",
    ]
    assert ignored(".", names) == {
        "session.tlog",
        "recording.mp4",
        ".pytest_cache",
    }


def test_directory_export_is_regular_audited_and_refuses_overwrite(tmp_path, monkeypatch):
    import pytest
    from scripts import build_release_packages as packages
    source=tmp_path/'source'
    source.mkdir()
    (source/'README.md').write_text('公开项目\n')
    (source/'LICENSE').write_text('MIT License\n')
    (source/'pyproject.toml').write_text('[project]\nname="sample"\nversion="1.0"\n')
    (source/'.git').mkdir()
    (source/'.git'/'secret').write_text('private history')
    image=source/'datasets/v1/images/sample.jpg'
    image.parent.mkdir(parents=True)
    image.write_bytes(b'local dataset content; must not be exported')
    monkeypatch.setitem(packages.PROJECT_DIRS,'simulation',source)
    result=export_directory('simulation',tmp_path/'export',link_dataset_files=True)
    assert not (result/'.git').exists()
    assert not any(path.is_symlink() for path in result.rglob('*'))
    assert (result/'PACKAGE_SHA256SUMS').is_file()
    assert (result/'README.md').stat().st_ino != (source/'README.md').stat().st_ino
    with pytest.raises(FileExistsError):
        export_directory('simulation',result)


def test_public_directory_export_includes_reviewed_model_archives_only(tmp_path: Path):
    """Only the two reviewed runtime models enter a public export."""
    result = export_directory("public", tmp_path / "cuadc-source")
    assert sorted(path.name for path in result.rglob("*.pt.gz")) == sorted(
        [
            "task1_v4m_best.pt.gz",
            "task2_sim_digits_v1.pt.gz",
            "task1_v4m_best.pt.gz",
            "task2_sim_digits_v1.pt.gz",
        ]
    )
    assert not list(result.rglob("yolov8n_pretraining.pt.gz"))
    assert not list(result.rglob("*.onnx.gz"))
    assert (result / "gazebo_sim/pyproject.toml").is_file()
    assert (result / "onboard/pyproject.toml").is_file()
    assert not list((result / "gazebo_sim/assets/targets").rglob("*.png"))
    assert not (result / "gazebo_sim/datasets/samples").exists()
    assert not (result / "gazebo_sim/models/vision/training").exists()


def test_public_export_runs_from_outside_checkout_without_private_assets(tmp_path: Path):
    result = export_directory("public", tmp_path / "cuadc-source")
    runtime = tmp_path / "runtime"
    environment = {
        **os.environ,
        "PYTHONPATH": str(result / "gazebo_sim/src"),
        "CUADC_RUNTIME_DIR": str(runtime),
    }
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "cuadc_sim.cli",
            "generate-world",
            "--task",
            "2",
            "--seed",
            "1",
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    assert '"world"' in completed.stdout
    world = (runtime / "cuadc_world.sdf").read_text(encoding="utf-8")
    assert world.count('<visual name="target_surface">') == 6
    assert world.count("<albedo_map>") < 6


def test_onboard_one_command_deployer_has_valid_shell_syntax():
    deployer = ROOT / "onboard/scripts/deploy_onboard.sh"
    subprocess.run(["bash", "-n", str(deployer)], check=True)
    source = deployer.read_text(encoding="utf-8")
    assert '"$SCRIPT_DIR/install_onboard.sh"' in source
    assert '"$SCRIPT_DIR/install_onboard_service.sh"' in source
    assert "systemctl restart cuadc-onboard.target" in source


def test_clean_checkout_has_a_documented_no_vision_simulator_path():
    bootstrap = ROOT / "gazebo_sim/scripts/bootstrap.sh"
    runner = ROOT / "gazebo_sim/scripts/run_sim.sh"
    subprocess.run(["bash", "-n", str(bootstrap)], check=True)
    subprocess.run(["bash", "-n", str(runner)], check=True)
    assert "--no-vision" in bootstrap.read_text(encoding="utf-8")
    assert "--no-vision" in runner.read_text(encoding="utf-8")
    assert (ROOT / "gazebo_sim/requirements-core.txt").is_file()


def test_onboard_systemd_runs_hardware_preflight_before_algorithm():
    unit = (ROOT / "onboard/systemd/cuadc-onboard.service.in").read_text(
        encoding="utf-8"
    )
    wait_index = unit.index("ExecStartPre=@WAIT_SCRIPT@")
    check_index = unit.index("ExecStartPre=@PROJECT_DIR@/.venv-onboard/bin/cuadc-onboard")
    run_index = unit.index("ExecStart=@PROJECT_DIR@/.venv-onboard/bin/cuadc-onboard")
    assert wait_index < check_index < run_index
    assert "--camera --serial --timeout 60" in unit
    assert "--config @CONFIG_PATH@ check --skip-camera" in unit
    assert "TimeoutStopSec=45" in unit
    assert "reset_uvc_camera" not in unit


def test_onboard_production_code_cannot_arm_switch_mode_or_start_a_mission():
    production = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "onboard/src/cuadc_onboard").glob("*.py")
    )
    for forbidden in (
        "MAV_CMD_COMPONENT_ARM_DISARM",
        "MAV_CMD_DO_SET_MODE",
        "MAV_CMD_MISSION_START",
        "set_mode_send",
    ):
        assert forbidden not in production

    # Finite rescan intentionally moves the active mission index back to the
    # first scan waypoint.  This is neither arming nor MISSION_START, but the
    # raw MAVLink side effect must stay inside the transport wrapper so mission
    # policy cannot send it from arbitrary modules.
    uses = [
        path.relative_to(ROOT).as_posix()
        for path in (ROOT / "onboard/src/cuadc_onboard").glob("*.py")
        if "mission_set_current_send" in path.read_text(encoding="utf-8")
    ]
    assert uses == ["onboard/src/cuadc_onboard/mavlink_io.py"]


def test_onboard_installs_modemmanager_ignore_rule_for_cubepilot():
    rule = ROOT / "onboard/udev/99-cuadc-flight-controller.rules"
    installer = (ROOT / "onboard/scripts/install_onboard_service.sh").read_text(
        encoding="utf-8"
    )
    text = rule.read_text(encoding="utf-8")
    assert 'ATTRS{idVendor}=="2dae"' in text
    assert 'ATTRS{idProduct}=="1058"' in text
    assert 'ENV{ID_MM_DEVICE_IGNORE}="1"' in text
    assert "99-cuadc-flight-controller.rules" in installer
    assert "udevadm control --reload-rules" in installer
    assert "--property-match=ID_VENDOR_ID=2dae" in installer
    assert "--property-match=ID_MODEL_ID=1058" in installer


def test_onboard_uses_pinned_cpu_only_pytorch_builds():
    requirements = (ROOT / "onboard/requirements.txt").read_text(encoding="utf-8")
    assert "torch-2.13.0%2Bcpu" in requirements
    assert "torchvision-0.28.0%2Bcpu" in requirements
    assert requirements.count("https://download.pytorch.org/whl/cpu/") == 2


def test_ground_station_has_reproducible_windows_onedir_packaging():
    ground_station = ROOT / "ground_station"
    spec = (ground_station / "CUADCGroundStation.spec").read_text(encoding="utf-8")
    powershell = (ground_station / "build_windows.ps1").read_text(encoding="utf-8")
    assert (ground_station / "windows_entry.py").is_file()
    assert (ROOT / "README.md").is_file()
    assert (ground_station / "build_windows.bat").is_file()
    assert 'collect_all("pymavlink")' in spec
    assert 'console=False' in spec
    assert 'name="CUADC中文靶标监控"' in spec
    assert "Compress-Archive" in powershell
    assert "Get-FileHash -Algorithm SHA256" in powershell
    assert "Mission Planner" in (ROOT / "README.md").read_text(
        encoding="utf-8"
    )
