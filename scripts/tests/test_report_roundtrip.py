from scripts.e2e_reports import run


def test_onboard_report_survives_real_mavlink_serialization_and_tlog_decode(tmp_path):
    result = run(tmp_path)
    assert result["passed"]
    assert result["packets"] > 20
    assert result["selected_value"] == 12
    assert result["report"]["最终选靶"]["中文名称"] == "轰炸机"
