from cuadc_sim.selection import (
    Detection,
    aggregate_detections,
    select_target,
)


def detection(track, value, confidence, north, east=0.0, **kwargs):
    return Detection(track, value, confidence, north_m=north, east_m=east, **kwargs)


def test_task1_selects_highest_value_not_highest_confidence():
    selection = select_target(
        1,
        [
            detection("T01", 12, 0.40, 180),
            detection("T02", 7, 0.99, 200),
            detection("T03", 2, 0.95, 220),
        ],
    )
    assert selection.track_id == "T01"
    assert selection.value == 12
    assert not selection.guessed


def test_task1_rejects_weak_fourth_cluster_before_highest_value_rule():
    selection = select_target(
        1,
        [
            detection("T01", 9, 0.99, 182, -17, frames=4),
            detection("T02", 6, 0.99, 182, 17, frames=3),
            detection("T03", 5, 0.99, 218, -17, frames=2),
            detection("T04", 12, 0.98, 218, 17, frames=1),
        ],
    )
    assert selection.track_id == "T01"
    assert selection.value == 9
    assert not selection.guessed


def test_task2_selects_numeric_median_including_leading_zero():
    selection = select_target(
        2,
        [
            detection("T01", 0, 0.70, -180, label="00"),
            detection("T02", 44, 0.80, -200, label="44"),
            detection("T03", 99, 0.90, -220, label="99"),
        ],
    )
    assert selection.track_id == "T02"
    assert selection.value == 44
    assert not selection.guessed


def test_task2_two_target_guess_uses_value_nearest_49_5():
    selection = select_target(
        2,
        [
            detection("T01", 4, 0.99, 180),
            detection("T02", 60, 0.55, 220),
        ],
    )
    assert selection.track_id == "T02"
    assert selection.guessed


def test_task2_one_target_guess_uses_that_target():
    selection = select_target(2, [detection("T01", 7, 0.4, 200)])
    assert selection.track_id == "T01"
    assert selection.guessed


def test_aggregation_clusters_by_estimated_position_and_keeps_best_class():
    aggregate = aggregate_detections(
        [
            detection("", 10, 0.7, 200.0, 0.0),
            detection("", 10, 0.8, 201.0, 0.5),
            detection("", 11, 0.9, 200.5, -0.5),
            detection("", 8, 0.8, 218.0, 0.0),
        ]
    )
    assert len(aggregate) == 2
    assert aggregate[0].track_id == "T01"
    assert aggregate[0].value == 10
    assert aggregate[0].confidence > 0.9
    assert 200.0 < aggregate[0].north_m < 201.0


def test_no_detection_is_explicit_center_guess():
    selection = select_target(1, [])
    assert selection.track_id is None
    assert selection.source == "NO_DETECTION"
    assert selection.guessed


def test_distinct_frame_majority_beats_a_confident_minority():
    fused=aggregate_detections([
        detection('',9,.65,100,frames=4),
        detection('',8,.99,100.1,frames=2),
    ])
    assert len(fused)==1 and fused[0].value==9 and fused[0].frames==4


def test_same_frame_duplicate_boxes_cannot_cast_extra_votes():
    fused=aggregate_detections([
        detection('',9,.9,100),detection('',3,.6,100.1),detection('',3,.65,100.2),
    ],prefer_frame_votes=False)
    assert len(fused)==1 and fused[0].value==9 and fused[0].frames==1
