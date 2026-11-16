import numpy as np

from cuadc_sim.pixel_rois import panel_pixels_support


def test_antialiased_thin_print_is_not_rejected_for_having_no_black_pixels():
    panel=np.full((20,20,3),225,np.uint8)
    panel[8:12,9:11]=130
    assert panel.min()>65
    assert panel_pixels_support(panel,(0,0,20,20))


def test_plain_substrate_and_saturated_colour_are_not_printed_panels():
    plain=np.full((20,20,3),200,np.uint8)
    assert not panel_pixels_support(plain,(0,0,20,20))
    plain[8:12,9:11]=(30,30,160)
    assert not panel_pixels_support(plain,(0,0,20,20))
