"""compute_layout — pick viewport + sidebar rectangles for the current window size."""
from constants import RIGHT_MARGIN_RATIO


def compute_layout(screen_size, bg_ratio):
    """Return (viewport_rect, sidebar_rect, orientation).

    In portrait mode the field can only occupy `w × (w / bg_ratio)`, so the
    leftover vertical space is folded into the sidebar (instead of wasted as
    black margin above/below the field). Sidebar gets at least RIGHT_MARGIN_RATIO
    of the screen so it doesn't collapse on near-square windows.
    """
    w, h = screen_size
    if h > w:
        min_sb_h = int(h * RIGHT_MARGIN_RATIO)
        field_h = int(w / bg_ratio)
        vp_h = min(field_h, h - min_sb_h)
        sb_h = h - vp_h
        return (0, 0, w, vp_h), (0, vp_h, w, sb_h), "bottom"
    min_sb_w = int(w * RIGHT_MARGIN_RATIO)
    field_w = int(h * bg_ratio)
    vp_w = min(field_w, w - min_sb_w)
    sb_w = w - vp_w
    return (0, 0, vp_w, h), (vp_w, 0, sb_w, h), "right"
