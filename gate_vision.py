"""
Gate detector for VQ1 (FlightSim 2.0).

Gates are bright pink/red glowing frames in a desaturated gray environment.
Verified against real simulator frames: HSV hue ~160-180, value >140.

detect_gate() is a pure function: image in, nearest-gate detection out.
Keep it that way -- it lets us test detection on saved frames without
running the simulator.
"""

from dataclasses import dataclass

import math
import cv2
import numpy as np

import config


@dataclass
class GateDetection:
    cx: float          # bbox center x (pixels)
    cy: float          # bbox center y (pixels)
    w: float           # bbox width (pixels)
    h: float           # bbox height (pixels)
    area_frac: float   # bbox area / frame area
    n_candidates: int  # how many gate-like blobs were visible
    frame_w: int
    frame_h: int


_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))

# Blue guidance-tube mask (bright cyan). Real gates sit ON the tube;
# decoys (start podiums, start-light lamps) do not.
BLUE_LO = (85, 60, 120)
BLUE_HI = (110, 255, 255)
SOLID_REJECT_FILL = 0.65     # candidates wider than SOLID_CHECK_W must be hollow
SOLID_CHECK_W = 25           # px
BLUE_NEAR_EXPAND = 0.6       # bbox expansion for blue proximity test
BLUE_NEAR_MIN_PX = 6


def detect_gate(img, prefer=None) -> "GateDetection | None":
    """Find the pink gate. If prefer=(cx, cy) is given (last known gate
    position), a candidate near that position wins over a LARGER blob
    elsewhere -- stops the detector flip-flopping between the current gate
    and the next one visible through/behind it, which fed garbage azimuth
    swings into the controller. Returns None if nothing found."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, config.GATE_HSV_LO_1, config.GATE_HSV_HI_1)
    m2 = cv2.inRange(hsv, config.GATE_HSV_LO_2, config.GATE_HSV_HI_2)
    mask = cv2.bitwise_or(m1, m2)
    # close small gaps so a gate's glowing frame becomes one blob
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    H, W = img.shape[:2]
    min_area = W * H * config.GATE_MIN_AREA_FRAC
    blue = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

    cands = []
    n = 0
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        if w * h < min_area:
            continue
        # ---- decoy rejection ----
        # candidates clipped by the frame border can't be judged by shape:
        # a half-visible gate looks like a sliver. Let them through.
        on_border = (x <= 1 or y <= 1 or x + w >= W - 2 or y + h >= H - 2)
        # 1) aspect: gates are roughly square; podium banners are tall
        aspect = w / float(h)
        if not on_border and (aspect < 0.45 or aspect > 2.2):
            continue
        # 2) hollowness: a close gate is a FRAME (ring), a podium is SOLID
        fill = cv2.countNonZero(mask[y:y + h, x:x + w]) / float(w * h)
        is_hollow = fill < SOLID_REJECT_FILL
        # 3) blue-tube proximity: gates thread the guidance tube
        ex, ey = int(w * BLUE_NEAR_EXPAND), int(h * BLUE_NEAR_EXPAND)
        x0, y0 = max(0, x - ex), max(0, y - ey)
        x1, y1 = min(W, x + w + ex), min(H, y + h + ey)
        blue_near = cv2.countNonZero(blue[y0:y1, x0:x1]) >= BLUE_NEAR_MIN_PX
        if not on_border and not is_hollow and not blue_near:
            continue          # solid blob with no guidance tube nearby: decoy
                              # (podium banners, start-light lamps)
        n += 1
        cands.append((w * h, x, y, w, h))

    if not cands:
        return None
    best = max(cands)
    if prefer is not None and len(cands) > 1:
        px, py = prefer
        lock_r = W * config.GATE_LOCK_RADIUS_FRAC
        near = [c for c in cands
                if math.hypot(c[1] + c[3] / 2.0 - px, c[2] + c[4] / 2.0 - py) <= lock_r]
        if near:
            # among candidates near the lock, still take the biggest
            best = max(near)
    _, x, y, w, h = best
    # Aim point: centroid of the gate's dark OPENING (the actual passage),
    # not the bbox center. From oblique angles these differ a lot.
    aim_x, aim_y = x + w / 2.0, y + h / 2.0
    if w >= 24 and h >= 24:
        mx0, my0 = x + int(w * 0.2), y + int(h * 0.2)
        mx1, my1 = x + int(w * 0.8), y + int(h * 0.8)
        core = mask[my0:my1, mx0:mx1]
        hole = (core == 0)
        n_hole = int(hole.sum())
        if n_hole > core.size * 0.12:
            ys, xs = np.nonzero(hole)
            aim_x = mx0 + float(xs.mean())
            aim_y = my0 + float(ys.mean())
    return GateDetection(
        cx=aim_x, cy=aim_y, w=float(w), h=float(h),
        area_frac=(w * h) / float(W * H), n_candidates=n,
        frame_w=W, frame_h=H,
    )


def annotate(img, det: "GateDetection | None", mode: str = ""):
    """Draw detection overlay on a copy of the frame (for debug logging)."""
    vis = img.copy()
    H, W = vis.shape[:2]
    cv2.drawMarker(vis, (W // 2, H // 2), (255, 255, 0), cv2.MARKER_TILTED_CROSS, 14, 1)
    if det is not None:
        p1 = (int(det.cx - det.w / 2), int(det.cy - det.h / 2))
        p2 = (int(det.cx + det.w / 2), int(det.cy + det.h / 2))
        cv2.rectangle(vis, p1, p2, (0, 255, 0), 2)
        cv2.drawMarker(vis, (int(det.cx), int(det.cy)), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.line(vis, (W // 2, H // 2), (int(det.cx), int(det.cy)), (0, 255, 255), 1)
        label = f"{mode} n={det.n_candidates} area={det.area_frac * 100:.1f}%"
    else:
        label = f"{mode} NO GATE"
    cv2.putText(vis, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return vis


class TubeInfo:
    __slots__ = ("near_x", "far_x", "visible")
    def __init__(self, near_x, far_x, visible):
        self.near_x = near_x    # -1..1 lateral offset of tube in NEAR band (bottom)
        self.far_x = far_x      # -1..1 lateral offset of tube in FAR band (middle)
        self.visible = visible


def detect_tube(img):
    """Locate the blue guidance tube in two horizontal bands.

    near band = bottom 20%% of frame (tube right under us -> strafe cue)
    far band  = 55-80%% rows (tube ahead -> heading cue)
    Returns TubeInfo with normalized x offsets, or visible=False."""
    H, W = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

    def band_x(y0, y1):
        band = blue[int(H * y0):int(H * y1), :]
        n = int(cv2.countNonZero(band))
        if n < config.TUBE_MIN_PIX:
            return None
        xs = cv2.findNonZero(band)
        cx = float(xs[:, 0, 0].mean())
        return (cx - W / 2.0) / (W / 2.0)

    near = band_x(0.92, 1.00)
    far = band_x(0.82, 0.92)
    if near is None and far is None:
        return TubeInfo(0.0, 0.0, False)
    if near is None: near = far
    if far is None: far = near
    return TubeInfo(near, far, True)
