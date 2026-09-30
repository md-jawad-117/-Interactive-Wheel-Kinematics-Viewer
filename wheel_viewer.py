"""Wheel kinematics viewer.

Pick a drive type, command a body velocity (vx forward, vy left, w turn left)
and watch how every wheel has to spin to make that motion happen.

  stripes moving on a wheel = it is rolling (they move the way the top of the tyre moves)
  green = spinning forward, red = spinning backward, grey = stopped
  orange arrow = wheel being dragged sideways (slip / scrubbing)

Drive with the sliders or hold keys: W/S forward/back, A/D turn, Q/E strafe,
Space stop, R reset, M demo, 1-5 pick drive type.

    python wheel_viewer.py
"""
import math
from collections import deque

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, FancyArrowPatch, Polygon, Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.widgets import Button, RadioButtons, Slider

R = 0.05            # wheel radius (m)
WW = 0.04           # wheel width, top view (m)
A, B = 0.18, 0.15   # half wheelbase (front/back), half track (left/right)
KIWI = 0.17         # omni wheel distance from centre
MAX_STEER = math.radians(35)
DT = 0.03

DEMO = [  # (label, vx, vy, w), 3 s each
    ("forward", 0.3, 0, 0),
    ("reverse", -0.3, 0, 0),
    ("spin in place", 0, 0, 1.5),
    ("arc left", 0.3, 0, 1.0),
    ("strafe left", 0, 0.3, 0),
    ("diagonal", 0.25, 0.25, 0),
    ("strafe + spin", 0, 0.3, 1.0),
]


def wheel(name, x, y, kind, heading=0.0, s=0, r=R):
    return dict(name=name, pos=(x, y), kind=kind, heading=heading, s=s, r=r, last=heading)


def four(kind, s=(0, 0, 0, 0)):
    return [wheel("FL", A, B, kind, s=s[0]), wheel("FR", A, -B, kind, s=s[1]),
            wheel("RL", -A, B, kind, s=s[2]), wheel("RR", -A, -B, kind, s=s[3])]


ROBOTS = {
    "2-wheel differential": dict(
        drive="diff", body=("rect", 0.30, 0.23),
        wheels=[wheel("L", 0, B, "fixed"), wheel("R", 0, -B, "fixed"),
                wheel("caster", -0.12, 0, "caster", r=0.025)],
        eq="w_L = (vx - w*b) / r\n"
           "w_R = (vx + w*b) / r\n\n"
           "Turns by spinning left and\n"
           "right at different speeds.\n"
           "vy is ignored: it cannot\n"
           "move sideways. The caster\n"
           "just swivels to follow."),
    "4-wheel skid steer": dict(
        drive="skid", body=("rect", 2 * A + 0.06, 2 * B - 0.07),
        wheels=four("fixed"),
        eq="left pair  = (vx - w*b) / r\n"
           "right pair = (vx + w*b) / r\n\n"
           "Same maths as differential,\n"
           "but when turning the front\n"
           "and rear wheels get dragged\n"
           "sideways (orange). That scrub\n"
           "is why skid-steer odometry\n"
           "is bad at measuring turns."),
    "4-wheel mecanum": dict(
        drive="holo", body=("rect", 2 * A + 0.06, 2 * B - 0.07),
        wheels=four("mecanum", (-1, 1, 1, -1)),
        eq="k = a + b\n"
           "FL = (vx - vy - k*w) / r\n"
           "FR = (vx + vy + k*w) / r\n"
           "RL = (vx + vy - k*w) / r\n"
           "RR = (vx - vy + k*w) / r\n\n"
           "45 deg rollers turn part of\n"
           "each wheel's push sideways,\n"
           "so it can strafe and spin\n"
           "at the same time."),
    "3-wheel omni (kiwi)": dict(
        drive="holo", body=("circle", KIWI),
        wheels=[wheel(n, KIWI * math.cos(p), KIWI * math.sin(p), "omni", heading=p + math.pi / 2)
                for n, p in (("F", 0.0), ("BL", 2 * math.pi / 3), ("BR", -2 * math.pi / 3))],
        eq="w_i = (-sin(p_i)*vx\n"
           "       + cos(p_i)*vy + L*w) / r\n"
           "p_i = wheel angle (0,120,240)\n\n"
           "Each wheel only pushes along\n"
           "its own direction; rollers\n"
           "slide freely the other way.\n"
           "Moves in any direction."),
    "4-wheel car (Ackermann)": dict(
        drive="ackermann", body=("rect", 2 * A + 0.06, 2 * B - 0.07),
        wheels=[wheel("FL", A, B, "steer"), wheel("FR", A, -B, "steer"),
                wheel("RL", -A, B, "fixed"), wheel("RR", -A, -B, "fixed")],
        eq="curvature = w / vx\n"
           "steer = atan(wheelbase * w / vx)\n\n"
           "Front wheels steer so every\n"
           "wheel circles the same point:\n"
           "inner wheel steers more.\n"
           "Cannot turn in place, and\n"
           "turn rate is limited by speed\n"
           "and max steer (35 deg)."),
}


def solve(robot, vx, vy, w):
    """Body command -> what each wheel does. Returns actual (vx, vy, w) and per wheel (heading, omega, slip)."""
    drive = robot["drive"]
    if drive in ("diff", "skid", "ackermann"):
        vy = 0.0
    if drive == "ackermann":
        k_max = math.tan(MAX_STEER) / (2 * A)
        w = float(np.clip(w, -abs(vx) * k_max, abs(vx) * k_max))
        vy = w * A  # body frame is at the centre, rear axle must not slide sideways

    out = []
    for wh in robot["wheels"]:
        px, py = wh["pos"]
        cx, cy = vx - w * py, vy + w * px  # ground speed of this wheel's contact point
        h = wh["heading"]
        if wh["kind"] in ("steer", "caster"):
            if math.hypot(cx, cy) > 1e-4:
                h = math.atan2(cy, cx)
                if wh["kind"] == "steer" and cx < 0:
                    h = math.atan2(-cy, -cx)  # reversing: keep the wheel facing forward
                wh["last"] = h
            h = wh["last"]
        roll = cx * math.cos(h) + cy * math.sin(h)
        slip = -cx * math.sin(h) + cy * math.cos(h)
        if wh["kind"] == "mecanum":
            roll, slip = cx + wh["s"] * cy, 0.0  # rollers take up the rest
        elif wh["kind"] != "fixed":
            slip = 0.0  # omni rollers, steering and casters absorb sideways motion
        out.append((h, roll / wh["r"], slip))
    return (vx, vy, w), out



def make_wheel(ax, body_tf, wh):
    """Create one wheel's artists once; update_wheel moves them every frame."""
    r = wh["r"]
    L, W = 2 * r, (0.02 if wh["kind"] == "caster" else WW)
    aff = Affine2D()
    tf = aff + body_tf
    rect = Rectangle((-L / 2, -W / 2), L, W, transform=tf, ec="black", lw=1, zorder=3, animated=True)
    ax.add_patch(rect)
    skew = math.radians(-45 * wh["s"]) if wh["kind"] == "mecanum" else 0.0
    dx = W / 2 * math.tan(skew)
    stripes = []
    for _ in range(7 if wh["kind"] == "mecanum" else 5):
        line = Line2D([], [], transform=tf, color="white", lw=1.4, zorder=4, animated=True)
        line.set_clip_path(rect)
        ax.add_line(line)
        stripes.append(line)
    arts = [rect, *stripes]
    if wh["kind"] == "omni":
        arts.append(ax.add_line(Line2D([-L / 2, L / 2], [0, 0], transform=tf, color="black",
                                       lw=0.6, ls=":", zorder=4, animated=True)))
    slip = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=12, color="orange",
                           lw=2, transform=tf, zorder=5, animated=True)
    ax.add_patch(slip)
    arts.append(slip)
    return dict(wh=wh, aff=aff, rect=rect, stripes=stripes, slip=slip, L=L, W=W, dx=dx,
                span=L + 2 * abs(dx)), arts


def update_wheel(w, h, phase, omega, slip):
    wh = w["wh"]
    w["aff"].clear().rotate(h).translate(*wh["pos"])
    w["rect"].set_facecolor(color_for(omega, "#616161"))
    n, span, dx, W = len(w["stripes"]), w["span"], w["dx"], w["W"]
    for k, line in enumerate(w["stripes"]):
        u = (k * span / n + phase * wh["r"]) % span - span / 2
        line.set_data([u - dx, u + dx], [-W / 2, W / 2])
    w["slip"].set_visible(abs(slip) > 0.01)
    w["slip"].set_positions((0, 0), (0, slip * 0.6))


def color_for(omega, idle):
    return "#2e7d32" if omega > 0.3 else "#c62828" if omega < -0.3 else idle


# key -> (axis, direction); axis 0 = vx, 1 = vy, 2 = w
DRIVE_KEYS = {"w": (0, 1), "up": (0, 1), "s": (0, -1), "down": (0, -1),
              "q": (1, 1), "e": (1, -1),
              "a": (2, 1), "left": (2, 1), "d": (2, -1), "right": (2, -1)}
MAX_CMD = (0.5, 0.5, 2.0)
ACCEL = (1.0, 1.0, 4.0)  # per second, so keys ramp up/down like an RC car
VIEW = 0.7               # half-size of the view around the robot (m)
GRID = 0.25


def build():
    # free up the keys matplotlib normally uses (s = save, q = quit, arrows = history...)
    for k in list(plt.rcParams):
        if k.startswith("keymap."):
            plt.rcParams[k] = []

    fig = plt.figure(figsize=(15, 8))
    fig.canvas.manager.set_window_title("Wheel kinematics viewer")
    ax = fig.add_axes([0.22, 0.2, 0.46, 0.75])
    ax_bar = fig.add_axes([0.73, 0.55, 0.25, 0.4])
    ax_txt = fig.add_axes([0.71, 0.03, 0.28, 0.42])

    radio = RadioButtons(fig.add_axes([0.01, 0.55, 0.19, 0.38]),
                         [f"{i + 1}  {n}" for i, n in enumerate(ROBOTS)])
    b_stop = Button(fig.add_axes([0.01, 0.46, 0.09, 0.05]), "Stop")
    b_demo = Button(fig.add_axes([0.11, 0.46, 0.09, 0.05]), "Demo: off")
    b_reset = Button(fig.add_axes([0.01, 0.39, 0.19, 0.05]), "Reset position")
    sliders = [Slider(fig.add_axes([0.28, y, 0.36, 0.025]), lab, -m, m, valinit=0.0)
               for y, lab, m in ((0.11, "vx fwd (m/s)", 0.5), (0.07, "vy left (m/s)", 0.5),
                                 (0.03, "w turn (rad/s)", 2.0))]
    fig.text(0.01, 0.35, "Keys (hold to drive)\n"
                         "  W/S  or Up/Down     forward / back\n"
                         "  A/D  or Left/Right  turn\n"
                         "  Q/E                 strafe left / right\n"
                         "  Space stop   R reset   M demo\n"
                         "  1-5  pick drive type\n\n"
                         "green = wheel spinning forward\nred = spinning backward\n"
                         "orange = dragged sideways\nblue = robot velocity\n"
                         "Stripes = top of the tyre rolling.",
             fontsize=8.5, va="top", family="monospace")

    # world view is drawn around the robot, so the axes limits never change and we can blit
    for a in (ax, ax_bar, ax_txt):
        a.set_xticks([])
        a.set_yticks([])
    ax.set_xlim(-VIEW, VIEW)
    ax.set_ylim(-VIEW, VIEW)
    ax.set_aspect("equal")
    ax_bar.set_xlim(-0.5, 3.5)
    ax_bar.set_ylim(-40, 40)
    ax_bar.set_yticks(range(-40, 41, 10))
    ax_bar.set_ylabel("wheel speed (rad/s)")
    ax_bar.set_title("How fast each wheel spins")
    ax_bar.axhline(0, color="black", lw=0.8)
    ax_txt.axis("off")

    grid = LineCollection([], colors="#dddddd", lw=1, zorder=0, animated=True)
    ax.add_collection(grid)
    trail = ax.add_line(Line2D([], [], color="#90caf9", lw=1.5, zorder=1, animated=True))
    vel = FancyArrowPatch((0, 0), (0, 0), arrowstyle="-|>", mutation_scale=18, color="#1565c0",
                          lw=2.5, zorder=6, animated=True)
    ax.add_patch(vel)
    title = ax.text(0.5, 0.97, "", transform=ax.transAxes, ha="center", va="top", fontsize=13,
                    animated=True)
    # big text is slow to draw, so the equations are plain (redrawn only on robot change)
    eq = ax_txt.text(0, 1, "", va="top", family="monospace", fontsize=9.5)
    live = ax.text(0.02, 0.02, "", transform=ax.transAxes, va="bottom", family="monospace",
                   fontsize=9, animated=True)
    slider_arts = []
    for s in sliders:
        s.drawon = False  # we redraw them ourselves in the blit
        for a in (s.poly, s._handle, s.valtext):
            a.set_animated(True)
            slider_arts.append(a)
    fixed_arts = [grid, trail, vel, title, live, *slider_arts]

    body_aff = Affine2D()
    body_tf = body_aff + ax.transData
    st = dict(name=None, pose=[0.0, 0.0, 0.0], trail=deque(maxlen=600), cmd=[0.0, 0.0, 0.0],
              keyed=[False, False, False], held=set(), demo=False, t=0.0, step=-1,
              robot_arts=[], bar_arts=[], wheels=[], bars=[], phases=None)

    def pick(label):
        name = label.split("  ", 1)[1]
        for a in st["robot_arts"] + st["bar_arts"]:
            a.remove()
        robot = ROBOTS[name]
        shape = robot["body"]
        if shape[0] == "rect":
            body = Rectangle((-shape[1] / 2, -shape[2] / 2), shape[1], shape[2], transform=body_tf,
                             fc="#eceff1", ec="#37474f", lw=1.5, zorder=2, animated=True)
            nose = shape[1] / 2
        else:
            body = Circle((0, 0), shape[1], transform=body_tf, fc="#eceff1", ec="#37474f", lw=1.5,
                          zorder=2, animated=True)
            nose = shape[1] * 0.8
        tip = Polygon([(nose - 0.06, 0.04), (nose - 0.06, -0.04), (nose, 0)], transform=body_tf,
                      fc="#37474f", zorder=2, animated=True)
        ax.add_patch(body)
        ax.add_patch(tip)
        arts, wheels = [body, tip], []
        for wh in robot["wheels"]:
            w, a = make_wheel(ax, body_tf, wh)
            wheels.append(w)
            arts += a

        bars, bar_arts = [], []
        for x, wh in zip(np.linspace(0, 3, len(robot["wheels"])), robot["wheels"]):
            rect = ax_bar.add_patch(Rectangle((x - 0.35, 0), 0.7, 0, animated=True))
            rpm = ax_bar.text(x, 0, "", ha="center", fontsize=8, animated=True)
            lab = ax_bar.text(x, -38, wh["name"], ha="center", va="bottom", fontsize=10, animated=True)
            bars.append((x, rect, rpm))
            bar_arts += [rect, rpm, lab]

        eq.set_text(robot["eq"])
        fig.canvas.draw_idle()
        st.update(name=name, robot_arts=arts, wheels=wheels, bars=bars, bar_arts=bar_arts,
                  phases=np.zeros(len(robot["wheels"])))

    def on_slider(i, v):
        st["cmd"][i] = v
        st["keyed"][i] = False

    def stop(_=None):
        st["demo"] = False
        b_demo.label.set_text("Demo: off")
        st["cmd"][:] = [0.0, 0.0, 0.0]
        fig.canvas.draw_idle()

    def demo(_=None):
        st["demo"], st["t"], st["step"] = not st["demo"], 0.0, -1
        b_demo.label.set_text("Demo: on" if st["demo"] else "Demo: off")
        fig.canvas.draw_idle()

    def reset(_=None):
        st["pose"][:] = [0.0, 0.0, 0.0]
        st["trail"].clear()

    def key_name(event):
        return (event.key or "").split("+")[-1].lower()

    def on_press(event):
        k = key_name(event)
        if k in DRIVE_KEYS:
            st["held"].add(k)
            if st["demo"]:
                demo()
        elif k in (" ", "space"):
            stop()
        elif k == "r":
            reset()
        elif k == "m":
            demo()
        elif k.isdigit() and 1 <= int(k) <= len(ROBOTS):
            radio.set_active(int(k) - 1)

    def on_release(event):
        st["held"].discard(key_name(event))

    radio.on_clicked(pick)
    b_stop.on_clicked(stop)
    b_demo.on_clicked(demo)
    b_reset.on_clicked(reset)
    for i, s in enumerate(sliders):
        s.on_changed(lambda v, i=i: on_slider(i, v))
    fig.canvas.mpl_connect("key_press_event", on_press)
    fig.canvas.mpl_connect("key_release_event", on_release)
    pick(radio.value_selected)

    def drive_from_keys():
        push = [0, 0, 0]
        for k in st["held"]:
            i, d = DRIVE_KEYS[k]
            push[i] += d
        for i in range(3):
            if push[i]:
                st["keyed"][i] = True
                target = math.copysign(MAX_CMD[i], push[i])
            elif st["keyed"][i]:
                target = 0.0
            else:
                continue  # this axis belongs to the slider
            step = ACCEL[i] * DT
            st["cmd"][i] += float(np.clip(target - st["cmd"][i], -step, step))
            if not push[i] and st["cmd"][i] == 0.0:
                st["keyed"][i] = False

    def update(_):
        label = ""
        if st["demo"]:
            st["t"] += DT
            i = int(st["t"] // 3) % len(DEMO)
            label = DEMO[i][0]
            if i != st["step"]:
                st["step"] = i
                st["cmd"][:] = DEMO[i][1:]
        else:
            drive_from_keys()
        for s, v in zip(sliders, st["cmd"]):
            if abs(s.val - v) > 1e-6:
                s.eventson = False
                s.set_val(v)
                s.eventson = True

        robot = ROBOTS[st["name"]]
        (vx, vy, w), states = solve(robot, *st["cmd"])
        x, y, th = st["pose"]
        vwx, vwy = vx * math.cos(th) - vy * math.sin(th), vx * math.sin(th) + vy * math.cos(th)
        x, y, th = x + vwx * DT, y + vwy * DT, th + w * DT
        st["pose"][:] = [x, y, th]
        st["trail"].append((x, y))
        st["phases"] += np.array([om for _, om, _ in states]) * DT

        # the world scrolls past the robot
        lines = np.arange(-VIEW - GRID, VIEW + 2 * GRID, GRID)
        gx, gy = lines - x % GRID, lines - y % GRID
        grid.set_segments([[(g, -1), (g, 1)] for g in gx] + [[(-1, g), (1, g)] for g in gy])
        tr = np.array(st["trail"]) - (x, y)
        trail.set_data(tr[:, 0], tr[:, 1])
        body_aff.clear().rotate(th)
        for wart, (h, om, slip), ph in zip(st["wheels"], states, st["phases"]):
            update_wheel(wart, h, ph, om, slip)
        vel.set_visible(math.hypot(vwx, vwy) > 0.01)
        vel.set_positions((0, 0), (vwx * 0.8, vwy * 0.8))
        title.set_text(st["name"] + (f"   [demo: {label}]" if label else ""))

        for (bx, rect, rpm), (_, om, _) in zip(st["bars"], states):
            rect.set_height(om)
            rect.set_facecolor(color_for(om, "#9e9e9e"))
            rpm.set_position((bx, om + (1.5 if om >= 0 else -1.5)))
            rpm.set_va("bottom" if om >= 0 else "top")
            rpm.set_text(f"{om * 60 / (2 * math.pi):.0f} rpm")

        note = ""
        if abs(st["cmd"][2] - w) > 1e-3:
            note = "\n(turn limited by this drive)"
        if robot["drive"] != "holo" and abs(st["cmd"][1]) > 1e-3:
            note += "\n(vy ignored: cannot strafe)"
        live.set_text(f"actual: vx={vx:+.2f}  vy={vy:+.2f}  w={w:+.2f}" + note)
        return fixed_arts + st["robot_arts"] + st["bar_arts"]

    anim = FuncAnimation(fig, update, interval=int(DT * 1000), blit=True, cache_frame_data=False)
    return dict(fig=fig, anim=anim, update=update, pick=pick, sliders=sliders, radio=radio,
                press=on_press, release=on_release, state=st)


if __name__ == "__main__":
    app = build()
    plt.show()
