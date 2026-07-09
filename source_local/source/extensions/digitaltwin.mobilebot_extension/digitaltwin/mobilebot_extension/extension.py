import json
import math
import time
import urllib.error
import urllib.request

import omni.ext
import omni.kit.app
import omni.ui as ui
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdShade


# =============================================================================
# ROBOT BODY PARAMETERS
# Y-up convention: XZ = ground plane, Y = vertical. Distances are centimetres.
# =============================================================================

BODY_RADIUS = 20.0
BODY_HEIGHT = 12.0
BODY_Y_OFFSET = 0.0


# =============================================================================
# WHEEL PARAMETERS
# =============================================================================

WHEEL_ANGLES_DEG = [90.0, 210.0, 330.0]
WHEEL_OFFSET_R = 18.0
WHEEL_RADIUS = 4.0
WHEEL_WIDTH = 3.0
WHEEL_Y_CENTRE = WHEEL_RADIUS
WHEEL_Y_OFFSETS = [0.0, 0.0, 0.0]


# =============================================================================
# VELOCITY ARROW
# =============================================================================

VELOCITY_ARROW_SCALE = 30.0
VELOCITY_ARROW_Y = BODY_Y_OFFSET + BODY_HEIGHT + 4.0


# =============================================================================
# SYNCra STATE SERVICE
# =============================================================================

HTTP_STATE_URL = "http://127.0.0.1:8000/state/latest"
HTTP_POLL_INTERVAL_S = 0.5
HTTP_TIMEOUT_S = 0.15


# =============================================================================
# FORWARD KINEMATICS HELPER
# Retained for simple tests and for comparing legacy wheel-only data.
# =============================================================================

def _wheel_angles_rad():
    return [math.radians(a) for a in WHEEL_ANGLES_DEG]


def wheel_to_body_velocity(w0, w1, w2):
    angles = _wheel_angles_rad()
    radius_m = WHEEL_OFFSET_R / 100.0
    vx = (1.0 / 3.0) * (
        -math.sin(angles[0]) * w0
        - math.sin(angles[1]) * w1
        - math.sin(angles[2]) * w2
    )
    vz = (1.0 / 3.0) * (
        math.cos(angles[0]) * w0
        + math.cos(angles[1]) * w1
        + math.cos(angles[2]) * w2
    )
    omega = (w0 + w1 + w2) / (3.0 * radius_m)
    return vx, vz, omega


# =============================================================================
# USD HELPERS
# =============================================================================

def _get_or_create_stage():
    ctx = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None:
        ctx.new_stage()
        stage = ctx.get_stage()
    return stage


def _make_material(stage, path, color, roughness=0.4, metallic=0.2):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def _translate(prim, x, y, z):
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(x, y, z))


def _rotate_y_deg(prim, deg):
    UsdGeom.XformCommonAPI(prim).SetRotate(
        Gf.Vec3f(0, deg, 0),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )


def _wheel_orient_quat(angle_rad):
    sin_a = math.sin(angle_rad)
    cos_a = math.cos(angle_rad)
    half = math.sqrt(2.0) / 2.0
    return Gf.Quatf(half, Gf.Vec3f(half * sin_a, 0.0, half * -cos_a))


def _set_orient(prim, quat):
    xform = UsdGeom.Xformable(prim)
    orient_op = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            orient_op = op
            break
    if orient_op is None:
        orient_op = xform.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
    orient_op.Set(quat)


# =============================================================================
# VELOCITY ARROW
# =============================================================================

def _build_velocity_arrow(stage, path, mat):
    xf = UsdGeom.Xform.Define(stage, path)

    shaft = UsdGeom.Cylinder.Define(stage, f"{path}/Shaft")
    shaft.CreateRadiusAttr(0.6)
    shaft.CreateHeightAttr(1.0)
    shaft.CreateAxisAttr(UsdGeom.Tokens.x)
    _bind(shaft.GetPrim(), mat)
    _translate(shaft.GetPrim(), 0.5, 0, 0)

    head = UsdGeom.Cone.Define(stage, f"{path}/Head")
    head.CreateRadiusAttr(1.8)
    head.CreateHeightAttr(4.0)
    head.CreateAxisAttr(UsdGeom.Tokens.x)
    _bind(head.GetPrim(), mat)
    _translate(head.GetPrim(), 1.0, 0, 0)

    return xf.GetPrim()


def _update_velocity_arrow(arrow_prim, shaft_prim, head_prim, vx, vz, rx, ry, rz):
    speed = math.sqrt(vx * vx + vz * vz)
    if speed < 1e-4:
        UsdGeom.Imageable(arrow_prim).MakeInvisible()
        return

    UsdGeom.Imageable(arrow_prim).MakeVisible()
    shaft_len = speed * VELOCITY_ARROW_SCALE
    UsdGeom.XformCommonAPI(shaft_prim).SetScale(Gf.Vec3f(shaft_len, 1.0, 1.0))
    _translate(shaft_prim, shaft_len / 2.0, 0, 0)
    _translate(head_prim, shaft_len, 0, 0)
    _translate(arrow_prim, rx, ry, rz)
    _rotate_y_deg(arrow_prim, -math.degrees(math.atan2(vz, vx)))


# =============================================================================
# SCENE BUILD
# =============================================================================

def build_robot(stage):
    root = "/World/CylRobot"
    UsdGeom.Xform.Define(stage, root)

    material_root = f"{root}/Materials"
    m_body = _make_material(stage, f"{material_root}/Body", Gf.Vec3f(0.18, 0.45, 0.72), 0.35, 0.5)
    m_wheel = _make_material(stage, f"{material_root}/Wheel", Gf.Vec3f(0.12, 0.12, 0.14), 0.60, 0.3)
    m_rim = _make_material(stage, f"{material_root}/Rim", Gf.Vec3f(0.70, 0.70, 0.72), 0.40, 0.6)
    m_arrow = _make_material(stage, f"{material_root}/Arrow", Gf.Vec3f(0.95, 0.35, 0.10), 0.30, 0.0)
    m_top = _make_material(stage, f"{material_root}/Top", Gf.Vec3f(0.22, 0.55, 0.85), 0.30, 0.5)

    status_materials = {
        "OK": _make_material(stage, f"{material_root}/StatusOK", Gf.Vec3f(0.05, 0.80, 0.35), 0.20, 0.0),
        "STALE": _make_material(stage, f"{material_root}/StatusStale", Gf.Vec3f(0.95, 0.72, 0.10), 0.25, 0.0),
        "MISSING": _make_material(stage, f"{material_root}/StatusMissing", Gf.Vec3f(0.50, 0.52, 0.56), 0.45, 0.0),
        "ABNORMAL": _make_material(stage, f"{material_root}/StatusAbnormal", Gf.Vec3f(0.95, 0.12, 0.10), 0.25, 0.0),
        "ERROR": _make_material(stage, f"{material_root}/StatusError", Gf.Vec3f(0.95, 0.12, 0.10), 0.25, 0.0),
    }

    robot_xf = UsdGeom.Xform.Define(stage, f"{root}/Robot")
    _translate(robot_xf.GetPrim(), 0, 0, 0)

    body = UsdGeom.Cylinder.Define(stage, f"{root}/Robot/Body")
    body.CreateRadiusAttr(BODY_RADIUS)
    body.CreateHeightAttr(BODY_HEIGHT)
    body.CreateAxisAttr(UsdGeom.Tokens.y)
    _bind(body.GetPrim(), m_body)
    _translate(body.GetPrim(), 0, BODY_Y_OFFSET + BODY_HEIGHT / 2.0, 0)

    top_disc = UsdGeom.Cylinder.Define(stage, f"{root}/Robot/TopDisc")
    top_disc.CreateRadiusAttr(BODY_RADIUS * 0.6)
    top_disc.CreateHeightAttr(1.0)
    top_disc.CreateAxisAttr(UsdGeom.Tokens.y)
    _bind(top_disc.GetPrim(), m_top)
    _translate(top_disc.GetPrim(), 0, BODY_Y_OFFSET + BODY_HEIGHT + 0.5, 0)

    pip = UsdGeom.Cylinder.Define(stage, f"{root}/Robot/HeadingPip")
    pip.CreateRadiusAttr(1.5)
    pip.CreateHeightAttr(BODY_RADIUS * 0.5)
    pip.CreateAxisAttr(UsdGeom.Tokens.z)
    _bind(pip.GetPrim(), m_arrow)
    _translate(pip.GetPrim(), 0, BODY_Y_OFFSET + BODY_HEIGHT + 0.5, BODY_RADIUS * 0.85)

    status_light = UsdGeom.Sphere.Define(stage, f"{root}/Robot/StatusLight")
    status_light.CreateRadiusAttr(2.2)
    _bind(status_light.GetPrim(), status_materials["MISSING"])
    _translate(status_light.GetPrim(), 0, BODY_Y_OFFSET + BODY_HEIGHT + 4.0, -BODY_RADIUS * 0.45)

    angles = _wheel_angles_rad()
    for index, angle in enumerate(angles):
        wheel_x = math.cos(angle) * WHEEL_OFFSET_R
        wheel_z = math.sin(angle) * WHEEL_OFFSET_R
        wheel_y = WHEEL_Y_CENTRE + WHEEL_Y_OFFSETS[index]
        wheel_path = f"{root}/Robot/Wheel{index}"

        wheel_xf = UsdGeom.Xform.Define(stage, wheel_path)
        UsdGeom.XformCommonAPI(wheel_xf.GetPrim()).SetTranslate(Gf.Vec3d(wheel_x, wheel_y, wheel_z))
        _set_orient(wheel_xf.GetPrim(), _wheel_orient_quat(angle))

        tyre = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Tyre")
        tyre.CreateRadiusAttr(WHEEL_RADIUS)
        tyre.CreateHeightAttr(WHEEL_WIDTH)
        tyre.CreateAxisAttr(UsdGeom.Tokens.y)
        _bind(tyre.GetPrim(), m_wheel)

        rim = UsdGeom.Cylinder.Define(stage, f"{wheel_path}/Rim")
        rim.CreateRadiusAttr(WHEEL_RADIUS * 0.55)
        rim.CreateHeightAttr(WHEEL_WIDTH + 0.2)
        rim.CreateAxisAttr(UsdGeom.Tokens.y)
        _bind(rim.GetPrim(), m_rim)

    arrow_root = _build_velocity_arrow(stage, f"{root}/VelocityArrow", m_arrow)
    UsdGeom.Imageable(arrow_root).MakeInvisible()

    return {
        "robot_xf": robot_xf.GetPrim(),
        "arrow_root": arrow_root,
        "shaft_prim": stage.GetPrimAtPath(f"{root}/VelocityArrow/Shaft"),
        "head_prim": stage.GetPrimAtPath(f"{root}/VelocityArrow/Head"),
        "status_light": status_light.GetPrim(),
        "status_materials": status_materials,
    }


# =============================================================================
# STATE APPLICATION
# =============================================================================

def _set_status_light(prims, status):
    status_materials = prims.get("status_materials", {})
    material = status_materials.get(status) or status_materials.get("MISSING")
    if material is not None:
        _bind(prims["status_light"], material)


def apply_latest_state(prims, latest_state):
    if not latest_state:
        return False, "no latest state available"

    pose = latest_state.get("pose") or {}
    velocity = latest_state.get("velocity") or {}

    try:
        x_cm = float(pose["x_m"]) * 100.0
        z_cm = float(pose["z_m"]) * 100.0
        heading_rad = float(pose["heading_rad"])
        vx_mps = float(velocity.get("vx_mps", 0.0))
        vz_mps = float(velocity.get("vz_mps", 0.0))
    except (KeyError, TypeError, ValueError) as exc:
        return False, f"invalid latest state: {exc}"

    _translate(prims["robot_xf"], x_cm, 0, z_cm)
    _rotate_y_deg(prims["robot_xf"], math.degrees(heading_rad))
    _update_velocity_arrow(
        prims["arrow_root"],
        prims["shaft_prim"],
        prims["head_prim"],
        vx_mps,
        vz_mps,
        x_cm,
        VELOCITY_ARROW_Y,
        z_cm,
    )
    return True, ""


def _fmt(value, digits=3):
    if value is None:
        return "-"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "-"


# =============================================================================
# OMNIVERSE EXTENSION
# =============================================================================

class MobileBotDigitalTwinExt(omni.ext.IExt):
    def on_startup(self, _ext_id):
        stage = _get_or_create_stage()
        self._prims = build_robot(stage)
        self._poll_elapsed_s = HTTP_POLL_INTERVAL_S
        self._last_snapshot = None
        self._last_status = "MISSING"
        self._last_warning = "waiting for Syncra state service"
        self._last_poll_time = None

        self._build_window()
        self._set_status("MISSING", "waiting for Syncra state service")

        self._sub_tick = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update)
        )
        print("[Syncra] MobileBot extension started")
        print(f"[Syncra] polling {HTTP_STATE_URL}")
        print("[Syncra] safety: visualization reads state only and sends no robot commands")

    def _build_window(self):
        self._window = ui.Window("Syncra MobileBot State", width=380, height=285)
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Syncra one-way digital twin", height=22)
                ui.Label("MQTT -> state service -> HTTP -> Omniverse", height=20)
                ui.Spacer(height=6)
                self._status_label = ui.Label("Status: MISSING", height=22)
                self._connection_label = ui.Label("HTTP: checking", height=20)
                self._sequence_label = ui.Label("Sequence: -", height=20)
                self._pose_label = ui.Label("Pose: -", height=20)
                self._velocity_label = ui.Label("Velocity: -", height=20)
                self._battery_label = ui.Label("Battery: -", height=20)
                self._warning_label = ui.Label("Warning: waiting for data", height=44, word_wrap=True)

    def _fetch_latest_snapshot(self):
        with urllib.request.urlopen(HTTP_STATE_URL, timeout=HTTP_TIMEOUT_S) as response:
            return json.loads(response.read().decode("utf-8"))

    def _on_update(self, event):
        payload = dict(event.payload.get_dict())
        dt = float(payload.get("dt", 0.016))
        self._poll_elapsed_s += dt

        if self._poll_elapsed_s < HTTP_POLL_INTERVAL_S:
            return

        self._poll_elapsed_s = 0.0
        self._last_poll_time = time.time()

        try:
            snapshot = self._fetch_latest_snapshot()
        except (urllib.error.URLError, TimeoutError, OSError, json.JSONDecodeError) as exc:
            self._last_snapshot = None
            self._set_status("MISSING", f"state service unavailable: {exc}")
            self._refresh_ui()
            return

        self._last_snapshot = snapshot
        status = snapshot.get("status") or "MISSING"
        latest = snapshot.get("latest")
        warnings = snapshot.get("warnings") or []

        if latest:
            applied, warning = apply_latest_state(self._prims, latest)
            if not applied:
                status = "ERROR"
                warnings = warnings + [warning]

        warning_text = "; ".join(warnings) if warnings else "none"
        self._set_status(status, warning_text)
        self._refresh_ui()

    def _set_status(self, status, warning):
        self._last_status = status
        self._last_warning = warning
        _set_status_light(self._prims, status)

    def _refresh_ui(self):
        snapshot = self._last_snapshot or {}
        latest = snapshot.get("latest") or {}
        pose = latest.get("pose") or {}
        velocity = latest.get("velocity") or {}

        http_state = "connected" if self._last_snapshot else "unavailable"
        age = snapshot.get("age_s")
        sequence = latest.get("sequence", "-")
        battery = latest.get("battery_pct")

        self._status_label.text = f"Status: {self._last_status}"
        self._connection_label.text = f"HTTP: {http_state} | age: {_fmt(age, 2)} s"
        self._sequence_label.text = f"Sequence: {sequence}"
        self._pose_label.text = (
            f"Pose: x={_fmt(pose.get('x_m'))} m  "
            f"z={_fmt(pose.get('z_m'))} m  "
            f"h={_fmt(pose.get('heading_rad'))} rad"
        )
        self._velocity_label.text = (
            f"Velocity: vx={_fmt(velocity.get('vx_mps'))}  "
            f"vz={_fmt(velocity.get('vz_mps'))}  "
            f"om={_fmt(velocity.get('omega_radps'))}"
        )
        self._battery_label.text = f"Battery: {_fmt(battery, 1)} %"
        self._warning_label.text = f"Warning: {self._last_warning}"

    def on_shutdown(self):
        if getattr(self, "_sub_tick", None) is not None:
            self._sub_tick.unsubscribe()
            self._sub_tick = None
        self._window = None
        print("[Syncra] MobileBot extension shutdown")


# Backward-compatible alias for any generated Kit discovery that used the old class name.
CylRobotExt = MobileBotDigitalTwinExt
