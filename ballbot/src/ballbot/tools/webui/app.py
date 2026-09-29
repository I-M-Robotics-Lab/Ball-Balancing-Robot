import os
import math
from flask import Flask, render_template, request, jsonify
from flask_cors import CORS

from robotKinematics import RobotKinematics
from controller import RobotController

ROOT = os.path.dirname(os.path.abspath(__file__))

app = Flask(
    __name__,
    template_folder=os.path.join(ROOT, "templates"),
    static_folder=os.path.join(ROOT, "static"),
)
CORS(app)

robot = RobotKinematics()

# Hard-coded parameters
robot.lp = 7.125
robot.l1 = 6.2
robot.l2 = 4.5
robot.lb = 4.0

def compute_maxh(l1, l2, lp, lb):
    return math.sqrt(((l1 + l2) ** 2) - ((lp - lb) ** 2))

def compute_minh(l1, l2, lp, lb):
    if l1 > l2:
        return math.sqrt((l1 ** 2) - ((lb + l2 - lp) ** 2))
    if l2 > l1:
        return math.sqrt(((l2 - l1) ** 2) - ((lp - lb) ** 2))
    return 0.0

robot.maxh = compute_maxh(robot.l1, robot.l2, robot.lp, robot.lb) - 0.2
robot.minh = compute_minh(robot.l1, robot.l2, robot.lp, robot.lb) + 0.45

DEFAULT_H = (robot.minh + robot.maxh) / 2.0
DEFAULT_SLIDER_H = int(DEFAULT_H * 100)

HW_ENABLED = os.getenv("BALLBOT_HW", "0") == "1"
rc = RobotController(robot) if HW_ENABLED else None

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/update", methods=["POST"])
def update_robot():
    data = request.get_json(force=True) or {}

    slider_theta = float(data.get("slider_theta", 0))
    slider_phi   = float(data.get("slider_phi", 0))
    slider_h     = float(data.get("slider_h", DEFAULT_SLIDER_H))

    theta_deg = slider_theta / 100.0
    phi_deg   = slider_phi   / 100.0
    h         = slider_h     / 100.0

    alpha, beta, gamma = 0.0, 0.0, 1.0
    max_theta_for_h = 10.0

    ok = True
    err = None

    try:
        theta_rad = math.radians(theta_deg)
        phi_rad   = math.radians(phi_deg)

        alpha = math.sin(theta_rad) * math.cos(phi_rad)
        beta  = math.sin(theta_rad) * math.sin(phi_rad)
        gamma = math.cos(theta_rad)

        max_theta_for_h = robot.max_theta(h)
        robot.solve_inverse_kinematics_vector(alpha, beta, gamma, h)

        if rc is not None:
            rc.set_motor_angles(
                math.degrees(math.pi * 0.5 - robot.theta1),
                math.degrees(math.pi * 0.5 - robot.theta2),
                math.degrees(math.pi * 0.5 - robot.theta3),
            )

    except Exception as e:
        ok = False
        err = str(e)

    return jsonify({
        "ok": ok,
        "err": err,
        "hardware_enabled": HW_ENABLED,

        "alpha": alpha, "beta": beta, "gamma": gamma, "h": h,
        "theta1": getattr(robot, "theta1", 0.0),
        "theta2": getattr(robot, "theta2", 0.0),
        "theta3": getattr(robot, "theta3", 0.0),

        "A_points": [robot.A1, robot.A2, robot.A3],
        "B_points": [robot.B1, robot.B2, robot.B3],
        "C_points": [robot.C1, robot.C2, robot.C3],

        "line_A": [robot.A1, robot.A2, robot.A2, robot.A3, robot.A3, robot.A1],
        "line_B": [robot.B1, robot.B2, robot.B2, robot.B3, robot.B3, robot.B1],
        "line_C1": [robot.A1, robot.C1, robot.C1, robot.B1],
        "line_C2": [robot.A2, robot.C2, robot.C2, robot.B2],
        "line_C3": [robot.A3, robot.C3, robot.C3, robot.B3],

        "lp": robot.lp, "l1": robot.l1, "l2": robot.l2, "lb": robot.lb,
        "minh": robot.minh, "maxh": robot.maxh,
        "max_theta": max_theta_for_h,
    })

if __name__ == "__main__":
    print("starting server...")
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False)
