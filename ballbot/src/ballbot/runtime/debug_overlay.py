import cv2

def draw_legend(img, dbg):
    y = 20
    dy = 18
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.45
    thick = 1

    GREEN = (0, 255, 0)
    GRAY  = (160, 160, 160)
    WHITE = (255, 255, 255)

    def line(text, on=True):
        nonlocal y
        color = GREEN if on else GRAY
        cv2.putText(img, text, (5, y), font, scale, color, thick, cv2.LINE_AA)
        y += dy

    def header(text):
        nonlocal y
        cv2.putText(img, text, (5, y), font, scale, WHITE, thick, cv2.LINE_AA)
        y += dy

    # --- Base keys ---
    header("KEYS:")
    line(" q : quit")
    line(" t : debug master")

    # --- Debug-only keys ---
    if dbg.master:
        header("DEBUG:")
        line(f" c : camera       {'ON' if dbg.cam else 'off'}", dbg.cam)
        line(f" m : mask         {'ON' if dbg.cam_mask else 'off'}", dbg.cam_mask)
        line(f" r : controller   {'ON' if dbg.ctrl else 'off'}", dbg.ctrl)
        line(f" p : PID          {'ON' if dbg.pid else 'off'}", dbg.pid)
        line(" h : help")
