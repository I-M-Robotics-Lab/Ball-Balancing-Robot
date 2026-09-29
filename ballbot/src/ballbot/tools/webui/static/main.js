/* static/main.js */
/* 3RRS Flask simulator client (Three.js + controls) */

(() => {
  // -----------------------------
  // DOM helpers + elements
  // -----------------------------
  const $ = (id) => document.getElementById(id);

  const leftPanel = $("left-panel");

  const sliderTheta = $("slider_theta");
  const sliderPhi   = $("slider_phi");
  const sliderH     = $("slider_h");

  const spinTheta = $("spin_theta");
  const spinPhi   = $("spin_phi");
  const spinH     = $("spin_h");

  const autoUpdate = $("auto_update");
  const applyBtn   = $("apply_button");
  const resetBtn   = $("reset_button");

  const SCALE = 100; // sliders are scaled by 100 (centi-units)

  function syncSliderToSpin(slider, spin, scale = SCALE) {
    spin.value = (Number(slider.value) / scale).toFixed(2);
  }
  function syncSpinToSlider(spin, slider, scale = SCALE) {
    slider.value = Math.round(Number(spin.value) * scale);
  }

  // Initial sync so the number boxes show something sensible
  syncSliderToSpin(sliderTheta, spinTheta);
  syncSliderToSpin(sliderPhi,   spinPhi);
  syncSliderToSpin(sliderH,     spinH);

  function updateLabels(data) {
    $("label_alpha").textContent  = Number(data.alpha).toFixed(2);
    $("label_beta").textContent   = Number(data.beta).toFixed(2);
    $("label_gamma").textContent  = Number(data.gamma).toFixed(2);
    $("label_h").textContent      = Number(data.h).toFixed(2);

    $("label_theta1").textContent = Number(data.theta1).toFixed(2);
    $("label_theta2").textContent = Number(data.theta2).toFixed(2);
    $("label_theta3").textContent = Number(data.theta3).toFixed(2);

    $("robot_lp").textContent     = Number(data.lp).toFixed(3);
    $("robot_l1").textContent     = Number(data.l1).toFixed(3);
    $("robot_l2").textContent     = Number(data.l2).toFixed(3);
    $("robot_lb").textContent     = Number(data.lb).toFixed(3);
    $("robot_minh").textContent   = Number(data.minh).toFixed(3);
    $("robot_maxh").textContent   = Number(data.maxh).toFixed(3);

    // Constrain H slider/spin if server provides bounds
    if (typeof data.minh === "number" && typeof data.maxh === "number") {
      sliderH.min = Math.round(data.minh * SCALE);
      sliderH.max = Math.round(data.maxh * SCALE);
      spinH.min = data.minh;
      spinH.max = data.maxh;
    }

    // Optional: show error state if server reports it
    if (data.ok === false && data.err) {
      console.warn("Server /update error:", data.err);
    }
  }

  // -----------------------------
  // Three.js setup (optional)
  // -----------------------------
  let threeOk = true;

  let scene, camera, renderer, controls;
  let ptsA, ptsB, ptsC;
  let lineA, lineB, lineC1, lineC2, lineC3;

  function makePoints(size) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(9), 3)); // 3 points
    const mat = new THREE.PointsMaterial({ size });
    return new THREE.Points(geom, mat);
  }

  function makeLine() {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array(18), 3)); // up to 6 points
    const mat = new THREE.LineBasicMaterial();
    return new THREE.LineSegments(geom, mat);
  }

  function setPoints(pointsObj, pts3) {
    if (!Array.isArray(pts3) || pts3.length < 3) return;
    const arr = pointsObj.geometry.attributes.position.array;
    for (let i = 0; i < 3; i++) {
      arr[i * 3 + 0] = pts3[i][0];
      arr[i * 3 + 1] = pts3[i][1];
      arr[i * 3 + 2] = pts3[i][2];
    }
    pointsObj.geometry.attributes.position.needsUpdate = true;
    pointsObj.geometry.computeBoundingSphere();
  }

  function setLine(lineObj, ptsN) {
    if (!Array.isArray(ptsN) || ptsN.length < 2) return;
    const arr = lineObj.geometry.attributes.position.array;
    const n = Math.min(ptsN.length, arr.length / 3);
    for (let i = 0; i < n; i++) {
      arr[i * 3 + 0] = ptsN[i][0];
      arr[i * 3 + 1] = ptsN[i][1];
      arr[i * 3 + 2] = ptsN[i][2];
    }
    lineObj.geometry.attributes.position.needsUpdate = true;
    lineObj.geometry.computeBoundingSphere();
  }

  function initThree() {
    // If CDN failed, THREE will be undefined.
    if (typeof THREE === "undefined") {
      threeOk = false;
      console.error("Three.js not loaded (THREE is undefined). 3D view disabled.");
      return;
    }
    if (!leftPanel) {
      threeOk = false;
      console.error("Missing #left-panel. 3D view disabled.");
      return;
    }

    scene = new THREE.Scene();
    camera = new THREE.PerspectiveCamera(
      60,
      leftPanel.clientWidth / leftPanel.clientHeight,
      0.1,
      1000
    );
    camera.position.set(15, 15, 15);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(leftPanel.clientWidth, leftPanel.clientHeight);
    leftPanel.appendChild(renderer.domElement);

    // OrbitControls from examples attaches to THREE.OrbitControls in your current HTML
    if (typeof THREE.OrbitControls === "function") {
      controls = new THREE.OrbitControls(camera, renderer.domElement);
      controls.target.set(0, 0, 0);
      controls.update();
    } else {
      console.warn("OrbitControls not found. Continuing without orbit controls.");
    }

    // lights + axes
    scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const dir = new THREE.DirectionalLight(0xffffff, 0.8);
    dir.position.set(10, 10, 10);
    scene.add(dir);
    scene.add(new THREE.AxesHelper(5));

    // objects
    ptsA = makePoints(0.2);
    ptsB = makePoints(0.2);
    ptsC = makePoints(0.2);

    lineA = makeLine();
    lineB = makeLine();
    lineC1 = makeLine();
    lineC2 = makeLine();
    lineC3 = makeLine();

    scene.add(ptsA, ptsB, ptsC, lineA, lineB, lineC1, lineC2, lineC3);

    // resize
    window.addEventListener("resize", () => {
      if (!renderer || !camera) return;
      const w = leftPanel.clientWidth;
      const h = leftPanel.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    });

    // render loop
    const animate = () => {
      requestAnimationFrame(animate);
      renderer.render(scene, camera);
    };
    animate();
  }

  initThree();

  function updateGeometry(data) {
    if (!threeOk) return;

    setPoints(ptsA, data.A_points);
    setPoints(ptsB, data.B_points);
    setPoints(ptsC, data.C_points);

    setLine(lineA, data.line_A);
    setLine(lineB, data.line_B);

    setLine(lineC1, data.line_C1);
    setLine(lineC2, data.line_C2);
    setLine(lineC3, data.line_C3);
  }

  // -----------------------------
  // Networking
  // -----------------------------
  let hInitialized = false;
  let inFlight = false; // prevent request pile-ups

  async function postUpdate() {
    if (inFlight) return;
    inFlight = true;

    try {
      const payload = {
        slider_theta: Number(sliderTheta.value),
        slider_phi: Number(sliderPhi.value),
        slider_h: Number(sliderH.value),
      };

      const res = await fetch("/update", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(payload),
      });

      const data = await res.json();

      // First response: use it to learn minh/maxh, then set H to mid-range and re-fetch once.
      if (!hInitialized && typeof data.minh === "number" && typeof data.maxh === "number") {
        const mid = (data.minh + data.maxh) / 2;
        sliderH.value = Math.round(mid * SCALE);
        syncSliderToSpin(sliderH, spinH, SCALE);
        hInitialized = true;

        // Now that sliderH changed, run again so geometry matches the slider.
        inFlight = false;
        return postUpdate();
      }

      updateLabels(data);
      updateGeometry(data);

    } catch (e) {
      console.error("postUpdate failed:", e);
    } finally {
      inFlight = false;
    }
  }

  // -----------------------------
  // UI events
  // -----------------------------
  function maybeUpdate() {
    if (autoUpdate.checked) postUpdate();
  }

  sliderTheta.addEventListener("input", () => {
    syncSliderToSpin(sliderTheta, spinTheta);
    maybeUpdate();
  });
  sliderPhi.addEventListener("input", () => {
    syncSliderToSpin(sliderPhi, spinPhi);
    maybeUpdate();
  });
  sliderH.addEventListener("input", () => {
    syncSliderToSpin(sliderH, spinH);
    maybeUpdate();
  });

  spinTheta.addEventListener("change", () => {
    syncSpinToSlider(spinTheta, sliderTheta);
    maybeUpdate();
  });
  spinPhi.addEventListener("change", () => {
    syncSpinToSlider(spinPhi, sliderPhi);
    maybeUpdate();
  });
  spinH.addEventListener("change", () => {
    syncSpinToSlider(spinH, sliderH);
    maybeUpdate();
  });

  applyBtn.addEventListener("click", () => postUpdate());

  resetBtn.addEventListener("click", () => {
    sliderTheta.value = 0;
    sliderPhi.value = 0;

    // Keep H in-range; use current slider bounds if set
    const minH = Number(sliderH.min || 0);
    const maxH = Number(sliderH.max || 20000);
    sliderH.value = Math.round((minH + maxH) / 2);

    syncSliderToSpin(sliderTheta, spinTheta);
    syncSliderToSpin(sliderPhi, spinPhi);
    syncSliderToSpin(sliderH, spinH);

    postUpdate();
  });

  // Kick off initial update
  postUpdate();
})();
