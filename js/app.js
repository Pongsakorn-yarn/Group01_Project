(function () {
  'use strict';

  const cadData = window.ARM_CAD_DATA;
  if (!cadData) {
    alert('ไม่พบข้อมูลโมเดล CAD (js/arm_cad_data.js)');
    return;
  }

  // --- Helper: Decode Base64 to Float32Array ---
  function b64ToFloat32(b64) {
    const bin = atob(b64);
    const len = bin.length;
    const bytes = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
      bytes[i] = bin.charCodeAt(i);
    }
    return new Float32Array(bytes.buffer);
  }

  // --- State ---
  const state = {
    // Joint offsets from CAD Home Pose:
    // j1: Base Yaw deg (-90..+90)
    // j2: Shoulder Pitch deg (-45..+65) (+ = forward/down)
    // j3: Elbow Pitch deg (-55..+55) (+ = forearm down)
    // j4: Gripper Open % (0 = closed, 100 = wide open; 35% = CAD home)
    // wristTrim: Wrist pitch trim deg (-45..+45)
    target: { j1: 0, j2: 0, j3: 0, j4: 35, wrist: 0 },
    current: { j1: 0, j2: 0, j3: 0, j4: 35, wrist: 0 },
    smoothing: 0.22,
    explodeFactor: 0,
    showEdges: true,
    showPivots: true,
    xrayMode: false,
    handMode: 'ik', // 'ik' | 'direct'
    mirrorCam: true,
    camRunning: false,
    demoMode: null, // 'pick' | 'circle' | null
    demoTime: 0,
    selectedBodyId: null,
    // Recording
    recording: false,
    playing: false,
    recordedFrames: [],
    playIndex: 0,
    // IoT WebSerial
    serialPort: null,
    serialWriter: null,
    lastSerialSend: 0
  };

  // --- Three.js Scene Setup ---
  const container = document.getElementById('canvas-container');
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x070b14);
  scene.fog = new THREE.FogExp2(0x070b14, 0.0009);

  const camera = new THREE.PerspectiveCamera(
    42,
    window.innerWidth / window.innerHeight,
    1,
    3000
  );
  camera.position.set(240, 210, 260);

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputEncoding = THREE.sRGBEncoding;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.15;
  container.appendChild(renderer.domElement);

  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(25, 85, 0);
  controls.enableDamping = true;
  controls.dampingFactor = 0.06;
  controls.maxPolarAngle = Math.PI / 2 + 0.08;
  controls.minDistance = 80;
  controls.maxDistance = 900;

  // --- Lighting ---
  const hemiLight = new THREE.HemisphereLight(0xdbeafe, 0x0f172a, 0.85);
  scene.add(hemiLight);

  const dirLight = new THREE.DirectionalLight(0xffffff, 1.15);
  dirLight.position.set(180, 340, 220);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.width = 2048;
  dirLight.shadow.mapSize.height = 2048;
  dirLight.shadow.camera.near = 50;
  dirLight.shadow.camera.far = 800;
  const d = 220;
  dirLight.shadow.camera.left = -d;
  dirLight.shadow.camera.right = d;
  dirLight.shadow.camera.top = d;
  dirLight.shadow.camera.bottom = -d;
  dirLight.shadow.bias = -0.0005;
  scene.add(dirLight);

  const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.55);
  rimLight.position.set(-220, 140, -180);
  scene.add(rimLight);

  // --- Worktable Grid & Ground ---
  const gridHelper = new THREE.GridHelper(500, 25, 0x0ea5e9, 0x1e293b);
  gridHelper.position.y = -0.2;
  scene.add(gridHelper);

  const groundGeo = new THREE.PlaneGeometry(1200, 1200);
  const groundMat = new THREE.MeshStandardMaterial({
    color: 0x090d18,
    roughness: 0.85,
    metalness: 0.1
  });
  const groundMesh = new THREE.Mesh(groundGeo, groundMat);
  groundMesh.rotation.x = -Math.PI / 2;
  groundMesh.position.y = -0.5;
  groundMesh.receiveShadow = true;
  scene.add(groundMesh);

  // --- CAD Root Coordinate Frame (CAD Z-up -> Three.js Y-up) ---
  // In CAD: X = forward, Y = left, Z = up.
  // Rotating cadRoot by -90 deg around X makes CAD +Z point along Three.js +Y,
  // CAD +X point along Three.js +X, and CAD +Y point along Three.js -Z.
  const cadRoot = new THREE.Group();
  cadRoot.rotation.x = -Math.PI / 2;
  scene.add(cadRoot);

  // --- Pivot Helper Factory: Creates a Group at P_cur that rotates around P_0 in CAD space ---
  function createPivotNode(p0) {
    const pivot = new THREE.Group();
    pivot.position.set(p0[0], p0[1], p0[2]);
    const offset = new THREE.Group();
    offset.position.set(-p0[0], -p0[1], -p0[2]);
    pivot.add(offset);
    return { pivot, offset, p0: [...p0] };
  }

  // Exact CAD Home Pose Pivot Coordinates (verified from Robotic_Arm_4DOF_Assembly.sc)
  const PIVOTS = {
    yaw: [-28.0, 0.0, 29.0],
    shoulder: [-28.0, 0.0, 64.0],
    upperParRear: [-44.0, -26.5, 72.0],
    elbow: [5.809, 0.0, 136.505],
    bellcrankTop: [5.809, -20.5, 159.505],
    bellcrankBack: [-16.191, -26.5, 126.505],
    elbowServoHorn: [-33.831, -26.5, 64.984],
    wrist: [85.371, 0.0, 128.142],
    clawLeft: [113.371, 12.0, 127.642],
    clawRight: [113.371, -12.0, 127.642],
    driveLinkRight: [111.371, -20.0, 134.642]
  };

  // Build Kinematic Tree inside cadRoot
  const baseStaticGroup = new THREE.Group();
  cadRoot.add(baseStaticGroup);

  const turntableNode = createPivotNode(PIVOTS.yaw);
  cadRoot.add(turntableNode.pivot);
  // turntableNode.offset is in CAD Home Pose coordinates, rotating with Base Yaw!
  const turntableWorld = turntableNode.offset;

  const upperMainNode = createPivotNode(PIVOTS.shoulder);
  turntableWorld.add(upperMainNode.pivot);

  const upperParNode = createPivotNode(PIVOTS.upperParRear);
  turntableWorld.add(upperParNode.pivot);

  const bellcrankNode = createPivotNode(PIVOTS.elbow);
  turntableWorld.add(bellcrankNode.pivot);

  const pushRodNode = createPivotNode(PIVOTS.bellcrankBack);
  turntableWorld.add(pushRodNode.pivot);

  const forearmMainNode = createPivotNode(PIVOTS.elbow);
  turntableWorld.add(forearmMainNode.pivot);

  const forearmParNode = createPivotNode(PIVOTS.bellcrankTop);
  turntableWorld.add(forearmParNode.pivot);

  const gripperMountNode = createPivotNode(PIVOTS.wrist);
  turntableWorld.add(gripperMountNode.pivot);

  // Claws & Drive Link are children of gripperMountNode.offset (which is in CAD Home coords)
  const clawLeftNode = createPivotNode(PIVOTS.clawLeft);
  gripperMountNode.offset.add(clawLeftNode.pivot);

  const clawRightNode = createPivotNode(PIVOTS.clawRight);
  gripperMountNode.offset.add(clawRightNode.pivot);

  const driveLinkNode = createPivotNode(PIVOTS.driveLinkRight);
  gripperMountNode.offset.add(driveLinkNode.pivot);

  const groupMap = {
    base_static: baseStaticGroup,
    turntable_yaw: turntableWorld,
    upper_main_arm: upperMainNode.offset,
    upper_parallel_link: upperParNode.offset,
    bellcrank: bellcrankNode.offset,
    elbow_push_rod: pushRodNode.offset,
    forearm_main: forearmMainNode.offset,
    forearm_parallel_link: forearmParNode.offset,
    gripper_mount: gripperMountNode.offset,
    gripper_claw_left: clawLeftNode.offset,
    gripper_claw_right: clawRightNode.offset,
    gripper_drive_link: driveLinkNode.offset
  };

  // --- Visual Joint Pivot Markers & TCP Indicator ---
  const pivotMarkersGroup = new THREE.Group();
  cadRoot.add(pivotMarkersGroup);

  function makePivotIndicator(radius = 3.2, color = 0x10b981) {
    const g = new THREE.Group();
    const ringGeo = new THREE.TorusGeometry(radius, 0.55, 12, 28);
    const ringMat = new THREE.MeshBasicMaterial({ color });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    g.add(ring);
    const axGeo = new THREE.CylinderGeometry(0.45, 0.45, radius * 4.5, 10);
    const axMat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.7 });
    const ax = new THREE.Mesh(axGeo, axMat);
    g.add(ax);
    return g;
  }

  const markerYaw = makePivotIndicator(9, 0xf59e0b);
  markerYaw.position.set(...PIVOTS.yaw);
  markerYaw.children[1].rotation.x = Math.PI / 2;
  turntableWorld.add(markerYaw);

  const markerShoulder = makePivotIndicator(5.5, 0x10b981);
  markerShoulder.position.set(...PIVOTS.shoulder);
  markerShoulder.children[0].rotation.x = Math.PI / 2;
  turntableWorld.add(markerShoulder);

  const markerElbow = makePivotIndicator(5.0, 0x38bdf8);
  markerElbow.children[0].rotation.x = Math.PI / 2;
  bellcrankNode.pivot.add(markerElbow);

  const markerWrist = makePivotIndicator(4.5, 0xf43f5e);
  markerWrist.children[0].rotation.x = Math.PI / 2;
  gripperMountNode.pivot.add(markerWrist);

  const pivotMarkers = [markerYaw, markerShoulder, markerElbow, markerWrist];

  // Tool Center Point (TCP) glowing sphere between gripper jaws
  const tcpLocalPos = new THREE.Vector3(162.0, 0.0, 129.0);
  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(2.8, 16, 16),
    new THREE.MeshBasicMaterial({ color: 0x10b981 })
  );
  tcpMarker.position.copy(tcpLocalPos);
  gripperMountNode.offset.add(tcpMarker);

  // TCP Motion Trail in Scene Space
  const MAX_TRAIL = 120;
  const trailPositions = new Float32Array(MAX_TRAIL * 3);
  const trailGeo = new THREE.BufferGeometry();
  trailGeo.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
  const trailLine = new THREE.Line(
    trailGeo,
    new THREE.LineBasicMaterial({ color: 0x38bdf8, transparent: true, opacity: 0.55 })
  );
  scene.add(trailLine);
  let trailCount = 0;

  // --- Interactive Pick-and-Place Target Cube (in CAD space) ---
  const targetCubeGeo = new THREE.BoxGeometry(18, 18, 18);
  const targetCubeMat = new THREE.MeshStandardMaterial({
    color: 0xf59e0b,
    roughness: 0.25,
    metalness: 0.2,
    emissive: 0x78350f,
    emissiveIntensity: 0.35
  });
  const targetCube = new THREE.Mesh(targetCubeGeo, targetCubeMat);
  targetCube.castShadow = true;
  targetCube.receiveShadow = true;
  const CUBE_HOME = new THREE.Vector3(135.0, 45.0, 9.0);
  targetCube.position.copy(CUBE_HOME);
  cadRoot.add(targetCube);
  let cubeGrabbed = false;

  // --- Build All 39 CAD Body Meshes ---
  const bodyMeshes = [];
  const edgeLines = [];
  const partsListEl = document.getElementById('parts-list');

  cadData.bodies.forEach((b) => {
    const posArr = b64ToFloat32(b.pos_b64);
    const normArr = b64ToFloat32(b.norm_b64);
    const edgeArr = b64ToFloat32(b.edge_b64);

    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
    geom.setAttribute('normal', new THREE.BufferAttribute(normArr, 3));

    // Determine realistic material finish based on part name/color
    let [r, g, bl] = b.color;
    let metalness = 0.2;
    let roughness = 0.32;
    if (b.name.startsWith('Pin_') || b.name.startsWith('Standoff_')) {
      r = 212; g = 218; bl = 228;
      metalness = 0.82;
      roughness = 0.22;
    } else if (b.name.startsWith('Servo_')) {
      r = 38; g = 44; bl = 58;
      metalness = 0.35;
      roughness = 0.55;
    }

    const colorHex = new THREE.Color(r / 255, g / 255, bl / 255);
    const mat = new THREE.MeshStandardMaterial({
      color: colorHex,
      metalness,
      roughness,
      side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geom, mat);
    mesh.name = b.name;
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // Compute body centroid for Exploded View
    const center = new THREE.Vector3(
      0.5 * (b.bbox_min[0] + b.bbox_max[0]),
      0.5 * (b.bbox_min[1] + b.bbox_max[1]),
      0.5 * (b.bbox_min[2] + b.bbox_max[2])
    );
    mesh.userData = {
      bodyId: b.id,
      name: b.name,
      group: b.group,
      triCount: b.tri_count,
      baseColor: colorHex.clone(),
      center
    };

    // Feature outline edges
    const edgeGeo = new THREE.BufferGeometry();
    edgeGeo.setAttribute('position', new THREE.BufferAttribute(edgeArr, 3));
    const edgeMat = new THREE.LineBasicMaterial({
      color: 0xbae6fd,
      transparent: true,
      opacity: 0.28
    });
    const lines = new THREE.LineSegments(edgeGeo, edgeMat);
    mesh.add(lines);
    edgeLines.push(lines);

    const parentGroup = groupMap[b.group] || baseStaticGroup;
    parentGroup.add(mesh);
    bodyMeshes.push(mesh);

    // Populate CAD Parts Inspector list (using safe DOM APIs)
    const item = document.createElement('div');
    item.className = 'part-item';
    item.dataset.bodyId = b.id;

    const leftWrap = document.createElement('span');
    leftWrap.style.cssText = 'display:flex;align-items:center;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;';

    const swatch = document.createElement('span');
    swatch.className = 'part-swatch';
    swatch.style.background = `rgb(${r},${g},${bl})`;

    const nameSpan = document.createElement('span');
    nameSpan.textContent = `${String(b.id).padStart(2, '0')}. ${b.name}`;

    leftWrap.appendChild(swatch);
    leftWrap.appendChild(nameSpan);

    const triSpan = document.createElement('span');
    triSpan.style.cssText = 'color:#64748b;font-family:var(--font-mono);font-size:10px;';
    triSpan.textContent = `${b.tri_count}t`;

    item.appendChild(leftWrap);
    item.appendChild(triSpan);
    item.addEventListener('click', () => selectPart(b.id));
    partsListEl.appendChild(item);
  });

  function selectPart(bodyId) {
    state.selectedBodyId = state.selectedBodyId === bodyId ? null : bodyId;
    const badge = document.getElementById('selected-part-badge');
    document.querySelectorAll('.part-item').forEach((el) => {
      el.classList.toggle('selected', Number(el.dataset.bodyId) === state.selectedBodyId);
    });
    bodyMeshes.forEach((m) => {
      const isSel = m.userData.bodyId === state.selectedBodyId;
      m.material.emissive.setHex(isSel ? 0x0ea5e9 : 0x000000);
      m.material.emissiveIntensity = isSel ? 0.55 : 0.0;
    });
    if (state.selectedBodyId !== null) {
      const b = cadData.bodies[state.selectedBodyId];
      badge.textContent = `#${b.id} ${b.name} (${b.group})`;
    } else {
      badge.textContent = 'All 39 Parts';
    }
  }

  // Raycasting on 3D Viewport click
  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let downPos = { x: 0, y: 0 };
  renderer.domElement.addEventListener('pointerdown', (e) => {
    downPos = { x: e.clientX, y: e.clientY };
  });
  renderer.domElement.addEventListener('pointerup', (e) => {
    if (Math.hypot(e.clientX - downPos.x, e.clientY - downPos.y) > 5) return;
    pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
    pointer.y = -(e.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    const hits = raycaster.intersectObjects(bodyMeshes, false);
    if (hits.length > 0) {
      const hitId = hits[0].object.userData.bodyId;
      selectPart(hitId);
      const itemEl = partsListEl.querySelector(`[data-body-id="${hitId}"]`);
      if (itemEl) itemEl.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
    } else if (state.selectedBodyId !== null) {
      selectPart(state.selectedBodyId);
    }
  });

  // --- Exact 4-Bar Parallel Linkage Forward Kinematics Update ---
  const V1_HOME = {
    x: PIVOTS.elbow[0] - PIVOTS.shoulder[0], // 33.809 mm
    z: PIVOTS.elbow[2] - PIVOTS.shoulder[2]  // 72.505 mm
  };
  const V2_HOME = {
    x: PIVOTS.wrist[0] - PIVOTS.elbow[0],    // 79.562 mm
    z: PIVOTS.wrist[2] - PIVOTS.elbow[2]     // -8.363 mm
  };
  const PUSHROD_HOME_ANGLE = Math.atan2(
    PIVOTS.bellcrankBack[2] - PIVOTS.elbowServoHorn[2],
    PIVOTS.bellcrankBack[0] - PIVOTS.elbowServoHorn[0]
  );

  function rotateY2D(vx, vz, angleRad) {
    // In right-handed (X, Y, Z), rotation around +Y by angleRad:
    // x' = x*cos(a) + z*sin(a)
    // z' = -x*sin(a) + z*cos(a)
    const c = Math.cos(angleRad);
    const s = Math.sin(angleRad);
    return {
      x: vx * c + vz * s,
      z: -vx * s + vz * c
    };
  }

  function updateArmKinematics() {
    const deg2rad = Math.PI / 180;
    const th1 = state.current.j1 * deg2rad;
    const th2 = state.current.j2 * deg2rad;
    const th3 = state.current.j3 * deg2rad;
    const wristTrim = state.current.wrist * deg2rad;
    // Map j4 (0..100%) to claw rotation rad (-0.06 rad closed .. +0.58 rad open; 35% -> ~0.16 rad)
    const clawAngle = -0.05 + (state.current.j4 / 100) * 0.62;

    // 1. Base Yaw around Z at [-28, 0, 29]
    turntableNode.pivot.rotation.z = th1;

    // 2. Upper Main Arm & Upper Parallel Link around Y by th2
    upperMainNode.pivot.rotation.y = th2;
    upperParNode.pivot.rotation.y = th2;

    // 3. New Elbow position in Turntable coordinates
    const v1 = rotateY2D(V1_HOME.x, V1_HOME.z, th2);
    const elbowCur = {
      x: PIVOTS.shoulder[0] + v1.x,
      y: 0.0,
      z: PIVOTS.shoulder[2] + v1.z
    };

    // 4. Bellcrank translates with Elbow and stays upright (0 pitch)
    bellcrankNode.pivot.position.set(elbowCur.x, elbowCur.y, elbowCur.z);
    bellcrankNode.pivot.rotation.y = 0;

    // 5. Push Rod top hole moves with Bellcrank back hole (elbowCur + [-22, -26.5, -10])
    const bcBackCur = {
      x: elbowCur.x - 22.0,
      y: -26.5,
      z: elbowCur.z - 10.0
    };
    pushRodNode.pivot.position.set(bcBackCur.x, bcBackCur.y, bcBackCur.z);
    const curPrAngle = Math.atan2(
      bcBackCur.z - PIVOTS.elbowServoHorn[2],
      bcBackCur.x - PIVOTS.elbowServoHorn[0]
    );
    pushRodNode.pivot.rotation.y = PUSHROD_HOME_ANGLE - curPrAngle;

    // 6. Forearm Main Link translates with Elbow and rotates around Y by th3
    forearmMainNode.pivot.position.set(elbowCur.x, elbowCur.y, elbowCur.z);
    forearmMainNode.pivot.rotation.y = th3;

    // 7. Forearm Parallel Link translates with Bellcrank Top (elbowCur + [0, -20.5, 23]) and rotates by th3
    forearmParNode.pivot.position.set(elbowCur.x, -20.5, elbowCur.z + 23.0);
    forearmParNode.pivot.rotation.y = th3;

    // 8. New Wrist position in Turntable coordinates
    const v2 = rotateY2D(V2_HOME.x, V2_HOME.z, th3);
    const wristCur = {
      x: elbowCur.x + v2.x,
      y: 0.0,
      z: elbowCur.z + v2.z
    };

    // 9. Gripper Mount translates with Wrist and stays level (0 deg) + optional wristTrim
    gripperMountNode.pivot.position.set(wristCur.x, wristCur.y, wristCur.z);
    gripperMountNode.pivot.rotation.y = wristTrim;

    // 10. Geared Gripper Claws & Drive Link
    clawLeftNode.pivot.rotation.z = clawAngle;
    clawRightNode.pivot.rotation.z = -clawAngle;
    driveLinkNode.pivot.rotation.z = -clawAngle * 0.55;

    // Update TCP world coordinates in CAD frame (mm)
    cadRoot.updateMatrixWorld(true);
    const tcpWorldThree = new THREE.Vector3();
    tcpMarker.getWorldPosition(tcpWorldThree);
    // Convert Three.js world back to CAD (X, Y, Z)
    const tcpCad = tcpWorldThree.clone().applyMatrix4(
      new THREE.Matrix4().copy(cadRoot.matrixWorld).invert()
    );

    document.getElementById('tcp-x').textContent = tcpCad.x.toFixed(1);
    document.getElementById('tcp-y').textContent = tcpCad.y.toFixed(1);
    document.getElementById('tcp-z').textContent = tcpCad.z.toFixed(1);

    // Update TCP motion trail
    if (trailCount < MAX_TRAIL) {
      trailPositions[trailCount * 3] = tcpWorldThree.x;
      trailPositions[trailCount * 3 + 1] = tcpWorldThree.y;
      trailPositions[trailCount * 3 + 2] = tcpWorldThree.z;
      trailCount++;
    } else {
      trailPositions.copyWithin(0, 3);
      trailPositions[(MAX_TRAIL - 1) * 3] = tcpWorldThree.x;
      trailPositions[(MAX_TRAIL - 1) * 3 + 1] = tcpWorldThree.y;
      trailPositions[(MAX_TRAIL - 1) * 3 + 2] = tcpWorldThree.z;
    }
    trailGeo.setDrawRange(0, trailCount);
    trailGeo.attributes.position.needsUpdate = true;

    // Interactive Target Cube Grab / Release Physics
    const distToCube = tcpCad.distanceTo(targetCube.position);
    if (!cubeGrabbed && distToCube < 26.0 && state.current.j4 < 28) {
      cubeGrabbed = true;
      targetCubeMat.emissive.setHex(0x065f46);
    } else if (cubeGrabbed && state.current.j4 > 34) {
      cubeGrabbed = false;
      targetCubeMat.emissive.setHex(0x78350f);
      // Drop onto table surface (z = 9 mm for 18 mm cube)
      targetCube.position.z = 9.0;
      targetCube.rotation.set(0, 0, 0);
    }

    if (cubeGrabbed) {
      targetCube.position.copy(tcpCad);
      targetCube.rotation.z = th1;
    }
  }

  // --- Analytical 3D Inverse Kinematics Solver (for Hand Tracking & IK Demos) ---
  // Solves for (j1, j2, j3) given desired TCP target (tx, ty, tz) in CAD mm
  const L1 = 80.0;
  const L2 = 80.0;
  const HOME_ALPHA1 = Math.atan2(V1_HOME.x, V1_HOME.z); // 25.0 deg from +Z
  const HOME_ALPHA2 = Math.atan2(-V2_HOME.z, V2_HOME.x); // +6.0 deg downward from +X

  function solveIK(tx, ty, tz) {
    // 1. Base Yaw around [-28, 0]
    const dxYaw = tx - PIVOTS.yaw[0];
    const dyYaw = ty - PIVOTS.yaw[1];
    const j1Rad = Math.atan2(dyYaw, dxYaw);
    const j1Deg = THREE.MathUtils.clamp((j1Rad * 180) / Math.PI, -90, 90);

    // 2. Radial reach from yaw/shoulder axis (-28, 0)
    const radialTotal = Math.hypot(dxYaw, dyYaw);
    // Subtract Gripper TCP offset from Wrist (~76.6 mm forward)
    const wristRadial = Math.max(25.0, radialTotal - 76.6);
    const wristZ = Math.max(15.0, tz);

    const dr = wristRadial;
    const dz = wristZ - PIVOTS.shoulder[2]; // relative to shoulder_z = 64 mm
    let dist = Math.hypot(dr, dz);
    const maxReach = L1 + L2 - 2.0; // 158 mm
    const minReach = 28.0;
    if (dist > maxReach) dist = maxReach;
    if (dist < minReach) dist = minReach;

    // Angle from +Z vertical to target vector (dr, dz)
    const phi = Math.atan2(dr, dz);
    // Law of cosines for elbow internal angle
    const cosA = THREE.MathUtils.clamp((L1 * L1 + dist * dist - L2 * L2) / (2 * L1 * dist), -1, 1);
    const innerA = Math.acos(cosA);

    // Elbow-up configuration (matches parallel linkage geometry)
    const alpha1 = phi - innerA; // angle of Upper Arm from +Z vertical

    // Forearm vector from Elbow to Wrist
    const elR = L1 * Math.sin(alpha1);
    const elZ = L1 * Math.cos(alpha1);
    const fwR = (dr * (dist / Math.max(1e-5, Math.hypot(dr, dz)))) - elR;
    const fwZ = (dz * (dist / Math.max(1e-5, Math.hypot(dr, dz)))) - elZ;
    // Angle of Forearm downward from +X horizontal
    const alpha2 = Math.atan2(-fwZ, fwR);

    const j2Deg = THREE.MathUtils.clamp(((alpha1 - HOME_ALPHA1) * 180) / Math.PI, -45, 65);
    const j3Deg = THREE.MathUtils.clamp(((alpha2 - HOME_ALPHA2) * 180) / Math.PI, -55, 55);

    return { j1: j1Deg, j2: j2Deg, j3: j3Deg };
  }

  // --- UI Sliders & Controls Binding ---
  const sliderJ1 = document.getElementById('slider-j1');
  const sliderJ2 = document.getElementById('slider-j2');
  const sliderJ3 = document.getElementById('slider-j3');
  const sliderJ4 = document.getElementById('slider-j4');
  const sliderWrist = document.getElementById('slider-wrist');

  function syncSlidersFromState() {
    sliderJ1.value = state.target.j1;
    sliderJ2.value = state.target.j2;
    sliderJ3.value = state.target.j3;
    sliderJ4.value = state.target.j4;
    sliderWrist.value = state.target.wrist;
    updateJointLabels();
  }

  function updateJointLabels() {
    const pwm1 = Math.round(90 + state.current.j1);
    const pwm2 = Math.round(90 + state.current.j2);
    const pwm3 = Math.round(90 + state.current.j3);
    document.getElementById('val-j1').textContent = `${state.current.j1.toFixed(1)}° (PWM ${pwm1}°)`;
    document.getElementById('val-j2').textContent = `${state.current.j2.toFixed(1)}° (PWM ${pwm2}°)`;
    document.getElementById('val-j3').textContent = `${state.current.j3.toFixed(1)}° (PWM ${pwm3}°)`;
    document.getElementById('val-j4').textContent = `${Math.round(state.current.j4)}% Open`;
    document.getElementById('val-wrist').textContent =
      Math.abs(state.current.wrist) < 0.3
        ? '0.0° (4-Bar Level)'
        : `${state.current.wrist.toFixed(1)}°`;
  }

  [
    [sliderJ1, 'j1'],
    [sliderJ2, 'j2'],
    [sliderJ3, 'j3'],
    [sliderJ4, 'j4'],
    [sliderWrist, 'wrist']
  ].forEach(([el, key]) => {
    el.addEventListener('input', () => {
      state.demoMode = null;
      state.target[key] = parseFloat(el.value);
    });
  });

  document.getElementById('btn-reset-home').addEventListener('click', () => {
    state.demoMode = null;
    state.target = { j1: 0, j2: 0, j3: 0, j4: 35, wrist: 0 };
    syncSlidersFromState();
  });

  // Smoothing slider
  const sliderSmoothing = document.getElementById('slider-smoothing');
  sliderSmoothing.addEventListener('input', () => {
    state.smoothing = parseFloat(sliderSmoothing.value);
    document.getElementById('val-smoothing').textContent = state.smoothing.toFixed(2);
  });

  // Exploded View slider
  const sliderExplode = document.getElementById('slider-explode');
  sliderExplode.addEventListener('input', () => {
    const pct = parseFloat(sliderExplode.value);
    document.getElementById('val-explode').textContent = `${pct}%`;
    const factor = (pct / 100) * 0.65;
    bodyMeshes.forEach((m) => {
      const c = m.userData.center;
      m.position.set(c.x * factor * 0.25, c.y * factor * 1.4, (c.z - 60) * factor * 0.35);
    });
  });

  // Visual toggles
  const btnEdges = document.getElementById('btn-toggle-edges');
  btnEdges.addEventListener('click', () => {
    state.showEdges = !state.showEdges;
    btnEdges.classList.toggle('active', state.showEdges);
    edgeLines.forEach((l) => (l.visible = state.showEdges));
  });

  const btnPivots = document.getElementById('btn-toggle-pivots');
  btnPivots.addEventListener('click', () => {
    state.showPivots = !state.showPivots;
    btnPivots.classList.toggle('active', state.showPivots);
    pivotMarkers.forEach((p) => (p.visible = state.showPivots));
  });

  const btnXray = document.getElementById('btn-toggle-xray');
  btnXray.addEventListener('click', () => {
    state.xrayMode = !state.xrayMode;
    btnXray.classList.toggle('active', state.xrayMode);
    bodyMeshes.forEach((m) => {
      m.material.transparent = state.xrayMode;
      m.material.opacity = state.xrayMode ? 0.42 : 1.0;
    });
  });

  // Hand Mode & Mirror
  const selectHandMode = document.getElementById('select-hand-mode');
  selectHandMode.addEventListener('change', () => {
    state.handMode = selectHandMode.value;
    document.getElementById('hand-mode-tag').textContent =
      state.handMode === 'ik' ? 'MODE: IK 3D' : 'MODE: DIRECT 4-DOF';
  });

  const btnMirror = document.getElementById('btn-mirror-cam');
  btnMirror.addEventListener('click', () => {
    state.mirrorCam = !state.mirrorCam;
    btnMirror.classList.toggle('active', state.mirrorCam);
    btnMirror.textContent = state.mirrorCam ? '🪞 Mirror: ON' : '🪞 Mirror: OFF';
  });

  // --- Demo Routines ---
  document.getElementById('btn-demo-pick').addEventListener('click', () => {
    state.demoMode = state.demoMode === 'pick' ? null : 'pick';
    state.demoTime = 0;
  });

  document.getElementById('btn-demo-circle').addEventListener('click', () => {
    state.demoMode = state.demoMode === 'circle' ? null : 'circle';
    state.demoTime = 0;
  });

  document.getElementById('btn-reset-cube').addEventListener('click', () => {
    cubeGrabbed = false;
    targetCube.position.copy(CUBE_HOME);
    targetCube.rotation.set(0, 0, 0);
  });

  // --- Pose Recorder & Export ---
  const btnRecord = document.getElementById('btn-record-pose');
  const btnPlay = document.getElementById('btn-play-pose');
  btnRecord.addEventListener('click', () => {
    state.recording = !state.recording;
    if (state.recording) {
      state.recordedFrames = [];
      btnRecord.classList.add('danger');
      btnRecord.textContent = '⏹️ หยุดบันทึก (Stop)';
    } else {
      btnRecord.classList.remove('danger');
      btnRecord.textContent = `⏺️ บันทึก (${state.recordedFrames.length}f)`;
    }
  });

  btnPlay.addEventListener('click', () => {
    if (state.recordedFrames.length === 0) {
      alert('ยังไม่มีข้อมูลท่าทางที่บันทึกไว้ กดปุ่ม Record แล้วขยับแขนกลก่อนครับ');
      return;
    }
    state.playing = !state.playing;
    state.playIndex = 0;
    btnPlay.classList.toggle('active', state.playing);
  });

  document.getElementById('btn-export-traj').addEventListener('click', () => {
    const exportData = {
      model: 'Robotic_Arm_4DOF_Assembly.sc',
      timestamp: new Date().toISOString(),
      current_pose: state.current,
      recorded_frames: state.recordedFrames
    };
    const blob = new Blob([JSON.stringify(exportData, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'robotic_arm_4dof_trajectory.json';
    a.click();
    URL.revokeObjectURL(url);
  });

  // --- WebSerial IoT Bridge (Arduino / ESP32) ---
  const btnSerial = document.getElementById('btn-connect-serial');
  btnSerial.addEventListener('click', async () => {
    if (!('serial' in navigator)) {
      alert('เบราว์เซอร์นี้ไม่รองรับ WebSerial API (แนะนำให้เปิดด้วย Chrome หรือ Edge บนโน้ตบุ๊ก)');
      return;
    }
    try {
      if (state.serialPort) {
        if (state.serialWriter) state.serialWriter.releaseLock();
        await state.serialPort.close();
        state.serialPort = null;
        state.serialWriter = null;
        document.getElementById('serial-dot').className = 'dot';
        document.getElementById('serial-status-text').textContent = 'IoT Serial: Disconnected';
        btnSerial.textContent = '🔌 เชื่อมต่อ Arduino / ESP32 (WebSerial)';
        return;
      }
      const port = await navigator.serial.requestPort();
      await port.open({ baudRate: 115200 });
      state.serialPort = port;
      state.serialWriter = port.writable.getWriter();
      document.getElementById('serial-dot').className = 'dot active';
      document.getElementById('serial-status-text').textContent = 'IoT Serial: 115200 Connected';
      btnSerial.textContent = '🔌 ตัดการเชื่อมต่อ Serial';
    } catch (err) {
      console.warn('WebSerial connection cancelled or failed:', err);
    }
  });

  const serialEncoder = new TextEncoder();

  async function sendSerialTelemetry(now) {
    if (!state.serialWriter || now - state.lastSerialSend < 40) return;
    state.lastSerialSend = now;
    const s1 = Math.round(THREE.MathUtils.clamp(90 + state.current.j1, 0, 180));
    const s2 = Math.round(THREE.MathUtils.clamp(90 + state.current.j2, 0, 180));
    const s3 = Math.round(THREE.MathUtils.clamp(90 + state.current.j3, 0, 180));
    const s4 = Math.round(THREE.MathUtils.clamp((state.current.j4 / 100) * 180, 0, 180));
    const msg = `${s1},${s2},${s3},${s4}\n`;
    try {
      await state.serialWriter.write(serialEncoder.encode(msg));
    } catch (e) {
      console.warn('Serial write error, disconnecting:', e);
      try {
        state.serialWriter.releaseLock();
      } catch (_) { }
      try {
        if (state.serialPort) await state.serialPort.close();
      } catch (_) { }
      state.serialWriter = null;
      state.serialPort = null;
      document.getElementById('serial-dot').className = 'dot';
      document.getElementById('serial-status-text').textContent = 'IoT Serial: Disconnected';
      btnSerial.textContent = '🔌 เชื่อมต่อ Arduino / ESP32 (WebSerial)';
    }
  }

  // --- MediaPipe Hands & Webcam Integration ---
  const videoEl = document.getElementById('webcam-video');
  const trackCanvas = document.getElementById('tracking-canvas');
  const trackCtx = trackCanvas.getContext('2d');
  const camOverlay = document.getElementById('camera-overlay');
  const btnToggleCam = document.getElementById('btn-toggle-cam');
  const btnStartCamOverlay = document.getElementById('btn-start-cam-overlay');
  const handDetectTag = document.getElementById('hand-detect-tag');

  let handsInstance = null;
  let mediaStream = null;
  let animCamActive = false;

  function onHandResults(results) {
    trackCtx.save();
    trackCtx.clearRect(0, 0, trackCanvas.width, trackCanvas.height);

    if (state.mirrorCam) {
      trackCtx.translate(trackCanvas.width, 0);
      trackCtx.scale(-1, 1);
    }

    // Draw webcam frame
    if (results.image) {
      trackCtx.drawImage(results.image, 0, 0, trackCanvas.width, trackCanvas.height);
    }

    if (results.multiHandLandmarks && results.multiHandLandmarks.length > 0) {
      const lm = results.multiHandLandmarks[0];
      handDetectTag.textContent = 'HAND LOCKED';
      handDetectTag.style.borderColor = '#10b981';
      handDetectTag.style.color = '#6ee7b7';

      // Draw skeleton connectors & landmarks
      if (window.drawConnectors && window.HAND_CONNECTIONS) {
        window.drawConnectors(trackCtx, lm, window.HAND_CONNECTIONS, {
          color: '#0ea5e9',
          lineWidth: 3
        });
        window.drawLandmarks(trackCtx, lm, {
          color: '#f8fafc',
          fillColor: '#10b981',
          lineWidth: 1,
          radius: 4
        });
      }

      // Compute key hand control features:
      // 1. Palm center (average of wrist #0, index MCP #5, pinky MCP #17, middle MCP #9)
      const rawPalmX = (lm[0].x + lm[5].x + lm[9].x + lm[17].x) / 4;
      const palmX = state.mirrorCam ? 1.0 - rawPalmX : rawPalmX;
      const palmY = (lm[0].y + lm[5].y + lm[9].y + lm[17].y) / 4;

      // 2. Apparent hand size (2D distance between Wrist #0 and Middle MCP #9) for depth (reach) estimation
      const handScale = Math.hypot(lm[9].x - lm[0].x, lm[9].y - lm[0].y);

      // 3. Thumb tip (#4) to Index tip (#8) pinch distance normalized by handScale
      const pinchDist = Math.hypot(lm[4].x - lm[8].x, lm[4].y - lm[8].y);
      const normPinch = pinchDist / Math.max(0.08, handScale);
      const gripperOpenPct = THREE.MathUtils.clamp(((normPinch - 0.22) / 0.85) * 100, 0, 100);

      // Draw pinch line between thumb (#4) and index (#8)
      const tx = lm[4].x * trackCanvas.width;
      const ty = lm[4].y * trackCanvas.height;
      const ix = lm[8].x * trackCanvas.width;
      const iy = lm[8].y * trackCanvas.height;
      trackCtx.beginPath();
      trackCtx.moveTo(tx, ty);
      trackCtx.lineTo(ix, iy);
      trackCtx.strokeStyle = gripperOpenPct < 28 ? '#f43f5e' : '#10b981';
      trackCtx.lineWidth = 4;
      trackCtx.stroke();

      // Update pinch UI bar
      document.getElementById('pinch-bar').style.width = `${gripperOpenPct}%`;
      document.getElementById('pinch-val-text').textContent =
        gripperOpenPct < 28 ? `GRAB (${Math.round(gripperOpenPct)}%)` : `${Math.round(gripperOpenPct)}%`;

      // Map to Robot 4-DOF targets
      state.demoMode = null;
      if (state.handMode === 'ik') {
        // Map palmX (0.15..0.85) -> target Y (-120..+120 mm)
        // Map palmY (0.85..0.15) -> target Z (15..195 mm)
        // Map handScale (0.12..0.34) -> target X reach (90..195 mm)
        const targetY = ((0.5 - palmX) / 0.38) * 115.0;
        const targetZ = THREE.MathUtils.clamp(20.0 + (0.85 - palmY) * 230.0, 14.0, 200.0);
        const targetX = THREE.MathUtils.clamp(95.0 + ((handScale - 0.14) / 0.18) * 95.0, 80.0, 195.0);

        const ikSol = solveIK(targetX, targetY, targetZ);
        state.target.j1 = ikSol.j1;
        state.target.j2 = ikSol.j2;
        state.target.j3 = ikSol.j3;
        state.target.j4 = gripperOpenPct;
      } else {
        // Direct Joint Mapping
        state.target.j1 = THREE.MathUtils.clamp((0.5 - palmX) * 190, -90, 90);
        state.target.j2 = THREE.MathUtils.clamp((palmY - 0.45) * 140, -45, 65);
        // Hand roll angle (index MCP #5 to pinky MCP #17) controls Elbow j3
        const handAngle = Math.atan2(lm[17].y - lm[5].y, lm[17].x - lm[5].x);
        state.target.j3 = THREE.MathUtils.clamp(handAngle * 65, -55, 55);
        state.target.j4 = gripperOpenPct;
      }
      syncSlidersFromState();
    } else {
      handDetectTag.textContent = 'NO HAND';
      handDetectTag.style.borderColor = 'rgba(56, 189, 248, 0.35)';
      handDetectTag.style.color = '#e2e8f0';
    }

    trackCtx.restore();
  }

  async function toggleWebcam() {
    if (state.camRunning) {
      state.camRunning = false;
      animCamActive = false;
      if (mediaStream) {
        mediaStream.getTracks().forEach((t) => t.stop());
        mediaStream = null;
      }
      camOverlay.style.display = 'flex';
      document.getElementById('cam-dot').className = 'dot';
      document.getElementById('cam-status-text').textContent = 'Camera: Standby';
      btnToggleCam.textContent = '📷 เปิดกล้อง Hand Tracking';
      btnToggleCam.classList.remove('danger');
      return;
    }

    try {
      document.getElementById('cam-dot').className = 'dot warn';
      document.getElementById('cam-status-text').textContent = 'Camera: Initializing AI...';

      if (!handsInstance) {
        handsInstance = new window.Hands({
          locateFile: (file) => `https://cdn.jsdelivr.net/npm/@mediapipe/hands@0.4/${file}`
        });
        handsInstance.setOptions({
          maxNumHands: 1,
          modelComplexity: 1,
          minDetectionConfidence: 0.65,
          minTrackingConfidence: 0.65
        });
        handsInstance.onResults(onHandResults);
      }

      mediaStream = await navigator.mediaDevices.getUserMedia({
        video: { width: 640, height: 480, facingMode: 'user' },
        audio: false
      });
      videoEl.srcObject = mediaStream;
      await videoEl.play();

      state.camRunning = true;
      animCamActive = true;
      camOverlay.style.display = 'none';
      document.getElementById('cam-dot').className = 'dot active';
      document.getElementById('cam-status-text').textContent = 'Camera: Live Hand Tracking';
      btnToggleCam.textContent = '⏹️ ปิดกล้อง (Stop Camera)';
      btnToggleCam.classList.add('danger');

      async function processVideoFrame() {
        if (!animCamActive) return;
        try {
          if (videoEl.readyState >= 2) {
            await handsInstance.send({ image: videoEl });
          }
        } catch (err) {
          console.error('Hand tracking frame error:', err);
          document.getElementById('cam-dot').className = 'dot warn';
          document.getElementById('cam-status-text').textContent = 'Camera: Tracking Error';
          animCamActive = false;
          return;
        }
        if (animCamActive) {
          requestAnimationFrame(processVideoFrame);
        }
      }
      requestAnimationFrame(processVideoFrame);
    } catch (err) {
      console.error('Webcam error:', err);
      document.getElementById('cam-dot').className = 'dot warn';
      document.getElementById('cam-status-text').textContent = 'Camera: Permission/Error';
      alert(
        'ไม่สามารถเปิดกล้องเว็บแคมได้: ' +
        (err.message || err) +
        '\nตรวจสอบว่าอนุญาตสิทธิ์การใช้กล้องในเบราว์เซอร์แล้ว'
      );
    }
  }

  btnToggleCam.addEventListener('click', toggleWebcam);
  btnStartCamOverlay.addEventListener('click', toggleWebcam);

  // --- Main Render & Simulation Loop ---
  let lastTime = performance.now();
  let frameCounter = 0;
  let fpsTimer = performance.now();

  function animate(now) {
    requestAnimationFrame(animate);
    const dt = Math.min(0.05, (now - lastTime) / 1000);
    lastTime = now;

    // FPS counter
    frameCounter++;
    if (now - fpsTimer >= 500) {
      const fps = Math.round((frameCounter * 1000) / (now - fpsTimer));
      document.getElementById('fps-badge').textContent = `${fps} FPS`;
      frameCounter = 0;
      fpsTimer = now;
    }

    // Handle automated demo sequences
    if (state.demoMode === 'pick') {
      state.demoTime += dt;
      const t = state.demoTime % 8.0;
      if (t < 1.5) {
        // Move above cube at [135, 45, 60]
        const sol = solveIK(135, 45, 65);
        state.target = { ...sol, j4: 65, wrist: 0 };
      } else if (t < 2.8) {
        // Descend to cube [135, 45, 16]
        const sol = solveIK(135, 45, 16);
        state.target = { ...sol, j4: 60, wrist: 0 };
      } else if (t < 3.6) {
        // Close gripper to grab cube
        const sol = solveIK(135, 45, 16);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 5.2) {
        // Lift & swing to drop zone [135, -55, 85]
        const frac = (t - 3.6) / 1.6;
        const y = 45 - frac * 100;
        const z = 16 + Math.sin(frac * Math.PI) * 75;
        const sol = solveIK(135, y, z);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 6.2) {
        // Lower to drop zone [135, -55, 18]
        const sol = solveIK(135, -55, 18);
        state.target = { ...sol, j4: 12, wrist: 0 };
      } else if (t < 7.0) {
        // Release cube
        const sol = solveIK(135, -55, 22);
        state.target = { ...sol, j4: 65, wrist: 0 };
      } else {
        // Return cube to start for next cycle
        if (!cubeGrabbed) targetCube.position.copy(CUBE_HOME);
        const sol = solveIK(140, 0, 110);
        state.target = { ...sol, j4: 45, wrist: 0 };
      }
      syncSlidersFromState();
    } else if (state.demoMode === 'circle') {
      state.demoTime += dt * 1.8;
      const tx = 135 + 25 * Math.cos(state.demoTime);
      const ty = 55 * Math.sin(state.demoTime);
      const tz = 95 + 35 * Math.sin(state.demoTime * 2);
      const sol = solveIK(tx, ty, tz);
      state.target.j1 = sol.j1;
      state.target.j2 = sol.j2;
      state.target.j3 = sol.j3;
      state.target.j4 = 35 + 25 * Math.sin(state.demoTime * 3);
      syncSlidersFromState();
    } else if (state.playing && state.recordedFrames.length > 0) {
      const f = state.recordedFrames[state.playIndex];
      state.target = { ...f };
      state.playIndex = (state.playIndex + 1) % state.recordedFrames.length;
      syncSlidersFromState();
    }

    // Record pose frame if recording active
    if (state.recording) {
      state.recordedFrames.push({ ...state.current });
      document.getElementById('btn-record-pose').textContent = `⏹️ หยุด (${state.recordedFrames.length}f)`;
    }

    // Smooth Exponential Moving Average (EMA) interpolation
    const alpha = state.smoothing;
    ['j1', 'j2', 'j3', 'j4', 'wrist'].forEach((k) => {
      state.current[k] += (state.target[k] - state.current[k]) * alpha;
    });

    updateJointLabels();
    updateArmKinematics();
    sendSerialTelemetry(now);

    controls.update();
    renderer.render(scene, camera);
  }

  window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // Initial kinematics & start loop
  updateArmKinematics();
  requestAnimationFrame(animate);
})();
