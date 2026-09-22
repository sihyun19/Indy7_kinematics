// Three.js 3D 환경 기본 설정 및 Indy7 Kinematics 뷰어 (Option A: Direct World Matrix)

let scene, camera, renderer, controls;
let linkMeshes = [];
let tcpAxesHelper = null;
let isUpdating = false;
let pendingAngles = null;

const STL_PATHS = [
    '/model/meshes/indy7/visual/Indy7_0.stl',
    '/model/meshes/indy7/visual/Indy7_1.stl',
    '/model/meshes/indy7/visual/Indy7_2.stl',
    '/model/meshes/indy7/visual/Indy7_3.stl',
    '/model/meshes/indy7/visual/Indy7_4.stl',
    '/model/meshes/indy7/visual/Indy7_5.stl',
    '/model/meshes/indy7/visual/Indy7_6.stl'
];

function init() {
    const container = document.getElementById('canvas-container');

    // 씬 생성
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf5f7fb);

    // 카메라 생성 (Z-Up 좌표계)
    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 50.0);
    camera.up.set(0, 0, 1);
    camera.position.set(1.5, -1.8, 1.3);

    // 렌더러 생성
    renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    // OrbitControls 생성
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0.6);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.maxPolarAngle = Math.PI / 2 + 0.1; // 바닥 아래로 과도하게 내려가지 않도록 제한

    // 조명 구성
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    scene.add(ambientLight);

    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.75);
    dirLight1.position.set(3, -4, 6);
    dirLight1.castShadow = true;
    dirLight1.shadow.mapSize.width = 1024;
    dirLight1.shadow.mapSize.height = 1024;
    scene.add(dirLight1);

    const dirLight2 = new THREE.DirectionalLight(0xb0c4de, 0.35);
    dirLight2.position.set(-3, 4, 3);
    scene.add(dirLight2);

    // 그리드 생성 (Z-Up 기준 XY 평면)
    const gridHelper = new THREE.GridHelper(3.0, 30, 0x4a5568, 0xd2d6dc);
    gridHelper.rotation.x = Math.PI / 2;
    gridHelper.position.set(0, 0, -0.001);
    scene.add(gridHelper);

    // 베이스 월드 좌표계 축 (RGB: X(빨강), Y(초록), Z(파랑))
    const worldAxes = new THREE.AxesHelper(0.3);
    scene.add(worldAxes);

    // TCP 말단 좌표계 표시용 도우미
    tcpAxesHelper = new THREE.AxesHelper(0.15);
    tcpAxesHelper.matrixAutoUpdate = false;
    scene.add(tcpAxesHelper);

    // 이벤트 리스너
    window.addEventListener('resize', onWindowResize);
    setupUIEventListeners();

    // 메쉬 로드 및 초기 포즈 설정
    loadRobotMeshes()
        .then(() => {
            const badge = document.getElementById('status-badge');
            if (badge) {
                badge.innerText = '준비 완료';
                badge.classList.add('ready');
            }
            requestRobotPose([0, 0, 0, 0, 0, 0]);
        })
        .catch((err) => {
            console.error('메쉬 로드 실패:', err);
            const badge = document.getElementById('status-badge');
            if (badge) {
                badge.innerText = '로드 오류';
                badge.style.background = '#fed7d7';
                badge.style.color = '#9b2c2c';
            }
        });

    animate();
}

function loadRobotMeshes() {
    const loader = new THREE.STLLoader();
    const badge = document.getElementById('status-badge');
    let loadedCount = 0;

    const promises = STL_PATHS.map((path, idx) => {
        return new Promise((resolve, reject) => {
            loader.load(
                path,
                (geometry) => {
                    geometry.computeVertexNormals();

                    // Base는 짙은 메탈 그레이, Link 1~6은 뉴로메카 실버-화이트
                    const material = new THREE.MeshStandardMaterial({
                        color: idx === 0 ? 0x2d3748 : 0xe2e8f0,
                        roughness: idx === 0 ? 0.5 : 0.35,
                        metalness: idx === 0 ? 0.2 : 0.25
                    });

                    const mesh = new THREE.Mesh(geometry, material);
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;

                    // Option A 핵심: Three.js 자체 위치/자세 자동 계산 비활성화 (전역 변환행렬 직접 주입)
                    mesh.matrixAutoUpdate = false;

                    scene.add(mesh);
                    linkMeshes[idx] = mesh;

                    loadedCount++;
                    if (badge) {
                        badge.innerText = `로딩 중 (${loadedCount}/7)`;
                    }
                    resolve();
                },
                undefined,
                (error) => {
                    console.error(`STL 로드 오류 [${path}]:`, error);
                    reject(error);
                }
            );
        });
    });

    return Promise.all(promises);
}

function setupUIEventListeners() {
    for (let i = 0; i < 6; i++) {
        const slider = document.getElementById(`slider-q${i}`);
        const valText = document.getElementById(`val-q${i}`);
        if (!slider) continue;

        slider.addEventListener('input', () => {
            if (valText) {
                valText.innerText = `${parseFloat(slider.value).toFixed(1)}°`;
            }
            onSliderChanged();
        });
    }

    const btnHome = document.getElementById('btn-home');
    if (btnHome) {
        btnHome.addEventListener('click', () => {
            setSliderValues([0, 0, 0, 0, 0, 0]);
            onSliderChanged();
        });
    }

    const btnTest = document.getElementById('btn-test');
    if (btnTest) {
        btnTest.addEventListener('click', () => {
            setSliderValues([0, -30, 60, 0, 45, 0]);
            onSliderChanged();
        });
    }
}

function setSliderValues(anglesDeg) {
    for (let i = 0; i < 6; i++) {
        const slider = document.getElementById(`slider-q${i}`);
        const valText = document.getElementById(`val-q${i}`);
        if (slider) {
            slider.value = anglesDeg[i];
        }
        if (valText) {
            valText.innerText = `${anglesDeg[i].toFixed(1)}°`;
        }
    }
}

function getSliderAnglesRad() {
    const q_rad = [];
    for (let i = 0; i < 6; i++) {
        const slider = document.getElementById(`slider-q${i}`);
        const deg = slider ? parseFloat(slider.value) : 0.0;
        q_rad.push((deg * Math.PI) / 180.0);
    }
    return q_rad;
}

function onSliderChanged() {
    const q = getSliderAnglesRad();
    requestRobotPose(q);
}

function requestRobotPose(q_rad) {
    if (isUpdating) {
        pendingAngles = q_rad;
        return;
    }

    isUpdating = true;
    const query = q_rad.map((v) => v.toFixed(6)).join(',');

    fetch(`/api/fk?q=${query}`)
        .then((response) => {
            if (!response.ok) throw new Error(`HTTP error ${response.status}`);
            return response.json();
        })
        .then((data) => {
            applyKinematicsData(data);
        })
        .catch((err) => {
            console.error('FK 요청 에러:', err);
        })
        .finally(() => {
            isUpdating = false;
            if (pendingAngles !== null) {
                const nextQ = pendingAngles;
                pendingAngles = null;
                requestRobotPose(nextQ);
            }
        });
}

function applyKinematicsData(data) {
    if (data.matrices && Array.isArray(data.matrices)) {
        data.matrices.forEach((matColMajor, idx) => {
            if (linkMeshes[idx]) {
                linkMeshes[idx].matrix.fromArray(matColMajor);
            }
        });
    }

    if (tcpAxesHelper && data.tcp_matrix) {
        tcpAxesHelper.matrix.fromArray(data.tcp_matrix);
    }

    if (data.tcp) {
        const xEl = document.getElementById('tcp-x');
        const yEl = document.getElementById('tcp-y');
        const zEl = document.getElementById('tcp-z');
        if (xEl) xEl.innerText = (data.tcp[0] * 1000.0).toFixed(1);
        if (yEl) yEl.innerText = (data.tcp[1] * 1000.0).toFixed(1);
        if (zEl) zEl.innerText = (data.tcp[2] * 1000.0).toFixed(1);
    }

    if (data.tcp_rpy) {
        const rEl = document.getElementById('tcp-roll');
        const pEl = document.getElementById('tcp-pitch');
        const yawEl = document.getElementById('tcp-yaw');
        if (rEl) rEl.innerText = `${data.tcp_rpy[0].toFixed(1)}°`;
        if (pEl) pEl.innerText = `${data.tcp_rpy[1].toFixed(1)}°`;
        if (yawEl) yawEl.innerText = `${data.tcp_rpy[2].toFixed(1)}°`;
    }
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

window.onload = init;
