//Three.js 3D 환경 기본 설정 (Z-Up 좌표계 세팅)

let scene, camera, renderer, controls;

function init() {
    const container = document.getElementById('canvas-container');

    // 씬 생성
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff); // 배경색 설정

    // 카메라 생성
    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 100.0);
    camera.up.set(0, 0, 1); // Z-Up 좌표계 설정. Three.js의 기본 좌표계는 Y-Up이므로 Z-Up으로 변경
    camera.position.set(1.5, 1.5, 1.5);

    // 렌더러 생성
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;       // 그림자 연산 활성화
    container.appendChild(renderer.domElement); //위의 코드들이 생성되며 만들어진 renderer.domElement를 container에 붙여줌

    // OrbitControls 생성
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0,0,0.7);
    controls.enableDamping = true; //카메라에 관성부여
    controls.dampingFactor = 0.05; //관성 감쇠계수

    
    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6); // 전체 환경광
    scene.add(ambientLight);
    const dirLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight1.position.set(3, -3, 5);
    scene.add(dirLight1);
    const dirLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    dirLight2.position.set(-3, 3, 2);
    scene.add(dirLight2);

    //grid 생성
    const gridHelper = new THREE.GridHelper(3, 30, 0x00ffff, 0x444444);
    gridHelper.rotation.x = Math.PI / 2; // X축을 기준으로 90도 회전. y에서 z축으로 바꾸면서 발생한 불일치 해걀
    scene.add(gridHelper);

    const axesHelper = new THREE.AxesHelper(1.5);
    axesHelper.position.set(0, 0, 0);
    scene.add(axesHelper);

    window.addEventListener('resize', onWindowResize);

    animate();
}

function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}
function animate() {
    requestAnimationFrame(animate);
    controls.update(); // 댐핑 효과 갱신
    renderer.render(scene, camera);
}
// 브라우저 DOM 로드 완료 시 초기화 실행
window.onload = init;
