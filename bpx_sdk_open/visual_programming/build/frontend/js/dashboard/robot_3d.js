/**
 * BPX 3D 机器狗关节可视化
 * 使用 Three.js 渲染简化的机器狗关节模型
 */

class Robot3DView {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) {
            console.error('[Robot3D] Canvas not found:', canvasId);
            return;
        }

        this.ready = false;
        this.jointPositions = new Array(12).fill(0);

        // 延迟初始化，确保 DOM 布局完成
        this._initWhenReady();
    }

    _initWhenReady() {
        // 等待 canvas 有实际尺寸后再初始化
        const checkSize = () => {
            const w = this.canvas.clientWidth;
            const h = this.canvas.clientHeight;
            if (w > 0 && h > 0) {
                this._init(w, h);
            } else {
                requestAnimationFrame(checkSize);
            }
        };
        // 使用 requestAnimationFrame 确保在下一帧检查
        requestAnimationFrame(checkSize);
    }

    _init(w, h) {
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(60, w / h, 0.1, 100);
        this.renderer = new THREE.WebGLRenderer({ canvas: this.canvas, antialias: true, alpha: true });

        this.renderer.setSize(w, h);
        this.renderer.setPixelRatio(window.devicePixelRatio || 1);
        this.renderer.setClearColor(0x0d0d1a, 1);

        // 相机位置
        this.camera.position.set(0.8, 0.8, 0.8);
        this.camera.lookAt(0, 0, 0);

        // 环境光
        this.scene.add(new THREE.AmbientLight(0x404040, 0.6));

        // 方向光
        const light = new THREE.DirectionalLight(0xffffff, 0.8);
        light.position.set(2, 3, 1);
        this.scene.add(light);

        // 坐标轴辅助
        const axes = new THREE.AxesHelper(0.3);
        this.scene.add(axes);

        // 创建机器狗模型
        this._createRobotModel();

        // 鼠标控制
        this._setupMouseControl();

        // 监听容器尺寸变化
        this._setupResizeObserver();

        this.ready = true;
        console.log('[Robot3D] Initialized, size:', w, 'x', h);

        // 开始渲染
        this._animate();
    }

    _createRobotModel() {
        // 材质
        const bodyMat = new THREE.MeshPhongMaterial({ color: 0x00cec9, transparent: true, opacity: 0.8 });
        const legMat = new THREE.MeshPhongMaterial({ color: 0x0984e3 });
        const jointMat = new THREE.MeshPhongMaterial({ color: 0xe17055 });

        // 身体（长方体）
        const bodyGeo = new THREE.BoxGeometry(0.4, 0.1, 0.25);
        this.body = new THREE.Mesh(bodyGeo, bodyMat);
        this.scene.add(this.body);

        // 四条腿，保存命名引用
        this.legs = [];
        const legPositions = [
            { x: 0.18, z: 0.12, name: 'LF' },   // 左前
            { x: 0.18, z: -0.12, name: 'RF' },   // 右前
            { x: -0.18, z: 0.12, name: 'LR' },   // 左后
            { x: -0.18, z: -0.12, name: 'RR' }    // 右后
        ];

        for (let i = 0; i < 4; i++) {
            const legData = this._createLeg(legMat, jointMat);
            legData.group.position.set(legPositions[i].x, -0.05, legPositions[i].z);
            this.body.add(legData.group);
            this.legs.push(legData);
        }
    }

    _createLeg(legMat, jointMat) {
        const group = new THREE.Group();

        // 外展关节（球体）— 整条腿的根节点
        const abadJoint = new THREE.Mesh(new THREE.SphereGeometry(0.025), jointMat);
        group.add(abadJoint);

        // 上臂
        const upperGeo = new THREE.CylinderGeometry(0.015, 0.012, 0.15);
        const upper = new THREE.Mesh(upperGeo, legMat);
        upper.position.y = -0.075;
        abadJoint.add(upper);

        // 髋关节
        const hipJoint = new THREE.Mesh(new THREE.SphereGeometry(0.02), jointMat);
        hipJoint.position.y = -0.15;
        abadJoint.add(hipJoint);

        // 前臂
        const lowerGeo = new THREE.CylinderGeometry(0.012, 0.01, 0.15);
        const lower = new THREE.Mesh(lowerGeo, legMat);
        lower.position.y = -0.075;
        hipJoint.add(lower);

        // 膝关节
        const kneeJoint = new THREE.Mesh(new THREE.SphereGeometry(0.015), jointMat);
        kneeJoint.position.y = -0.15;
        hipJoint.add(kneeJoint);

        // 脚
        const foot = new THREE.Mesh(
            new THREE.SphereGeometry(0.018),
            new THREE.MeshPhongMaterial({ color: 0x00b894 })
        );
        foot.position.y = -0.02;
        kneeJoint.add(foot);

        return {
            group: group,       // 腿的根 Group
            abad: abadJoint,    // 外展关节 Mesh
            hip: hipJoint,      // 髋关节 Mesh
            knee: kneeJoint     // 膝关节 Mesh
        };
    }

    _setupMouseControl() {
        let isDown = false;
        let startX, startY;
        let rotX = 0.5, rotY = 0.5;

        this.canvas.addEventListener('mousedown', (e) => {
            isDown = true;
            startX = e.clientX;
            startY = e.clientY;
        });

        this.canvas.addEventListener('mousemove', (e) => {
            if (!isDown) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            rotY += dx * 0.01;
            rotX += dy * 0.01;
            startX = e.clientX;
            startY = e.clientY;

            const dist = 1.2;
            this.camera.position.set(
                dist * Math.sin(rotY) * Math.cos(rotX),
                dist * Math.sin(rotX),
                dist * Math.cos(rotY) * Math.cos(rotX)
            );
            this.camera.lookAt(0, 0, 0);
        });

        this.canvas.addEventListener('mouseup', () => isDown = false);
        this.canvas.addEventListener('mouseleave', () => isDown = false);
    }

    _setupResizeObserver() {
        if (typeof ResizeObserver === 'undefined') return;

        this._resizeObserver = new ResizeObserver((entries) => {
            for (const entry of entries) {
                if (entry.target === this.canvas) {
                    this.resize();
                }
            }
        });
        this._resizeObserver.observe(this.canvas);
    }

    /**
     * 更新关节角度
     * @param {number[]} positions - 12 个关节角度（弧度）
     */
    updateJoints(positions) {
        if (!this.ready || !positions || positions.length < 12) return;

        for (let i = 0; i < 4; i++) {
            const leg = this.legs[i];
            if (!leg) continue;

            const abad = positions[i * 3 + 0] || 0;
            const hip = positions[i * 3 + 1] || 0;
            const knee = positions[i * 3 + 2] || 0;

            // 外展旋转（绕 X 轴）
            leg.abad.rotation.x = abad;

            // 髋关节旋转（绕 Z 轴）
            leg.hip.rotation.z = hip;

            // 膝关节旋转（绕 Z 轴）
            leg.knee.rotation.z = knee;
        }
    }

    /**
     * 更新 IMU 姿态
     */
    updateImu(rpy) {
        if (!this.ready || !rpy || rpy.length < 3) return;
        if (this.body) {
            this.body.rotation.x = rpy[1] * 0.5; // pitch
            this.body.rotation.z = rpy[0] * 0.5; // roll
        }
    }

    _animate() {
        requestAnimationFrame(() => this._animate());
        this.renderer.render(this.scene, this.camera);
    }

    resize() {
        if (!this.canvas || !this.renderer) return;
        const w = this.canvas.clientWidth;
        const h = this.canvas.clientHeight;
        if (w === 0 || h === 0) return;

        this.camera.aspect = w / h;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(w, h);
    }

    destroy() {
        if (this._resizeObserver) {
            this._resizeObserver.disconnect();
        }
    }
}
