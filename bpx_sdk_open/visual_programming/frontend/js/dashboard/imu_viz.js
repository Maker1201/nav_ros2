/**
 * BPX IMU 姿态可视化
 */

class ImuVisualizer {
    constructor() {
        this.rollEl = document.getElementById('val-roll');
        this.pitchEl = document.getElementById('val-pitch');
        this.yawEl = document.getElementById('val-yaw');
    }

    /**
     * 更新 IMU 显示
     * @param {number[]} rpy - [roll, pitch, yaw] 弧度
     */
    update(rpy) {
        if (!rpy || rpy.length < 3) return;

        const toDeg = (rad) => (rad * 180 / Math.PI).toFixed(2);

        if (this.rollEl) this.rollEl.textContent = toDeg(rpy[0]);
        if (this.pitchEl) this.pitchEl.textContent = toDeg(rpy[1]);
        if (this.yawEl) this.yawEl.textContent = toDeg(rpy[2]);
    }
}
