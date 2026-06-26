/**
 * BPX 电池电量仪表盘
 */

class BatteryGauge {
    constructor() {
        this.valueEl = document.getElementById('val-battery');
        this.fillEl = document.getElementById('battery-fill');
    }

    /**
     * 更新电池显示
     * @param {number} level - 电量百分比 0-100
     */
    update(level) {
        if (level === undefined || level === null) return;

        if (this.valueEl) {
            this.valueEl.textContent = level + '%';
        }

        if (this.fillEl) {
            this.fillEl.style.width = level + '%';

            // 颜色变化
            this.fillEl.classList.remove('low', 'medium');
            if (level < 15) {
                this.fillEl.classList.add('low');
            } else if (level < 30) {
                this.fillEl.classList.add('medium');
            }
        }
    }
}
