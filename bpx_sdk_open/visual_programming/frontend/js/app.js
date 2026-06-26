/**
 * BPX 图形化编程 - 主应用逻辑
 */

(function() {
    'use strict';

    // === 全局状态 ===
    let blocklyWorkspace = null;
    let robot3dView = null;
    let imuViz = null;
    let batteryGauge = null;
    let isExecuting = false;

    // === 初始化 ===
    document.addEventListener('DOMContentLoaded', function() {
        initBlockly();
        initDashboard();
        initEventListeners();
        initWebSocket();
        initTemperatureBars();

        console.log('[App] BPX Visual Programming initialized');
    });

    // === Blockly 初始化 ===
    function initBlockly() {
        const toolbox = {
            kind: 'categoryToolbox',
            contents: [
                {
                    kind: 'category',
                    name: '🔌 连接',
                    colour: '210',
                    contents: [
                        { kind: 'block', type: 'bpx_connect' },
                        { kind: 'block', type: 'bpx_disconnect' },
                        { kind: 'block', type: 'bpx_is_connected' }
                    ]
                },
                {
                    kind: 'category',
                    name: '🐕 运动',
                    colour: '120',
                    contents: [
                        { kind: 'block', type: 'bpx_stand_up' },
                        { kind: 'block', type: 'bpx_sit_down' },
                        { kind: 'block', type: 'bpx_damping' },
                        { kind: 'block', type: 'bpx_upright' },
                        { kind: 'block', type: 'bpx_zero_positions' },
                        { kind: 'block', type: 'bpx_gait_select' },
                        {
                            kind: 'block', type: 'bpx_set_forward',
                            inputs: {
                                SPEED: { shadow: { type: 'math_number', fields: { NUM: 0.3 } } }
                            }
                        },
                        {
                            kind: 'block', type: 'bpx_set_lateral',
                            inputs: {
                                SPEED: { shadow: { type: 'math_number', fields: { NUM: 0.2 } } }
                            }
                        },
                        {
                            kind: 'block', type: 'bpx_set_turn',
                            inputs: {
                                SPEED: { shadow: { type: 'math_number', fields: { NUM: 0.5 } } }
                            }
                        },
                        {
                            kind: 'block', type: 'bpx_set_velocity',
                            inputs: {
                                X: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                                Y: { shadow: { type: 'math_number', fields: { NUM: 0 } } },
                                YAW: { shadow: { type: 'math_number', fields: { NUM: 0 } } }
                            }
                        },
                        { kind: 'block', type: 'bpx_velocity_control_flag' }
                    ]
                },
                {
                    kind: 'category',
                    name: '📊 传感器',
                    colour: '30',
                    contents: [
                        { kind: 'block', type: 'bpx_battery_level' },
                        { kind: 'block', type: 'bpx_battery_current' },
                        { kind: 'block', type: 'bpx_motion_state' },
                        { kind: 'block', type: 'bpx_last_motion_state' },
                        { kind: 'block', type: 'bpx_gait' },
                        { kind: 'block', type: 'bpx_last_gait' },
                        { kind: 'block', type: 'bpx_sub_gait' },
                        { kind: 'block', type: 'bpx_imu_rpy' },
                        { kind: 'block', type: 'bpx_imu_quat' },
                        { kind: 'block', type: 'bpx_imu_acc' },
                        { kind: 'block', type: 'bpx_imu_omega' },
                        { kind: 'block', type: 'bpx_body_velocity' },
                        { kind: 'block', type: 'bpx_leg_odom' },
                        { kind: 'block', type: 'bpx_joint_position' },
                        { kind: 'block', type: 'bpx_motor_temperature' },
                        { kind: 'block', type: 'bpx_driver_temperature' }
                    ]
                },
                {
                    kind: 'category',
                    name: '🦿 关节',
                    colour: '270',
                    contents: [
                        { kind: 'block', type: 'bpx_set_all_joints' },
                        { kind: 'block', type: 'bpx_set_leg_joint' },
                        { kind: 'block', type: 'bpx_set_gains' },
                        { kind: 'block', type: 'bpx_smooth_move' },
                        { kind: 'block', type: 'bpx_zero_joints' }
                    ]
                },
                {
                    kind: 'category',
                    name: '🔁 流程',
                    colour: '45',
                    contents: [
                        { kind: 'block', type: 'bpx_wait' },
                        { kind: 'block', type: 'bpx_repeat' },
                        { kind: 'block', type: 'bpx_repeat_until' },
                        { kind: 'block', type: 'bpx_if_else' },
                        { kind: 'block', type: 'bpx_emergency_stop' },
                        { kind: 'block', type: 'bpx_compare' },
                        { kind: 'block', type: 'bpx_logic' },
                        { kind: 'block', type: 'bpx_math' }
                    ]
                }
            ]
        };

        blocklyWorkspace = Blockly.inject('blockly-workspace', {
            toolbox: toolbox,
            grid: {
                spacing: 20,
                length: 3,
                colour: '#2a2a5e',
                snap: true
            },
            zoom: {
                controls: true,
                startScale: 0.9,
                maxScale: 2,
                minScale: 0.3,
                scaleSpeed: 1.2
            },
            trashcan: true,
            move: {
                scrollbars: true,
                drag: true,
                wheel: true
            },
            renderer: 'zelos',
            theme: Blockly.Themes.Classic
        });

        // 窗口大小变化时重新调整
        window.addEventListener('resize', function() {
            Blockly.svgResize(blocklyWorkspace);
            if (robot3dView) robot3dView.resize();
        });
    }

    // === 仪表盘初始化 ===
    function initDashboard() {
        robot3dView = new Robot3DView('robot-3d-canvas');
        imuViz = new ImuVisualizer();
        batteryGauge = new BatteryGauge();
    }

    // === 温度条初始化 ===
    function initTemperatureBars() {
        const container = document.getElementById('temp-bars');
        if (!container) return;

        const jointNames = [
            '左前-外展', '左前-髋', '左前-膝',
            '右前-外展', '右前-髋', '右前-膝',
            '左后-外展', '左后-髋', '左后-膝',
            '右后-外展', '右后-髋', '右后-膝'
        ];

        let html = '';
        for (let i = 0; i < 12; i++) {
            html += '<div class="temp-bar">' +
                '<div class="temp-bar-value" id="temp-val-' + i + '">0</div>' +
                '<div class="temp-bar-fill"><div class="temp-bar-fill-inner" id="temp-fill-' + i + '"></div></div>' +
                '<div class="temp-bar-label">' + jointNames[i] + '</div>' +
                '</div>';
        }
        container.innerHTML = html;
    }

    // === 事件监听 ===
    function initEventListeners() {
        // 运行按钮
        document.getElementById('btn-run').addEventListener('click', function() {
            if (!blocklyWorkspace) return;
            const program = generateProgram(blocklyWorkspace);
            if (program.length === 0) {
                logConsole('没有可执行的积木', 'warn');
                return;
            }
            logConsole('开始执行程序 (' + program.length + ' 个步骤)', 'success');
            bpxClient.executeProgram(program);
        });

        // 停止按钮
        document.getElementById('btn-stop').addEventListener('click', function() {
            bpxClient.stop();
            logConsole('已发送停止指令', 'warn');
        });

        // 紧急停止按钮
        document.getElementById('btn-emergency').addEventListener('click', function() {
            bpxClient.send({ action: 'emergencyStop' });
            logConsole('🔴 紧急停止！', 'error');
        });

        // 设置按钮
        document.getElementById('btn-settings').addEventListener('click', function() {
            document.getElementById('settings-dialog').style.display = 'flex';
            bpxClient.requestInterfaces();
        });

        // 关闭设置对话框
        document.getElementById('btn-close-settings').addEventListener('click', closeSettings);
        document.getElementById('btn-cancel-settings').addEventListener('click', closeSettings);

        // 通讯方式卡片选择
        document.querySelectorAll('.profile-card').forEach(function(card) {
            card.addEventListener('click', function() {
                document.querySelectorAll('.profile-card').forEach(function(c) {
                    c.classList.remove('selected');
                });
                card.classList.add('selected');
                var ip = card.getAttribute('data-ip');
                document.getElementById('input-robot-ip').value = ip;
            });
        });

        // 连接按钮 - 直接调用 API，确保可靠
        document.getElementById('btn-connect-settings').addEventListener('click', function() {
            var ip = document.getElementById('input-robot-ip').value;
            var iface = document.getElementById('select-interface').value;
            var connectBtn = document.getElementById('btn-connect-settings');

            connectBtn.disabled = true;
            connectBtn.textContent = '⏳ 连接中...';
            logConsole('正在连接到 ' + ip + ' ...');

            // 直接 fetch 调用，不经过 ws_client
            fetch('/api/command', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ type: 'connect', ip: ip, interface: iface })
            })
            .then(function(r) { return r.json(); })
            .then(function(data) {
                logConsole('后端响应: ' + JSON.stringify(data));
                if (data.success) {
                    // 开始轮询连接结果
                    var pollCount = 0;
                    var pollTimer = setInterval(function() {
                        pollCount++;
                        fetch('/api/robot-status')
                            .then(function(r) { return r.json(); })
                            .then(function(status) {
                                if (status.connected) {
                                    clearInterval(pollTimer);
                                    connectBtn.disabled = false;
                                    connectBtn.textContent = '🔌 连接';
                                    updateRobotConnectionStatus(true);
                                    closeSettings();
                                    logConsole('✅ 已连接到机器狗 ' + ip, 'success');
                                } else if (pollCount > 20) {
                                    clearInterval(pollTimer);
                                    connectBtn.disabled = false;
                                    connectBtn.textContent = '🔌 连接';
                                    logConsole('❌ 连接超时', 'error');
                                }
                            })
                            .catch(function() {});
                    }, 500);
                } else {
                    connectBtn.disabled = false;
                    connectBtn.textContent = '🔌 连接';
                    logConsole('❌ 连接失败: ' + (data.message || '未知错误'), 'error');
                }
            })
            .catch(function(e) {
                connectBtn.disabled = false;
                connectBtn.textContent = '🔌 连接';
                logConsole('❌ 请求失败: ' + e.message, 'error');
            });
        });

        // 断开连接按钮
        document.getElementById('btn-disconnect-settings').addEventListener('click', function() {
            bpxClient.disconnectRobot();
            logConsole('已断开连接', 'warn');
        });

        // 恢复默认
        document.getElementById('btn-restore-defaults').addEventListener('click', function() {
            document.getElementById('input-robot-ip').value = '10.21.20.1';
            document.getElementById('select-interface').value = 'auto';
            document.getElementById('input-state-rate').value = '100';
            document.getElementById('input-cmd-rate').value = '50';
            document.getElementById('input-state-port').value = '9873';
            document.getElementById('input-joint-port').value = '7895';
            document.getElementById('input-tcp-port').value = '0';
            document.getElementById('input-web-port').value = '8080';
            // 选中有线卡片
            document.querySelectorAll('.profile-card').forEach(function(c) {
                c.classList.remove('selected');
            });
            document.getElementById('profile-wired').classList.add('selected');
        });

        // 刷新网卡列表
        document.getElementById('btn-refresh-interfaces').addEventListener('click', function() {
            bpxClient.requestInterfaces();
        });

        // 检测连通性
        document.getElementById('btn-check-ping').addEventListener('click', function() {
            var ip = document.getElementById('input-robot-ip').value;
            bpxClient.checkConnectivity(ip);
            logConsole('正在检测 ' + ip + ' 的连通性...');
        });

        // 保存程序
        document.getElementById('btn-save').addEventListener('click', function() {
            if (!blocklyWorkspace) return;
            var xml = Blockly.Xml.workspaceToDom(blocklyWorkspace);
            var xmlText = Blockly.Xml.domToText(xml);
            var blob = new Blob([xmlText], { type: 'text/xml' });
            var url = URL.createObjectURL(blob);
            var a = document.createElement('a');
            a.href = url;
            a.download = 'bpx_program_' + new Date().toISOString().slice(0, 10) + '.xml';
            a.click();
            URL.revokeObjectURL(url);
            logConsole('程序已保存', 'success');
        });

        // 加载程序
        document.getElementById('btn-load').addEventListener('click', function() {
            document.getElementById('file-input').click();
        });

        document.getElementById('file-input').addEventListener('change', function(e) {
            var file = e.target.files[0];
            if (!file) return;
            var reader = new FileReader();
            reader.onload = function(ev) {
                try {
                    blocklyWorkspace.clear();
                    var xml = Blockly.utils.xml.textToDom(ev.target.result);
                    Blockly.Xml.domToWorkspace(xml, blocklyWorkspace);
                    logConsole('程序已加载: ' + file.name, 'success');
                } catch (err) {
                    logConsole('加载失败: ' + err.message, 'error');
                }
            };
            reader.readAsText(file);
            e.target.value = '';
        });

        // 清空日志
        document.getElementById('btn-clear-log').addEventListener('click', function() {
            document.getElementById('console-output').innerHTML = '';
        });
    }

    function closeSettings() {
        document.getElementById('settings-dialog').style.display = 'none';
    }

    // === WebSocket 事件处理 ===
    function initWebSocket() {
        bpxClient.on('connected', function() {
            updateConnectionStatus(true);
            logConsole('已连接到服务器', 'success');
        });

        bpxClient.on('disconnected', function() {
            updateConnectionStatus(false);
            logConsole('与服务器断开连接', 'error');
        });

        bpxClient.on('state', function(state) {
            updateDashboard(state);
            // 保存最新状态供流程控制条件评估使用
            window.__bpxRobotState = state;

            // 检查关节连接状态，显示警告
            if (state.connected && state.jointConnected === false) {
                if (!window.__jointWarnShown) {
                    logConsole('⚠ 关节控制未连接，关节相关积木将无法执行', 'warn');
                    window.__jointWarnShown = true;
                }
            }
        });

        bpxClient.on('log', function(msg) {
            logConsole(msg);
        });

        bpxClient.on('error', function(msg) {
            logConsole(msg, 'error');
        });

        bpxClient.on('execStatus', function(status) {
            isExecuting = status.running;
            document.getElementById('btn-run').disabled = status.running;
            document.getElementById('btn-stop').disabled = !status.running;

            if (status.running && status.currentStep !== undefined && status.totalSteps !== undefined) {
                logConsole('执行中: 步骤 ' + status.currentStep + '/' + status.totalSteps +
                    (status.currentAction ? ' (' + status.currentAction + ')' : ''));
            } else if (!status.running && status.finished) {
                logConsole('程序执行完毕', 'success');
            } else if (!status.running && status.stopped) {
                logConsole('程序已停止', 'warn');
            }
        });

        bpxClient.on('networkInterfaces', function(data) {
            updateInterfaceList(data);
        });

        bpxClient.on('connectivityResult', function(data) {
            if (data.reachable) {
                logConsole(data.ip + ' 可达 ✓', 'success');
            } else {
                logConsole(data.ip + ' 不可达 ✗', 'error');
            }
        });

        bpxClient.on('robotConnected', function(data) {
            if (data.success) {
                updateRobotConnectionStatus(true);
                closeSettings();
                logConsole('已连接到机器狗', 'success');
            }
        });

        bpxClient.on('robotConnecting', function(data) {
            logConsole('正在连接机器狗，请稍候...');
            // 开始轮询连接状态
            var pollTimer = setInterval(function() {
                bpxClient.pollRobotStatus();
            }, 500);
            // 10 秒后停止轮询
            setTimeout(function() {
                clearInterval(pollTimer);
            }, 10000);
        });

        bpxClient.on('networkConfigUpdated', function(data) {
            if (data.success) {
                logConsole(data.message || '网络配置已更新', 'success');
            }
        });

        bpxClient.connect();

        // 首次加载自动显示通讯方式选择对话框
        setTimeout(function() {
            document.getElementById('settings-dialog').style.display = 'flex';
            // 从服务器获取当前配置的机器狗 IP
            fetch('/api/robot-status')
                .then(function(r) { return r.json(); })
                .then(function(status) {
                    if (status.robotIp) {
                        document.getElementById('input-robot-ip').value = status.robotIp;
                    }
                    bpxClient.requestInterfaces();
                })
                .catch(function() {
                    bpxClient.requestInterfaces();
                });
        }, 500);
    }

    // === 更新界面 ===
    function updateConnectionStatus(connected) {
        var statusEl = document.getElementById('connection-status');
        if (connected) {
            statusEl.textContent = '● 服务器已连接';
            statusEl.className = 'status connected';
        } else {
            statusEl.textContent = '● 服务器断开';
            statusEl.className = 'status disconnected';
        }
    }

    function updateRobotConnectionStatus(connected) {
        var statusEl = document.getElementById('connection-status');
        var settingsStatusEl = document.getElementById('settings-connection-status');
        var connectBtn = document.getElementById('btn-connect-settings');
        var disconnectBtn = document.getElementById('btn-disconnect-settings');

        if (connected) {
            statusEl.textContent = '● 机器狗已连接';
            statusEl.className = 'status connected';
            if (settingsStatusEl) {
                settingsStatusEl.textContent = '● 已连接';
                settingsStatusEl.className = 'status connected';
            }
            if (connectBtn) connectBtn.style.display = 'none';
            if (disconnectBtn) disconnectBtn.style.display = 'inline-block';
        } else {
            statusEl.textContent = '● 未连接';
            statusEl.className = 'status disconnected';
            if (settingsStatusEl) {
                settingsStatusEl.textContent = '● 未连接';
                settingsStatusEl.className = 'status disconnected';
            }
            if (connectBtn) connectBtn.style.display = 'inline-block';
            if (disconnectBtn) disconnectBtn.style.display = 'none';
        }
    }

    function updateDashboard(state) {
        // 运动状态（对照 SDK MotionState 枚举）
        var motionStateNames = {
            0: '趴下 LyingDown', 1: '站立中 StandingUp', 2: '被动 Passive',
            3: '坐下 SitDown', 6: '运动中 Motion'
        };
        var motionEl = document.getElementById('val-motion-state');
        if (motionEl && state.motionState !== undefined) {
            motionEl.textContent = motionStateNames[state.motionState] || state.motionState;
        }

        // 步态（对照 SDK MotionGait 枚举）
        var gaitNames = {
            0: '行走 Walk', 3: '双足 Bipedal', 4: '空翻 Flip',
            6: '行走相位 WalkPhase', 7: '姿态追踪 PoseTracking',
            8: '跑步 Running', 10: '周期行走 WalkPeriod'
        };
        var gaitEl = document.getElementById('val-gait');
        if (gaitEl && state.gait !== undefined) {
            gaitEl.textContent = gaitNames[state.gait] || state.gait;
        }

        // 电池
        if (state.battery !== undefined) {
            batteryGauge.update(state.battery);
        }

        // 速度
        var velEl = document.getElementById('val-velocity');
        if (velEl && state.bodyVelocity) {
            velEl.textContent = 'x:' + state.bodyVelocity[0].toFixed(2) +
                ' y:' + state.bodyVelocity[1].toFixed(2) +
                ' yaw:' + state.bodyVelocity[2].toFixed(2);
        }

        // IMU
        if (state.imuRpy) {
            imuViz.update(state.imuRpy);
        }

        // 关节角度
        if (state.jointPos) {
            for (var i = 0; i < 12; i++) {
                var el = document.getElementById('j' + i);
                if (el) {
                    el.textContent = (state.jointPos[i] * 180 / Math.PI).toFixed(1);
                }
            }
        }

        // 温度
        if (state.motorTemp) {
            for (var i = 0; i < 12; i++) {
                var valEl = document.getElementById('temp-val-' + i);
                var fillEl = document.getElementById('temp-fill-' + i);
                if (valEl) valEl.textContent = state.motorTemp[i].toFixed(0);
                if (fillEl) {
                    var pct = Math.min(100, state.motorTemp[i] / 80 * 100);
                    fillEl.style.height = pct + '%';
                    fillEl.classList.remove('hot', 'warm');
                    if (state.motorTemp[i] > 60) fillEl.classList.add('hot');
                    else if (state.motorTemp[i] > 45) fillEl.classList.add('warm');
                }
            }
        }

        // 3D 模型
        if (robot3dView && state.jointPos) {
            robot3dView.updateJoints(state.jointPos);
        }
        if (robot3dView && state.imuRpy) {
            robot3dView.updateImu(state.imuRpy);
        }

        // 首次收到关节数据时打印日志
        if (state.jointPos && !window.__jointDataLogged) {
            window.__jointDataLogged = true;
            logConsole('✅ 已接收关节数据: [' + state.jointPos.map(function(v) { return (v * 180 / Math.PI).toFixed(1); }).join(', ') + ']°', 'success');
        }
    }

    function updateInterfaceList(data) {
        var select = document.getElementById('select-interface');
        if (!select) return;

        // 保留"自动检测"选项
        select.innerHTML = '<option value="auto">🔄 自动检测</option>';

        if (data.interfaces) {
            // 同步到全局缓存，供 Blockly 积木使用
            window.__bpxInterfaces = data.interfaces;

            data.interfaces.forEach(function(iface) {
                var opt = document.createElement('option');
                opt.value = iface.name;
                var icon = iface.type === 'ethernet' ? '🔌' :
                    iface.type === 'wireless' ? '📶' :
                        iface.type === 'usb' ? '🔗' : '❓';
                var subnet = iface.sameSubnet ? ' ✓ 同网段' : '';
                opt.textContent = icon + ' ' + iface.name + ' - ' + iface.ip + subnet;
                if (iface.sameSubnet) opt.selected = true;
                select.appendChild(opt);
            });
        }
    }

    function logConsole(message, type) {
        var console = document.getElementById('console-output');
        if (!console) return;

        var entry = document.createElement('div');
        entry.className = 'log-entry' + (type ? ' ' + type : '');
        var time = new Date().toLocaleTimeString();
        entry.textContent = '[' + time + '] ' + message;
        console.appendChild(entry);
        console.scrollTop = console.scrollHeight;
    }

})();
