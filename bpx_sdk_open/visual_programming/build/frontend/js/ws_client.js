/**
 * BPX 通信客户端
 * 使用 HTTP POST 发送指令，SSE 接收状态推送
 */
class BpxWsClient {
    constructor() {
        this.sse = null;
        this.connected = false;
        this.callbacks = {};
        this.reconnectTimer = null;
        this.reconnectDelay = 3000;
        this.logPollTimer = null;
    }

    /**
     * 注册回调
     */
    on(event, callback) {
        if (!this.callbacks[event]) {
            this.callbacks[event] = [];
        }
        this.callbacks[event].push(callback);
    }

    /**
     * 触发事件
     */
    _emit(event, data) {
        if (this.callbacks[event]) {
            this.callbacks[event].forEach(function(cb) { cb(data); });
        }
    }

    /**
     * 连接到服务器（启动 SSE 和日志轮询）
     */
    connect() {
        this._connectSSE();
        this._startLogPolling();
    }

    /**
     * SSE 连接
     */
    _connectSSE() {
        var self = this;

        if (this.sse) {
            this.sse.close();
        }

        this.sse = new EventSource('/api/state/stream');

        this.sse.onopen = function() {
            self.connected = true;
            self._emit('connected');
            console.log('[SSE] Connected');
        };

        this.sse.onmessage = function(event) {
            try {
                var data = JSON.parse(event.data);
                self._handleMessage(data);
            } catch (e) {
                console.error('[SSE] Parse error:', e);
            }
        };

        this.sse.onerror = function() {
            self.connected = false;
            self._emit('disconnected');
            console.log('[SSE] Disconnected, reconnecting...');
            self.sse.close();
            self.reconnectTimer = setTimeout(function() { self._connectSSE(); }, self.reconnectDelay);
        };
    }

    /**
     * 日志轮询（SSE 中的日志可能不完整，补充轮询）
     * 只处理尚未见过的日志，避免重复
     */
    _startLogPolling() {
        var self = this;
        this._seenLogTimes = new Set();
        this.logPollTimer = setInterval(function() {
            fetch('/api/logs')
                .then(function(r) { return r.json(); })
                .then(function(logs) {
                    if (Array.isArray(logs)) {
                        logs.forEach(function(log) {
                            // 使用 time + message 作为去重 key
                            var key = (log.time || 0) + ':' + (log.message || '');
                            if (!self._seenLogTimes.has(key)) {
                                self._seenLogTimes.add(key);
                                self._handleMessage(log);
                            }
                        });
                        // 限制 seen 集合大小，防止内存泄漏
                        if (self._seenLogTimes.size > 500) {
                            var arr = Array.from(self._seenLogTimes);
                            self._seenLogTimes = new Set(arr.slice(arr.length - 200));
                        }
                    }
                })
                .catch(function() {});
        }, 500);
    }

    /**
     * 处理收到的消息
     */
    _handleMessage(data) {
        var type = data.type || 'unknown';

        switch (type) {
            case 'state':
                this._emit('state', data);
                break;
            case 'log':
                this._emit('log', data.message || data);
                break;
            case 'execStatus':
                this._emit('execStatus', data);
                break;
            case 'error':
                this._emit('error', data.message || data);
                break;
            case 'networkInterfaces':
                this._emit('networkInterfaces', data);
                break;
            case 'networkConfigUpdated':
                this._emit('networkConfigUpdated', data);
                break;
            case 'connected':
                this._emit('robotConnected', data);
                break;
            case 'connecting':
                this._emit('robotConnecting', data);
                break;
            case 'connectivityResult':
                this._emit('connectivityResult', data);
                break;
            default:
                // 处理轮询状态响应（没有 type 字段但有 connected 字段）
                if (data.connected !== undefined) {
                    if (data.connected) {
                        this._emit('robotConnected', { success: true });
                    } else if (data.connecting) {
                        this._emit('robotConnecting', data);
                    }
                    break;
                }
                this._emit('message', data);
        }
    }

    /**
     * 发送 HTTP POST 请求
     */
    _post(url, data) {
        var self = this;
        return fetch(url, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data)
        })
        .then(function(r) { return r.json(); })
        .then(function(result) {
            self._handleMessage(result);
            return result;
        })
        .catch(function(e) {
            console.error('[HTTP] POST error:', e);
            self._emit('error', '请求失败: ' + e.message);
        });
    }

    /**
     * 发送指令（通过 /api/command）
     */
    send(data) {
        this._post('/api/command', data);
    }

    /**
     * 执行程序
     */
    executeProgram(program) {
        this._post('/api/execute', { type: 'execute', program: program });
    }

    /**
     * 停止执行
     */
    stop() {
        this._post('/api/stop', { type: 'stop' });
    }

    /**
     * 请求网卡列表
     */
    requestInterfaces() {
        this.send({ type: 'getNetworkInterfaces' });
    }

    /**
     * 切换网卡
     */
    setInterface(iface) {
        this.send({ type: 'setNetworkInterface', interface: iface });
    }

    /**
     * 更新网络配置
     */
    updateNetworkConfig(config) {
        config.type = 'updateNetworkConfig';
        this.send(config);
    }

    /**
     * 连接到机器人
     */
    connectRobot(ip, iface) {
        this.send({ type: 'connect', ip: ip, interface: iface || 'auto' });
    }

    /**
     * 断开机器人
     */
    disconnectRobot() {
        this.send({ type: 'disconnect' });
    }

    /**
     * 轮询机器人连接状态
     */
    pollRobotStatus() {
        var self = this;
        fetch('/api/robot-status')
            .then(function(r) { return r.json(); })
            .then(function(status) {
                self._handleMessage(status);
            })
            .catch(function() {});
    }

    /**
     * 检测连通性
     */
    checkConnectivity(ip) {
        this.send({ type: 'checkConnectivity', ip: ip });
    }

    /**
     * 关闭连接
     */
    close() {
        if (this.sse) this.sse.close();
        if (this.reconnectTimer) clearTimeout(this.reconnectTimer);
        if (this.logPollTimer) clearInterval(this.logPollTimer);
    }
}

// 全局实例
window.bpxClient = new BpxWsClient();
