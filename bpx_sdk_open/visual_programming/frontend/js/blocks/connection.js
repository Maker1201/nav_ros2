/**
 * BPX 连接类积木定义
 */

// 全局缓存的网卡列表
window.__bpxInterfaces = null;

/**
 * 获取网卡下拉列表选项
 * 如果已缓存则返回缓存，否则返回默认选项
 */
function getInterfaceOptions() {
    var options = [['🔄 自动检测', 'auto']];
    if (window.__bpxInterfaces && Array.isArray(window.__bpxInterfaces)) {
        window.__bpxInterfaces.forEach(function(iface) {
            var icon = iface.type === 'ethernet' ? '🔌' :
                       iface.type === 'wireless' ? '📶' :
                       iface.type === 'usb' ? '🔗' : '❓';
            var subnet = iface.sameSubnet ? ' ✓' : '';
            options.push([icon + ' ' + iface.name + ' (' + iface.ip + ')' + subnet, iface.name]);
        });
    } else {
        // 默认选项（未从服务器获取时）
        options.push(['🔌 eth0', 'eth0']);
        options.push(['📶 wlan0', 'wlan0']);
        options.push(['🔗 usb0', 'usb0']);
    }
    return options;
}

/**
 * 从服务器获取网卡列表并更新缓存
 */
function fetchInterfacesForBlock() {
    if (typeof bpxClient !== 'undefined') {
        fetch('/api/interfaces')
            .then(function(r) { return r.json(); })
            .then(function(data) {
                if (Array.isArray(data)) {
                    window.__bpxInterfaces = data;
                }
            })
            .catch(function() {});
    }
}

// 连接到机器人
Blockly.Blocks['bpx_connect'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🔌 连接到机器人');
        this.appendDummyInput()
            .appendField('  IP:')
            .appendField(new Blockly.FieldTextInput('10.21.20.1'), 'ROBOT_IP');
        this.appendDummyInput()
            .appendField('  网卡:')
            .appendField(new Blockly.FieldDropdown(function() {
                return getInterfaceOptions();
            }), 'INTERFACE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('连接到机器狗，指定 IP 地址和网络接口');
    }
};

// 页面加载时获取网卡列表
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', fetchInterfacesForBlock);
} else {
    fetchInterfacesForBlock();
}

// 断开连接
Blockly.Blocks['bpx_disconnect'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🔌 断开连接');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(210);
        this.setTooltip('断开与机器狗的连接');
    }
};

// 机器人已连接？（布尔值）
Blockly.Blocks['bpx_is_connected'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📡 机器人已连接？');
        this.setOutput(true, 'Boolean');
        this.setColour(210);
        this.setTooltip('返回机器人是否已连接');
    }
};
