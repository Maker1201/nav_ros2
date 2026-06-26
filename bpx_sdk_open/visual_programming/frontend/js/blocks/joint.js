/**
 * BPX 关节控制积木定义
 */

// 数字输入验证器（与 flow.js 共用，如果 flow.js 已加载则复用）
if (typeof numberValidator === 'undefined') {
    function numberValidator(newValue) {
        var n = parseFloat(newValue);
        if (isNaN(n)) return null;
        return String(n);
    }
}

// 设置全部腿关节
Blockly.Blocks['bpx_set_all_joints'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🦿 设置全部腿关节');
        this.appendDummyInput()
            .appendField('  外展 (°):')
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'ABAD');
        this.appendDummyInput()
            .appendField('  髋关节 (°):')
            .appendField(new Blockly.FieldTextInput('45', numberValidator), 'HIP');
        this.appendDummyInput()
            .appendField('  膝关节 (°):')
            .appendField(new Blockly.FieldTextInput('-90', numberValidator), 'KNEE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('设置全部四条腿的关节角度（度）');
    }
};

// 设置单条腿关节
Blockly.Blocks['bpx_set_leg_joint'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🦿 设置')
            .appendField(new Blockly.FieldDropdown([
                ['左前', '0'],
                ['右前', '1'],
                ['左后', '2'],
                ['右后', '3']
            ]), 'LEG')
            .appendField('腿关节');
        this.appendDummyInput()
            .appendField('  外展 (°):')
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'ABAD');
        this.appendDummyInput()
            .appendField('  髋关节 (°):')
            .appendField(new Blockly.FieldTextInput('45', numberValidator), 'HIP');
        this.appendDummyInput()
            .appendField('  膝关节 (°):')
            .appendField(new Blockly.FieldTextInput('-90', numberValidator), 'KNEE');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('设置指定腿的关节角度（度）');
    }
};

// 设置控制增益
Blockly.Blocks['bpx_set_gains'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🦿 设置控制增益');
        this.appendDummyInput()
            .appendField('  Kp:')
            .appendField(new Blockly.FieldTextInput('100', numberValidator), 'KP');
        this.appendDummyInput()
            .appendField('  Kd:')
            .appendField(new Blockly.FieldTextInput('2', numberValidator), 'KD');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('设置关节 PD 控制器的 Kp 和 Kd 增益');
    }
};

// 平滑移动到目标位置
Blockly.Blocks['bpx_smooth_move'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🦿 平滑移动到');
        this.appendDummyInput()
            .appendField('  腿:')
            .appendField(new Blockly.FieldDropdown([
                ['全部', '-1'],
                ['左前', '0'],
                ['右前', '1'],
                ['左后', '2'],
                ['右后', '3']
            ]), 'LEG');
        this.appendDummyInput()
            .appendField('  外展 (°):')
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'ABAD');
        this.appendDummyInput()
            .appendField('  髋 (°):')
            .appendField(new Blockly.FieldTextInput('45', numberValidator), 'HIP');
        this.appendDummyInput()
            .appendField('  膝 (°):')
            .appendField(new Blockly.FieldTextInput('-90', numberValidator), 'KNEE');
        this.appendDummyInput()
            .appendField('  时长 (秒):')
            .appendField(new Blockly.FieldTextInput('2', numberValidator), 'DURATION');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('平滑移动关节到目标位置');
    }
};

// 关节归零
Blockly.Blocks['bpx_zero_joints'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🦿 关节归零');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(270);
        this.setTooltip('发送零力矩指令，关节自由活动');
    }
};
