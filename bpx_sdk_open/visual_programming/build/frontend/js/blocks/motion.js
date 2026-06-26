/**
 * BPX 运动控制积木定义
 */

// 站立
Blockly.Blocks['bpx_stand_up'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 站立');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('让机器狗站立');
    }
};

// 坐下
Blockly.Blocks['bpx_sit_down'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 坐下');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('让机器狗坐下');
    }
};

// 进入阻尼模式
Blockly.Blocks['bpx_damping'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 进入阻尼模式');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('让机器狗进入关节阻尼模式（放松关节）');
    }
};

// 直立等待
Blockly.Blocks['bpx_upright'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 直立等待');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('让机器狗进入直立等待模式');
    }
};

// 设置速度（组合）
Blockly.Blocks['bpx_set_velocity'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 设置速度');
        this.appendValueInput('X')
            .setCheck('Number')
            .appendField('  前进 (m/s):');
        this.appendValueInput('Y')
            .setCheck('Number')
            .appendField('  左右 (m/s):');
        this.appendValueInput('YAW')
            .setCheck('Number')
            .appendField('  转向 (rad/s):');
        this.setInputsInline(false);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('设置机器狗的运动速度（同时设置前后、左右、转向）');
    }
};

// 设置前进速度
Blockly.Blocks['bpx_set_forward'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕')
            .appendField(new Blockly.FieldDropdown([
                ['前进', 'forward'],
                ['后退', 'backward']
            ]), 'DIRECTION');
        this.appendValueInput('SPEED')
            .setCheck('Number')
            .appendField('速度 (m/s):');
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('设置前进或后退速度');
    }
};

// 设置左右速度
Blockly.Blocks['bpx_set_lateral'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕')
            .appendField(new Blockly.FieldDropdown([
                ['左移', 'left'],
                ['右移', 'right']
            ]), 'DIRECTION');
        this.appendValueInput('SPEED')
            .setCheck('Number')
            .appendField('速度 (m/s):');
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('设置左右移动速度');
    }
};

// 设置转向速度
Blockly.Blocks['bpx_set_turn'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕')
            .appendField(new Blockly.FieldDropdown([
                ['左转', 'left'],
                ['右转', 'right']
            ]), 'DIRECTION');
        this.appendValueInput('SPEED')
            .setCheck('Number')
            .appendField('转向速度 (rad/s):');
        this.setInputsInline(true);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('设置转向速度');
    }
};

// 启用/禁用速度控制
Blockly.Blocks['bpx_velocity_control_flag'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕')
            .appendField(new Blockly.FieldDropdown([
                ['启用', 'true'],
                ['禁用', 'false']
            ]), 'ENABLED')
            .appendField('速度控制');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('启用或禁用速度控制模式');
    }
};

// 步态选择
Blockly.Blocks['bpx_gait_select'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 切换步态')
            .appendField(new Blockly.FieldDropdown([
                ['行走 Walk', 'walk'],
                ['跑步 Running', 'running'],
                ['左空翻 Left Flip', 'leftFlip'],
                ['右空翻 Right Flip', 'rightFlip'],
                ['双足 Bipedal', 'bipedal'],
                ['反双足 Inv Bipedal', 'invBipedal'],
                ['弹跳 Pronk', 'pronk'],
                ['溜步 Pace', 'pace'],
                ['跳跃 Bound', 'bound']
            ]), 'GAIT');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('切换机器狗的步态模式');
    }
};

// 零位标记
Blockly.Blocks['bpx_zero_positions'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🐕 零位标记');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(120);
        this.setTooltip('发送零位标记指令');
    }
};
