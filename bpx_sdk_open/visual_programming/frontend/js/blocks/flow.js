/**
 * BPX 流程控制积木定义
 */

// 数字输入验证器
function numberValidator(newValue) {
    var n = parseFloat(newValue);
    if (isNaN(n)) return null; // 拒绝无效输入
    return String(n);
}

// 等待
Blockly.Blocks['bpx_wait'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('⏱️ 等待')
            .appendField(new Blockly.FieldTextInput('1', numberValidator), 'SECONDS')
            .appendField('秒');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('等待指定的秒数');
    }
};

// 重复 N 次
Blockly.Blocks['bpx_repeat'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🔁 重复')
            .appendField(new Blockly.FieldTextInput('3', numberValidator), 'TIMES')
            .appendField('次');
        this.appendStatementInput('BODY')
            .appendField('执行');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('重复执行内部积木 N 次');
    }
};

// 重复直到条件
Blockly.Blocks['bpx_repeat_until'] = {
    init: function() {
        this.appendValueInput('CONDITION')
            .setCheck('Boolean')
            .appendField('🔁 重复直到');
        this.appendStatementInput('BODY')
            .appendField('执行');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('重复执行直到条件为真');
    }
};

// 如果...那么...否则
Blockly.Blocks['bpx_if_else'] = {
    init: function() {
        this.appendValueInput('CONDITION')
            .setCheck('Boolean')
            .appendField('❓ 如果');
        this.appendStatementInput('THEN')
            .appendField('那么');
        this.appendStatementInput('ELSE')
            .appendField('否则');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(45);
        this.setTooltip('根据条件执行不同的积木');
    }
};

// 紧急停止
Blockly.Blocks['bpx_emergency_stop'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('🔴 紧急停止');
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
        this.setColour(0);
        this.setTooltip('立即停止所有动作，进入阻尼模式');
    }
};

// 比较运算符
Blockly.Blocks['bpx_compare'] = {
    init: function() {
        this.appendDummyInput()
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'A');
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([
                ['=', 'EQ'],
                ['≠', 'NEQ'],
                ['<', 'LT'],
                ['≤', 'LTE'],
                ['>', 'GT'],
                ['≥', 'GTE']
            ]), 'OP');
        this.appendDummyInput()
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'B');
        this.setInputsInline(true);
        this.setOutput(true, 'Boolean');
        this.setColour(45);
        this.setTooltip('比较两个数值');
    }
};

// 逻辑运算
Blockly.Blocks['bpx_logic'] = {
    init: function() {
        this.appendValueInput('A')
            .setCheck('Boolean');
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([
                ['并且', 'AND'],
                ['或者', 'OR']
            ]), 'OP');
        this.appendValueInput('B')
            .setCheck('Boolean');
        this.setInputsInline(true);
        this.setOutput(true, 'Boolean');
        this.setColour(45);
        this.setTooltip('逻辑运算');
    }
};

// 数学运算
Blockly.Blocks['bpx_math'] = {
    init: function() {
        this.appendDummyInput()
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'A');
        this.appendDummyInput()
            .appendField(new Blockly.FieldDropdown([
                ['+', 'ADD'],
                ['-', 'SUB'],
                ['×', 'MUL'],
                ['÷', 'DIV']
            ]), 'OP');
        this.appendDummyInput()
            .appendField(new Blockly.FieldTextInput('0', numberValidator), 'B');
        this.setInputsInline(true);
        this.setOutput(true, 'Number');
        this.setColour(45);
        this.setTooltip('数学运算');
    }
};
