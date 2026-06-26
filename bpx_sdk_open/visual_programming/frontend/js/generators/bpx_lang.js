/**
 * BPX 代码生成器
 * 将 Blockly 积木转换为 JSON 指令序列
 */

// 初始化代码生成器
if (typeof Blockly !== 'undefined' && Blockly.JavaScript) {

    // === 连接类 ===

    Blockly.JavaScript.forBlock['bpx_connect'] = function(block) {
        var ip = block.getFieldValue('ROBOT_IP');
        var iface = block.getFieldValue('INTERFACE');
        var code = '{"action":"connect","ip":"' + ip + '","interface":"' + iface + '"},\n';
        return code;
    };

    Blockly.JavaScript.forBlock['bpx_disconnect'] = function(block) {
        return '{"action":"disconnect"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_is_connected'] = function(block) {
        return ['__state__.connected', Blockly.JavaScript.ORDER_MEMBER];
    };

    // === 运动控制类 ===

    Blockly.JavaScript.forBlock['bpx_stand_up'] = function(block) {
        return '{"action":"standUp"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_sit_down'] = function(block) {
        return '{"action":"sitDown"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_damping'] = function(block) {
        return '{"action":"damping"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_upright'] = function(block) {
        return '{"action":"upright"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_set_velocity'] = function(block) {
        var x = Blockly.JavaScript.valueToCode(block, 'X', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        var y = Blockly.JavaScript.valueToCode(block, 'Y', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        var yaw = Blockly.JavaScript.valueToCode(block, 'YAW', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        return '{"action":"setVelocity","x":' + x + ',"y":' + y + ',"yaw":' + yaw + '},\n';
    };

    Blockly.JavaScript.forBlock['bpx_velocity_control_flag'] = function(block) {
        var enabled = block.getFieldValue('ENABLED') === 'true';
        return '{"action":"setVelocityControlFlag","enabled":' + enabled + '},\n';
    };

    Blockly.JavaScript.forBlock['bpx_gait_select'] = function(block) {
        var gait = block.getFieldValue('GAIT');
        return '{"action":"' + gait + '"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_zero_positions'] = function(block) {
        return '{"action":"zeroPositions"},\n';
    };

    // 前进/后退速度（仅修改 x 轴，不影响其他轴）
    Blockly.JavaScript.forBlock['bpx_set_forward'] = function(block) {
        var dir = block.getFieldValue('DIRECTION');
        var speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        var sign = (dir === 'backward') ? '-' : '';
        return '{"action":"setForwardVelocity","value":' + sign + speed + '},\n';
    };

    // 左右速度（仅修改 y 轴，不影响其他轴）
    Blockly.JavaScript.forBlock['bpx_set_lateral'] = function(block) {
        var dir = block.getFieldValue('DIRECTION');
        var speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        var sign = (dir === 'left') ? '' : '-';
        return '{"action":"setLateralVelocity","value":' + sign + speed + '},\n';
    };

    // 转向速度（仅修改 yaw 轴，不影响其他轴）
    Blockly.JavaScript.forBlock['bpx_set_turn'] = function(block) {
        var dir = block.getFieldValue('DIRECTION');
        var speed = Blockly.JavaScript.valueToCode(block, 'SPEED', Blockly.JavaScript.ORDER_ASSIGNMENT) || '0';
        var sign = (dir === 'left') ? '' : '-';
        return '{"action":"setTurnVelocity","value":' + sign + speed + '},\n';
    };

    // === 传感器类 ===

    Blockly.JavaScript.forBlock['bpx_battery_level'] = function(block) {
        return ['__state__.battery || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_imu_rpy'] = function(block) {
        var axis = block.getFieldValue('AXIS');
        var idx = { 'roll': 0, 'pitch': 1, 'yaw': 2 }[axis] || 0;
        return ['((__state__.imuRpy || [0,0,0])[' + idx + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_body_velocity'] = function(block) {
        var axis = block.getFieldValue('AXIS');
        var idx = { 'x': 0, 'y': 1, 'yaw': 2 }[axis] || 0;
        return ['((__state__.bodyVelocity || [0,0,0])[' + idx + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_motion_state'] = function(block) {
        return ['__state__.motionState || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_gait'] = function(block) {
        return ['__state__.gait || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_joint_position'] = function(block) {
        var joint = parseInt(block.getFieldValue('JOINT'));
        return ['((__state__.jointPos || [0,0,0,0,0,0,0,0,0,0,0,0])[' + joint + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_motor_temperature'] = function(block) {
        var joint = parseInt(block.getFieldValue('JOINT'));
        return ['((__state__.motorTemp || [0,0,0,0,0,0,0,0,0,0,0,0])[' + joint + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_battery_current'] = function(block) {
        return ['__state__.batteryCurrent || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_sub_gait'] = function(block) {
        return ['__state__.subGait || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_leg_odom'] = function(block) {
        var axis = block.getFieldValue('AXIS');
        var idx = { 'x': 0, 'y': 1, 'yaw': 2 }[axis] || 0;
        return ['((__state__.legOdom || [0,0,0])[' + idx + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_imu_quat'] = function(block) {
        var comp = parseInt(block.getFieldValue('COMP'));
        return ['((__state__.imuQuat || [0,0,0,1])[' + comp + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_imu_acc'] = function(block) {
        var axis = parseInt(block.getFieldValue('AXIS'));
        return ['((__state__.imuAcc || [0,0,0])[' + axis + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_imu_omega'] = function(block) {
        var axis = parseInt(block.getFieldValue('AXIS'));
        return ['((__state__.imuOmega || [0,0,0])[' + axis + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_driver_temperature'] = function(block) {
        var joint = parseInt(block.getFieldValue('JOINT'));
        return ['((__state__.driverTemp || [0,0,0,0,0,0,0,0,0,0,0,0])[' + joint + '])', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_last_motion_state'] = function(block) {
        return ['__state__.lastMotionState || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    Blockly.JavaScript.forBlock['bpx_last_gait'] = function(block) {
        return ['__state__.lastGait || 0', Blockly.JavaScript.ORDER_MEMBER];
    };

    // === 关节控制类 ===

    Blockly.JavaScript.forBlock['bpx_set_all_joints'] = function(block) {
        var abad = block.getFieldValue('ABAD') || '0';
        var hip = block.getFieldValue('HIP') || '0';
        var knee = block.getFieldValue('KNEE') || '0';
        var abadRad = (parseFloat(abad) * Math.PI / 180).toFixed(4);
        var hipRad = (parseFloat(hip) * Math.PI / 180).toFixed(4);
        var kneeRad = (parseFloat(knee) * Math.PI / 180).toFixed(4);
        return '{"action":"setJointPosition","position":[' +
            abadRad + ',' + hipRad + ',' + kneeRad + ',' +
            abadRad + ',' + hipRad + ',' + kneeRad + ',' +
            abadRad + ',' + hipRad + ',' + kneeRad + ',' +
            abadRad + ',' + hipRad + ',' + kneeRad + ']},\n';
    };

    Blockly.JavaScript.forBlock['bpx_set_leg_joint'] = function(block) {
        var leg = parseInt(block.getFieldValue('LEG'));
        var abad = block.getFieldValue('ABAD') || '0';
        var hip = block.getFieldValue('HIP') || '0';
        var knee = block.getFieldValue('KNEE') || '0';
        var abadRad = (parseFloat(abad) * Math.PI / 180).toFixed(4);
        var hipRad = (parseFloat(hip) * Math.PI / 180).toFixed(4);
        var kneeRad = (parseFloat(knee) * Math.PI / 180).toFixed(4);
        return '{"action":"setLegJoint","leg":' + leg + ',"abad":' + abadRad + ',"hip":' + hipRad + ',"knee":' + kneeRad + '},\n';
    };

    Blockly.JavaScript.forBlock['bpx_set_gains'] = function(block) {
        var kp = block.getFieldValue('KP') || '100';
        var kd = block.getFieldValue('KD') || '2';
        return '{"action":"setJointGains","kp":' + kp + ',"kd":' + kd + '},\n';
    };

    Blockly.JavaScript.forBlock['bpx_smooth_move'] = function(block) {
        var leg = block.getFieldValue('LEG');
        var abad = block.getFieldValue('ABAD') || '0';
        var hip = block.getFieldValue('HIP') || '0';
        var knee = block.getFieldValue('KNEE') || '0';
        var duration = block.getFieldValue('DURATION') || '2';
        var abadRad = (parseFloat(abad) * Math.PI / 180).toFixed(4);
        var hipRad = (parseFloat(hip) * Math.PI / 180).toFixed(4);
        var kneeRad = (parseFloat(knee) * Math.PI / 180).toFixed(4);

        if (leg === '-1') {
            return '{"action":"smoothMove","target":[' +
                abadRad + ',' + hipRad + ',' + kneeRad + ',' +
                abadRad + ',' + hipRad + ',' + kneeRad + ',' +
                abadRad + ',' + hipRad + ',' + kneeRad + ',' +
                abadRad + ',' + hipRad + ',' + kneeRad + '],"duration":' + duration + '},\n';
        } else {
            return '{"action":"smoothMoveSingleLeg","leg":' + leg +
                ',"abad":' + abadRad + ',"hip":' + hipRad + ',"knee":' + kneeRad +
                ',"duration":' + duration + '},\n';
        }
    };

    Blockly.JavaScript.forBlock['bpx_zero_joints'] = function(block) {
        return '{"action":"zeroJoints"},\n';
    };

    // === 流程控制类 ===

    Blockly.JavaScript.forBlock['bpx_wait'] = function(block) {
        var seconds = block.getFieldValue('SECONDS') || '1';
        return '{"action":"wait","seconds":' + seconds + '},\n';
    };

    Blockly.JavaScript.forBlock['bpx_repeat'] = function(block) {
        var times = block.getFieldValue('TIMES') || '3';
        var body = Blockly.JavaScript.statementToCode(block, 'BODY');
        var bodyProgram = parseStatementsToProgram(body);
        var code = '{"action":"loop","times":"' + escapeJsonString(String(times)) + '","body":' + JSON.stringify(bodyProgram) + '},\n';
        return code;
    };

    Blockly.JavaScript.forBlock['bpx_repeat_until'] = function(block) {
        var condition = Blockly.JavaScript.valueToCode(block, 'CONDITION', Blockly.JavaScript.ORDER_ASSIGNMENT) || 'true';
        var body = Blockly.JavaScript.statementToCode(block, 'BODY');
        var bodyProgram = parseStatementsToProgram(body);
        var code = '{"action":"repeatUntil","condition":"' + escapeJsonString(condition) + '","body":' + JSON.stringify(bodyProgram) + '},\n';
        return code;
    };

    Blockly.JavaScript.forBlock['bpx_if_else'] = function(block) {
        var condition = Blockly.JavaScript.valueToCode(block, 'CONDITION', Blockly.JavaScript.ORDER_ASSIGNMENT) || 'true';
        var thenBody = Blockly.JavaScript.statementToCode(block, 'THEN');
        var elseBody = Blockly.JavaScript.statementToCode(block, 'ELSE');
        var thenProgram = parseStatementsToProgram(thenBody);
        var elseProgram = parseStatementsToProgram(elseBody);
        var code = '{"action":"if","condition":"' + escapeJsonString(condition) + '","then":' + JSON.stringify(thenProgram);
        if (elseProgram.length > 0) {
            code += ',"else":' + JSON.stringify(elseProgram);
        }
        code += '},\n';
        return code;
    };

    Blockly.JavaScript.forBlock['bpx_emergency_stop'] = function(block) {
        return '{"action":"emergencyStop"},\n';
    };

    Blockly.JavaScript.forBlock['bpx_compare'] = function(block) {
        var op = block.getFieldValue('OP');
        var a = block.getFieldValue('A') || '0';
        var b = block.getFieldValue('B') || '0';
        var ops = { 'EQ': '==', 'NEQ': '!=', 'LT': '<', 'LTE': '<=', 'GT': '>', 'GTE': '>=' };
        var code = a + ' ' + (ops[op] || '==') + ' ' + b;
        return [code, Blockly.JavaScript.ORDER_RELATIONAL];
    };

    Blockly.JavaScript.forBlock['bpx_logic'] = function(block) {
        var op = block.getFieldValue('OP');
        var a = Blockly.JavaScript.valueToCode(block, 'A', Blockly.JavaScript.ORDER_LOGICAL_AND) || 'true';
        var b = Blockly.JavaScript.valueToCode(block, 'B', Blockly.JavaScript.ORDER_LOGICAL_AND) || 'true';
        if (op === 'AND') {
            return [a + ' && ' + b, Blockly.JavaScript.ORDER_LOGICAL_AND];
        } else {
            return [a + ' || ' + b, Blockly.JavaScript.ORDER_LOGICAL_OR];
        }
    };

    Blockly.JavaScript.forBlock['bpx_math'] = function(block) {
        var op = block.getFieldValue('OP');
        var a = block.getFieldValue('A') || '0';
        var b = block.getFieldValue('B') || '0';
        var ops = { 'ADD': '+', 'SUB': '-', 'MUL': '*', 'DIV': '/' };
        var code = a + ' ' + (ops[op] || '+') + ' ' + b;
        return [code, Blockly.JavaScript.ORDER_ADDITION];
    };
}

/**
 * 转义字符串以便安全地嵌入 JSON
 */
function escapeJsonString(str) {
    return str.replace(/\\/g, '\\\\').replace(/"/g, '\\"').replace(/\n/g, '\\n');
}

/**
 * 将生成器输出的语句代码解析为 JSON 指令数组
 * 支持解析嵌套的 JSON 对象（循环、条件等）
 */
function parseStatementsToProgram(code) {
    var program = [];
    var lines = code.split('\n');
    for (var i = 0; i < lines.length; i++) {
        var line = lines[i].trim();
        if (line.startsWith('{') && line.endsWith(',')) {
            line = line.slice(0, -1); // 移除末尾逗号
        }
        if (line.startsWith('{')) {
            try {
                program.push(JSON.parse(line));
            } catch (e) {
                console.warn('Failed to parse statement:', line, e);
            }
        }
    }
    return program;
}

/**
 * 安全地评估条件表达式
 * 将 __state__ 引用替换为当前机器人状态
 */
function evaluateCondition(conditionStr, robotState) {
    try {
        var __state__ = robotState || {};
        // 使用 Function 构造器安全评估
        var fn = new Function('__state__', 'return (' + conditionStr + ');');
        return !!fn(__state__);
    } catch (e) {
        console.warn('Condition evaluation failed:', conditionStr, e);
        return false;
    }
}

/**
 * 解析 times 值为数字
 */
function parseTimesValue(timesStr) {
    var n = parseInt(timesStr, 10);
    return (isNaN(n) || n < 1) ? 1 : Math.min(n, 10000); // 限制最大循环次数
}

/**
 * 展开流程控制指令为扁平的指令序列
 * - loop: 重复 N 次展开 body
 * - if: 根据当前状态评估条件，选择 then 或 else 分支
 * - repeatUntil: 最多展开为 maxIterations 次（安全限制）
 */
function expandProgram(program, robotState) {
    var expanded = [];
    for (var i = 0; i < program.length; i++) {
        var cmd = program[i];
        if (!cmd || !cmd.action) continue;

        if (cmd.action === 'loop') {
            var times = parseTimesValue(cmd.times);
            var body = cmd.body || [];
            for (var t = 0; t < times; t++) {
                var bodyExpanded = expandProgram(body, robotState);
                expanded = expanded.concat(bodyExpanded);
            }
        } else if (cmd.action === 'if') {
            var condResult = evaluateCondition(cmd.condition, robotState);
            if (condResult) {
                var thenExpanded = expandProgram(cmd.then || [], robotState);
                expanded = expanded.concat(thenExpanded);
            } else if (cmd['else'] && cmd['else'].length > 0) {
                var elseExpanded = expandProgram(cmd['else'], robotState);
                expanded = expanded.concat(elseExpanded);
            }
        } else if (cmd.action === 'repeatUntil') {
            // repeatUntil 在前端展开为最多 N 次迭代
            // 每次迭代前重新评估条件（使用当前状态）
            var maxIter = 100; // 安全限制
            var body = cmd.body || [];
            for (var iter = 0; iter < maxIter; iter++) {
                if (evaluateCondition(cmd.condition, robotState)) break;
                var bodyExpanded = expandProgram(body, robotState);
                expanded = expanded.concat(bodyExpanded);
            }
        } else {
            // 普通指令，直接添加
            expanded.push(cmd);
        }
    }
    return expanded;
}

/**
 * 将 Blockly 工作区转换为扁平的 JSON 指令数组
 * 1. 生成结构化 JSON（含嵌套的流程控制）
 * 2. 展开流程控制为扁平指令序列
 */
function generateProgram(workspace) {
    // 生成 JavaScript 代码
    var code = Blockly.JavaScript.workspaceToCode(workspace);

    // 解析为结构化 JSON 指令
    var structured = parseStatementsToProgram(code);

    // 获取当前机器人状态用于条件评估
    var robotState = window.__bpxRobotState || {};

    // 展开流程控制为扁平指令序列
    return expandProgram(structured, robotState);
}
