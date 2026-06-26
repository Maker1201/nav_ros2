/**
 * BPX 传感器读取积木定义
 */

// 电池电量
Blockly.Blocks['bpx_battery_level'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 电池电量 (%)');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取电池电量百分比');
    }
};

// IMU 姿态
Blockly.Blocks['bpx_imu_rpy'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 IMU')
            .appendField(new Blockly.FieldDropdown([
                ['Roll', 'roll'],
                ['Pitch', 'pitch'],
                ['Yaw', 'yaw']
            ]), 'AXIS');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取 IMU 姿态角（度）');
    }
};

// 身体速度
Blockly.Blocks['bpx_body_velocity'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 身体速度')
            .appendField(new Blockly.FieldDropdown([
                ['前进 (x)', 'x'],
                ['左右 (y)', 'y'],
                ['转向 (yaw)', 'yaw']
            ]), 'AXIS');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取身体速度');
    }
};

// 当前运动状态
Blockly.Blocks['bpx_motion_state'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 当前运动状态');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取当前运动状态码');
    }
};

// 当前步态
Blockly.Blocks['bpx_gait'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 当前步态');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取当前步态编号');
    }
};

// 关节角度
Blockly.Blocks['bpx_joint_position'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 关节角度')
            .appendField(new Blockly.FieldDropdown([
                ['左前-外展', '0'],
                ['左前-髋', '1'],
                ['左前-膝', '2'],
                ['右前-外展', '3'],
                ['右前-髋', '4'],
                ['右前-膝', '5'],
                ['左后-外展', '6'],
                ['左后-髋', '7'],
                ['左后-膝', '8'],
                ['右后-外展', '9'],
                ['右后-髋', '10'],
                ['右后-膝', '11']
            ]), 'JOINT');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取指定关节的角度（弧度）');
    }
};

// 电机温度
Blockly.Blocks['bpx_motor_temperature'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 电机温度')
            .appendField(new Blockly.FieldDropdown([
                ['左前-外展', '0'],
                ['左前-髋', '1'],
                ['左前-膝', '2'],
                ['右前-外展', '3'],
                ['右前-髋', '4'],
                ['右前-膝', '5'],
                ['左后-外展', '6'],
                ['左后-髋', '7'],
                ['左后-膝', '8'],
                ['右后-外展', '9'],
                ['右后-髋', '10'],
                ['右后-膝', '11']
            ]), 'JOINT');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取指定关节的电机温度（°C）');
    }
};

// 电池电流
Blockly.Blocks['bpx_battery_current'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 电池电流 (A)');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取电池电流');
    }
};

// 子步态
Blockly.Blocks['bpx_sub_gait'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 子步态');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取当前子步态编号');
    }
};

// 里程计
Blockly.Blocks['bpx_leg_odom'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 里程计')
            .appendField(new Blockly.FieldDropdown([
                ['X', 'x'],
                ['Y', 'y'],
                ['Yaw', 'yaw']
            ]), 'AXIS');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取腿部里程计数据');
    }
};

// IMU 四元数
Blockly.Blocks['bpx_imu_quat'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 IMU 四元数')
            .appendField(new Blockly.FieldDropdown([
                ['X', '0'],
                ['Y', '1'],
                ['Z', '2'],
                ['W', '3']
            ]), 'COMP');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取 IMU 四元数分量');
    }
};

// IMU 加速度
Blockly.Blocks['bpx_imu_acc'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 IMU 加速度')
            .appendField(new Blockly.FieldDropdown([
                ['X', '0'],
                ['Y', '1'],
                ['Z', '2']
            ]), 'AXIS');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取 IMU 线性加速度 (m/s²)');
    }
};

// IMU 角速度
Blockly.Blocks['bpx_imu_omega'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 IMU 角速度')
            .appendField(new Blockly.FieldDropdown([
                ['X', '0'],
                ['Y', '1'],
                ['Z', '2']
            ]), 'AXIS');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取 IMU 角速度 (rad/s)');
    }
};

// 驱动器温度
Blockly.Blocks['bpx_driver_temperature'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 驱动器温度')
            .appendField(new Blockly.FieldDropdown([
                ['左前-外展', '0'],
                ['左前-髋', '1'],
                ['左前-膝', '2'],
                ['右前-外展', '3'],
                ['右前-髋', '4'],
                ['右前-膝', '5'],
                ['左后-外展', '6'],
                ['左后-髋', '7'],
                ['左后-膝', '8'],
                ['右后-外展', '9'],
                ['右后-髋', '10'],
                ['右后-膝', '11']
            ]), 'JOINT');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取指定关节的驱动器温度（°C）');
    }
};

// 上一运动状态
Blockly.Blocks['bpx_last_motion_state'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 上一运动状态');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取上一运动状态码');
    }
};

// 上一步态
Blockly.Blocks['bpx_last_gait'] = {
    init: function() {
        this.appendDummyInput()
            .appendField('📊 上一步态');
        this.setOutput(true, 'Number');
        this.setColour(30);
        this.setTooltip('获取上一步态编号');
    }
};
