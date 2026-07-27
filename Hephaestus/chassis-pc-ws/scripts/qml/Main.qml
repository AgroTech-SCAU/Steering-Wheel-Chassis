import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Window
import "components"

ApplicationWindow {
    id: window
    width: 1440
    height: 900
    minimumWidth: 1180
    minimumHeight: 720
    visible: true
    title: "主从臂遥操作控制台"
    color: "#F4F7FB"

    property int currentPage: 0
    property string dialogTitle: ""
    property string dialogMessage: ""
    property bool dialogCritical: false

    font.family: Qt.platform.os === "windows" ? "Microsoft YaHei UI" : "Noto Sans CJK SC"

    // Light application palette. Custom controls use the same neutral-blue scheme.
    palette.window: "#F4F7FB"
    palette.windowText: "#182433"
    palette.base: "#FFFFFF"
    palette.alternateBase: "#F7F9FC"
    palette.text: "#243442"
    palette.button: "#FFFFFF"
    palette.buttonText: "#243442"
    palette.highlight: "#178BC3"
    palette.highlightedText: "#FFFFFF"
    palette.placeholderText: "#8795A4"
    palette.mid: "#CBD5DF"
    palette.light: "#FFFFFF"
    palette.dark: "#AAB8C4"

    function statusTone() {
        if (backend.running)
            return "success"
        if (backend.status.indexOf("失败") >= 0)
            return "danger"
        if (backend.status.indexOf("连接") >= 0 || backend.status.indexOf("启动") >= 0 || backend.status.indexOf("停止") >= 0)
            return "warning"
        return "neutral"
    }

    function levelColor(level) {
        if (level === "ERROR") return "#C53E4E"
        if (level === "WARN") return "#A96E00"
        if (level === "MCU") return "#087FB8"
        if (level === "DATA") return "#168650"
        if (level === "DEBUG") return "#7C8995"
        return "#43515F"
    }

    Rectangle {
        anchors.fill: parent
        color: "#F4F7FB"

        ColumnLayout {
            anchors.fill: parent
            spacing: 0

            // Header
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 70
                color: "#FFFFFF"
                border.color: "#D9E2EC"
                border.width: 1

                RowLayout {
                    anchors.fill: parent
                    anchors.leftMargin: 22
                    anchors.rightMargin: 22
                    spacing: 14

                    Rectangle {
                        width: 38
                        height: 38
                        radius: 11
                        color: "#E6F4FB"
                        border.color: "#A8D6EB"
                        Text {
                            anchors.centerIn: parent
                            text: "⇄"
                            color: "#1687C0"
                            font.pixelSize: 22
                            font.weight: Font.Bold
                        }
                    }

                    ColumnLayout {
                        spacing: 1
                        Text {
                            text: "主从臂遥操作控制台"
                            color: "#182433"
                            font.pixelSize: 18
                            font.weight: Font.DemiBold
                        }
                        Text {
                            text: "Dynamixel Leader  ·  MCU Follower  ·  Binary Protocol"
                            color: "#6B7A89"
                            font.pixelSize: 11
                        }
                    }

                    Item { Layout.fillWidth: true }

                    StatusPill {
                        text: backend.configState
                        tone: backend.configDirty ? "warning" : backend.configLocked ? "neutral" : "success"
                    }
                    StatusPill {
                        text: backend.status
                        tone: window.statusTone()
                    }

                    AppButton {
                        text: "扫描串口"
                        compact: true
                        variant: "ghost"
                        enabled: !backend.configLocked
                        onClicked: backend.scanPorts()
                    }

                    Rectangle {
                        width: 1
                        height: 28
                        color: "#D9E2EC"
                        Layout.leftMargin: 4
                        Layout.rightMargin: 4
                    }

                    Text {
                        text: backend.platformName
                        color: "#6B7A89"
                        font.pixelSize: 12
                    }
                }
            }

            SplitView {
                id: mainSplit
                Layout.fillWidth: true
                Layout.fillHeight: true
                orientation: Qt.Vertical

                Rectangle {
                    SplitView.fillWidth: true
                    SplitView.fillHeight: true
                    SplitView.minimumHeight: 430
                    color: "#F4F7FB"

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 12

                        // Sidebar
                        Rectangle {
                            Layout.preferredWidth: 216
                            Layout.fillHeight: true
                            radius: 14
                            color: "#FFFFFF"
                            border.color: "#D9E2EC"

                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 12
                                spacing: 8

                                Text {
                                    text: "配置分类"
                                    color: "#708090"
                                    font.pixelSize: 11
                                    font.weight: Font.DemiBold
                                    Layout.leftMargin: 9
                                    Layout.topMargin: 4
                                    Layout.bottomMargin: 3
                                }

                                NavButton {
                                    Layout.fillWidth: true
                                    text: "连接与频率"
                                    symbol: "⌁"
                                    selected: window.currentPage === 0
                                    onClicked: window.currentPage = 0
                                }
                                NavButton {
                                    Layout.fillWidth: true
                                    text: "关节映射"
                                    symbol: "◎"
                                    selected: window.currentPage === 1
                                    onClicked: window.currentPage = 1
                                }
                                NavButton {
                                    Layout.fillWidth: true
                                    text: "夹爪与开关"
                                    symbol: "◇"
                                    selected: window.currentPage === 2
                                    onClicked: window.currentPage = 2
                                }
                                NavButton {
                                    Layout.fillWidth: true
                                    text: "日志与高级"
                                    symbol: "⚙"
                                    selected: window.currentPage === 3
                                    onClicked: window.currentPage = 3
                                }

                                Item { Layout.fillHeight: true }

                                Rectangle {
                                    Layout.fillWidth: true
                                    implicitHeight: 92
                                    radius: 11
                                    color: "#F8FAFC"
                                    border.color: "#D9E2EC"
                                    Column {
                                        anchors.fill: parent
                                        anchors.margins: 12
                                        spacing: 7
                                        Text {
                                            text: "当前连接"
                                            color: "#718092"
                                            font.pixelSize: 11
                                        }
                                        Text {
                                            width: parent.width
                                            text: "主臂  " + (String(backend.config.leader_port).length ? backend.config.leader_port : "未选择")
                                            color: "#2E3B49"
                                            font.pixelSize: 12
                                            elide: Text.ElideMiddle
                                        }
                                        Text {
                                            width: parent.width
                                            text: "从臂  " + (String(backend.config.mcu_port).length ? backend.config.mcu_port : "未选择")
                                            color: "#2E3B49"
                                            font.pixelSize: 12
                                            elide: Text.ElideMiddle
                                        }
                                    }
                                }

                                AppButton {
                                    Layout.fillWidth: true
                                    text: "验证并应用配置"
                                    variant: "secondary"
                                    enabled: !backend.configLocked
                                    onClicked: backend.applyConfig()
                                }
                                AppButton {
                                    Layout.fillWidth: true
                                    text: backend.running ? "运行中" : "开始运行"
                                    variant: "primary"
                                    enabled: backend.canRun
                                    onClicked: backend.startRun()
                                }
                                AppButton {
                                    Layout.fillWidth: true
                                    text: "停止并释放串口"
                                    variant: "danger"
                                    enabled: backend.canStop
                                    onClicked: backend.stopRun()
                                }
                            }
                        }

                        // Configuration area
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            radius: 14
                            color: "#FFFFFF"
                            border.color: "#D9E2EC"

                            ColumnLayout {
                                anchors.fill: parent
                                anchors.margins: 18
                                spacing: 12

                                RowLayout {
                                    Layout.fillWidth: true
                                    ColumnLayout {
                                        spacing: 3
                                        Text {
                                            text: ["连接与运行频率", "主臂关节映射", "夹爪与末端开关", "日志与高级选项"][window.currentPage]
                                            color: "#182433"
                                            font.pixelSize: 20
                                            font.weight: Font.DemiBold
                                        }
                                        Text {
                                            text: [
                                                "选择主从设备端口，并设置通信节拍与写入超时",
                                                "定义 q0~q4 的 Dynamixel ID、方向和零位转换",
                                                "配置 ID7 夹爪映射、末端开关阈值和启动行为",
                                                "管理 MCU 日志、协议参数与本地配置文件"
                                            ][window.currentPage]
                                            color: "#718092"
                                            font.pixelSize: 12
                                        }
                                    }
                                    Item { Layout.fillWidth: true }
                                    StatusPill {
                                        text: backend.configLocked ? "只读" : "可编辑"
                                        tone: backend.configLocked ? "neutral" : "success"
                                    }
                                }

                                ScrollView {
                                    id: configScroll
                                    Layout.fillWidth: true
                                    Layout.fillHeight: true
                                    clip: true
                                    ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

                                    StackLayout {
                                        width: configScroll.availableWidth
                                        currentIndex: window.currentPage

                                        // Page 0: connection
                                        ColumnLayout {
                                            spacing: 12
                                            enabled: !backend.configLocked

                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 12

                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "主臂 Dynamixel"
                                                    subtitle: "读取 q0~q4 与夹爪位置"
                                                    PortPicker {
                                                        label: "串口端口"
                                                        configKey: "leader_port"
                                                        fieldValue: String(backend.config.leader_port)
                                                        portModel: backend.ports
                                                    }
                                                    ComboField {
                                                        label: "通信波特率"
                                                        hint: "常用 1 Mbps"
                                                        configKey: "leader_baud"
                                                        fieldValue: String(backend.config.leader_baud)
                                                        values: backend.baudRates
                                                    }
                                                }

                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "从臂 MCU"
                                                    subtitle: "发送关节帧并接收调试日志"
                                                    PortPicker {
                                                        label: "串口端口"
                                                        configKey: "mcu_port"
                                                        fieldValue: String(backend.config.mcu_port)
                                                        portModel: backend.ports
                                                    }
                                                    ComboField {
                                                        label: "通信波特率"
                                                        hint: "默认 921600"
                                                        configKey: "mcu_baud"
                                                        fieldValue: String(backend.config.mcu_baud)
                                                        values: backend.baudRates
                                                    }
                                                }
                                            }

                                            SectionCard {
                                                Layout.fillWidth: true
                                                title: "运行节拍"
                                                subtitle: "所有高频工作均在独立 QThread 中执行，不阻塞 QML 界面"
                                                GridLayout {
                                                    Layout.fillWidth: true
                                                    columns: 2
                                                    columnSpacing: 16
                                                    rowSpacing: 8
                                                    FormField {
                                                        label: "关节发送频率"
                                                        hint: "Hz · 建议 30~100"
                                                        configKey: "master_send_freq_hz"
                                                        fieldValue: String(backend.config.master_send_freq_hz)
                                                    }
                                                    FormField {
                                                        label: "PC 心跳频率"
                                                        hint: "Hz · 0 为关闭"
                                                        configKey: "heartbeat_freq_hz"
                                                        fieldValue: String(backend.config.heartbeat_freq_hz)
                                                    }
                                                    FormField {
                                                        label: "状态日志频率"
                                                        hint: "Hz · 0 为关闭"
                                                        configKey: "print_freq_hz"
                                                        fieldValue: String(backend.config.print_freq_hz)
                                                    }
                                                    FormField {
                                                        label: "串口写超时"
                                                        hint: "秒"
                                                        configKey: "write_timeout_s"
                                                        fieldValue: String(backend.config.write_timeout_s)
                                                    }
                                                }
                                            }
                                        }

                                        // Page 1: joints
                                        ColumnLayout {
                                            spacing: 12
                                            enabled: !backend.configLocked

                                            SectionCard {
                                                Layout.fillWidth: true
                                                title: "q0 ~ q4 映射"
                                                subtitle: "读取原始 Present_Position 后执行 q = sign × q_raw + offset"
                                                FormField {
                                                    label: "Dynamixel ID"
                                                    hint: "5 个整数，例如 1,2,3,4,5"
                                                    configKey: "joint_ids"
                                                    fieldValue: String(backend.config.joint_ids)
                                                }
                                                FormField {
                                                    label: "方向系数"
                                                    hint: "5 个非零数"
                                                    configKey: "joint_signs"
                                                    fieldValue: String(backend.config.joint_signs)
                                                }
                                                FormField {
                                                    label: "零位偏置"
                                                    hint: "rad，5 个数"
                                                    configKey: "joint_offsets_rad"
                                                    fieldValue: String(backend.config.joint_offsets_rad)
                                                }
                                                FormField {
                                                    label: "每圈编码"
                                                    hint: "ticks / rev"
                                                    configKey: "ticks_per_rev"
                                                    fieldValue: String(backend.config.ticks_per_rev)
                                                }
                                            }

                                            SectionCard {
                                                Layout.fillWidth: true
                                                title: "位置解释方式"
                                                subtitle: "多圈模式或扩展位置模式下，应根据实际舵机模式调整"
                                                ToggleRow {
                                                    label: "对 Present_Position 做一圈取模"
                                                    description: "保持旧 DMLeader 的单圈角度转换行为"
                                                    configKey: "wrap_ticks"
                                                    checkedValue: Boolean(backend.config.wrap_ticks)
                                                }
                                                ToggleRow {
                                                    label: "按 int32 有符号数解释位置"
                                                    description: "普通 XL330 单圈读取通常无需开启"
                                                    configKey: "signed_position"
                                                    checkedValue: Boolean(backend.config.signed_position)
                                                }
                                            }
                                        }

                                        // Page 2: gripper
                                        ColumnLayout {
                                            spacing: 12
                                            enabled: !backend.configLocked

                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 12
                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "夹爪位置映射"
                                                    subtitle: "将 ID7 原始位置归一化为末端开关状态"
                                                    FormField {
                                                        label: "夹爪 Dynamixel ID"
                                                        configKey: "gripper_id"
                                                        fieldValue: String(backend.config.gripper_id)
                                                    }
                                                    FormField {
                                                        label: "打开位置"
                                                        hint: "tick"
                                                        configKey: "gripper_open_pos"
                                                        fieldValue: String(backend.config.gripper_open_pos)
                                                    }
                                                    FormField {
                                                        label: "闭合位置"
                                                        hint: "tick"
                                                        configKey: "gripper_closed_pos"
                                                        fieldValue: String(backend.config.gripper_closed_pos)
                                                    }
                                                    FormField {
                                                        label: "末端开关阈值"
                                                        hint: "norm > threshold 输出 1"
                                                        configKey: "end_switch_threshold"
                                                        fieldValue: String(backend.config.end_switch_threshold)
                                                    }
                                                }

                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "启动行为"
                                                    subtitle: "启动主臂通信时写入 ID7 控制表"
                                                    ToggleRow {
                                                        label: "配置夹爪模式并开启力矩"
                                                        description: "Current-based Position Control Mode"
                                                        configKey: "enable_gripper_torque_on_start"
                                                        checkedValue: Boolean(backend.config.enable_gripper_torque_on_start)
                                                    }
                                                    FormField {
                                                        label: "Current_Limit"
                                                        configKey: "gripper_current_limit"
                                                        fieldValue: String(backend.config.gripper_current_limit)
                                                    }
                                                    FormField {
                                                        label: "启动目标位置"
                                                        hint: "tick"
                                                        configKey: "gripper_goal_position_on_start"
                                                        fieldValue: String(backend.config.gripper_goal_position_on_start)
                                                    }
                                                }
                                            }
                                        }

                                        // Page 3: advanced
                                        ColumnLayout {
                                            spacing: 12
                                            enabled: !backend.configLocked

                                            RowLayout {
                                                Layout.fillWidth: true
                                                spacing: 12
                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "MCU 日志"
                                                    subtitle: "MCU → PC 方向按换行解析文本日志"
                                                    ToggleRow {
                                                        label: "读取 MCU 文本日志"
                                                        description: "与二进制下行帧共用同一串口"
                                                        configKey: "read_mcu_log"
                                                        checkedValue: Boolean(backend.config.read_mcu_log)
                                                    }
                                                    FormField {
                                                        label: "文本编码"
                                                        configKey: "mcu_log_encoding"
                                                        fieldValue: String(backend.config.mcu_log_encoding)
                                                    }
                                                    FormField {
                                                        label: "显示前缀"
                                                        configKey: "mcu_log_prefix"
                                                        fieldValue: String(backend.config.mcu_log_prefix)
                                                    }
                                                    FormField {
                                                        label: "接收缓存上限"
                                                        hint: "bytes"
                                                        configKey: "mcu_log_buffer_limit"
                                                        fieldValue: String(backend.config.mcu_log_buffer_limit)
                                                    }
                                                }

                                                SectionCard {
                                                    Layout.fillWidth: true
                                                    title: "协议与调试"
                                                    subtitle: "固定帧格式必须与 MCU 端协议保持一致"
                                                    FormField {
                                                        label: "CRC 名称"
                                                        configKey: "crc_name"
                                                        fieldValue: String(backend.config.crc_name)
                                                    }
                                                    ToggleRow {
                                                        label: "独占打开 MCU 串口"
                                                        description: backend.platformName === "Windows" ? "Windows 串口默认独占，此项不参与打开参数" : "防止 minicom / VOFA 同时占用"
                                                        configKey: "exclusive"
                                                        checkedValue: Boolean(backend.config.exclusive)
                                                        enabled: backend.platformName !== "Windows"
                                                    }
                                                    ToggleRow {
                                                        label: "Dry-run 模式"
                                                        description: "只读取主臂并打包，不打开从臂 MCU"
                                                        configKey: "dry_run"
                                                        checkedValue: Boolean(backend.config.dry_run)
                                                    }
                                                }
                                            }

                                            SectionCard {
                                                Layout.fillWidth: true
                                                title: "配置文件"
                                                subtitle: "保存的是当前编辑值；载入后仍需重新验证并应用"
                                                RowLayout {
                                                    Layout.fillWidth: true
                                                    spacing: 10
                                                    AppButton {
                                                        text: "载入 JSON"
                                                        compact: true
                                                        onClicked: backend.loadConfig()
                                                    }
                                                    AppButton {
                                                        text: "保存 JSON"
                                                        compact: true
                                                        onClicked: backend.saveConfig()
                                                    }
                                                    AppButton {
                                                        text: "恢复默认值"
                                                        compact: true
                                                        variant: "ghost"
                                                        onClicked: backend.resetDefaults()
                                                    }
                                                    Item { Layout.fillWidth: true }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        // Live monitor
                        Rectangle {
                            Layout.preferredWidth: 342
                            Layout.fillHeight: true
                            radius: 14
                            color: "#FFFFFF"
                            border.color: "#D9E2EC"

                            ScrollView {
                                id: liveMonitorScroll
                                anchors.fill: parent
                                anchors.margins: 14
                                clip: true
                                contentWidth: availableWidth
                                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                                ScrollBar.vertical.policy: ScrollBar.AsNeeded

                                ColumnLayout {
                                    width: liveMonitorScroll.availableWidth
                                    spacing: 12

                                    RowLayout {
                                        Layout.fillWidth: true
                                        Text {
                                            text: "实时状态"
                                            color: "#182433"
                                            font.pixelSize: 17
                                            font.weight: Font.DemiBold
                                        }
                                        Item { Layout.fillWidth: true }
                                        StatusPill {
                                            text: backend.running ? "ONLINE" : "OFFLINE"
                                            tone: backend.running ? "success" : "neutral"
                                        }
                                    }

                                    SectionCard {
                                        Layout.fillWidth: true
                                        title: "关节角"
                                        subtitle: "单位 rad，界面刷新上限 20 Hz"
                                        Repeater {
                                            model: 5
                                            Item {
                                                id: jointRow
                                                Layout.fillWidth: true
                                                implicitHeight: 42
                                                property real jointValue: Number(backend.stats.q_rad[index] || 0)
                                                ColumnLayout {
                                                    anchors.fill: parent
                                                    spacing: 5
                                                    RowLayout {
                                                        Layout.fillWidth: true
                                                        Text {
                                                            text: "q" + index
                                                            color: "#536273"
                                                            font.pixelSize: 12
                                                            font.weight: Font.DemiBold
                                                        }
                                                        Item { Layout.fillWidth: true }
                                                        Text {
                                                            text: jointRow.jointValue.toFixed(4)
                                                            color: "#2C3A47"
                                                            font.pixelSize: 12
                                                            font.family: "monospace"
                                                        }
                                                    }
                                                    Rectangle {
                                                        Layout.fillWidth: true
                                                        height: 5
                                                        radius: 3
                                                        color: "#E7EDF3"
                                                        Rectangle {
                                                            width: parent.width * Math.max(0.02, Math.min(1.0, (jointRow.jointValue + 6.283) / 12.566))
                                                            height: parent.height
                                                            radius: 3
                                                            color: "#178BC3"
                                                            Behavior on width { NumberAnimation { duration: 80 } }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }

                                    SectionCard {
                                        Layout.fillWidth: true
                                        title: "末端开关"
                                        subtitle: "夹爪位置归一化结果"
                                        RowLayout {
                                            Layout.fillWidth: true
                                            Rectangle {
                                                width: 52
                                                height: 52
                                                radius: 26
                                                color: Number(backend.stats.end_switch) === 1 ? "#E8F7EF" : "#EEF2F6"
                                                border.color: Number(backend.stats.end_switch) === 1 ? "#65B989" : "#CBD5DF"
                                                Text {
                                                    anchors.centerIn: parent
                                                    text: Number(backend.stats.end_switch) === 1 ? "ON" : "OFF"
                                                    color: Number(backend.stats.end_switch) === 1 ? "#1F7A4D" : "#667684"
                                                    font.pixelSize: 12
                                                    font.weight: Font.Bold
                                                }
                                            }
                                            ColumnLayout {
                                                Layout.fillWidth: true
                                                Text {
                                                    text: "raw  " + Number(backend.stats.gripper_raw)
                                                    color: "#344454"
                                                    font.pixelSize: 12
                                                    font.family: "monospace"
                                                }
                                                Text {
                                                    text: "norm " + Number(backend.stats.gripper_norm).toFixed(3)
                                                    color: "#718092"
                                                    font.pixelSize: 12
                                                    font.family: "monospace"
                                                }
                                            }
                                        }
                                    }

                                    GridLayout {
                                        Layout.fillWidth: true
                                        columns: 2
                                        columnSpacing: 9
                                        rowSpacing: 9
                                        MetricCard {
                                            Layout.fillWidth: true
                                            label: "关节帧"
                                            value: String(backend.stats.frames_sent)
                                            accent: "#168BC1"
                                        }
                                        MetricCard {
                                            Layout.fillWidth: true
                                            label: "心跳帧"
                                            value: String(backend.stats.heartbeat_sent)
                                            accent: "#7464D9"
                                        }
                                        MetricCard {
                                            Layout.fillWidth: true
                                            label: "接收字节"
                                            value: String(backend.stats.rx_bytes)
                                            unit: "B"
                                            accent: "#2FA66D"
                                        }
                                        MetricCard {
                                            Layout.fillWidth: true
                                            label: "错误计数"
                                            value: String(backend.stats.errors)
                                            accent: Number(backend.stats.errors) > 0 ? "#D84F60" : "#7B8996"
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // Logs
                Rectangle {
                    SplitView.fillWidth: true
                    SplitView.preferredHeight: 260
                    SplitView.minimumHeight: 160
                    color: "#F2F5F8"
                    border.color: "#D9E2EC"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 10
                            Text {
                                text: "运行日志"
                                color: "#182433"
                                font.pixelSize: 15
                                font.weight: Font.DemiBold
                            }
                            Rectangle {
                                width: 1
                                height: 20
                                color: "#D5DEE6"
                            }
                            Text {
                                text: logModel.count + " 条"
                                color: "#718092"
                                font.pixelSize: 11
                            }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: "筛选"
                                color: "#718092"
                                font.pixelSize: 11
                            }
                            ComboBox {
                                id: logFilter
                                model: ["全部", "INFO", "WARN", "ERROR", "MCU", "DATA", "DEBUG"]
                                implicitWidth: 92
                                implicitHeight: 32
                                font.pixelSize: 12
                                contentItem: Text {
                                    text: logFilter.displayText
                                    color: "#344454"
                                    font: logFilter.font
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: 10
                                }
                                background: Rectangle {
                                    radius: 8
                                    color: "#FFFFFF"
                                    border.color: "#CAD5DF"
                                }
                            }
                            CheckBox {
                                id: autoScroll
                                checked: true
                                text: "自动滚动"
                                font.pixelSize: 11
                                contentItem: Text {
                                    text: autoScroll.text
                                    color: "#5F6F7E"
                                    font: autoScroll.font
                                    leftPadding: autoScroll.indicator.width + 7
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }
                            AppButton {
                                text: "清空"
                                compact: true
                                variant: "ghost"
                                onClicked: backend.clearLogs()
                            }
                        }

                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            radius: 10
                            color: "#FFFFFF"
                            border.color: "#EEF2F6"

                            ListView {
                                id: logList
                                anchors.fill: parent
                                anchors.margins: 8
                                clip: true
                                spacing: 1
                                model: logModel
                                boundsBehavior: Flickable.StopAtBounds
                                ScrollBar.vertical: ScrollBar { }

                                delegate: Item {
                                    width: logList.width
                                    property bool allowed: logFilter.currentText === "全部" || logFilter.currentText === level
                                    height: allowed ? Math.max(24, logText.implicitHeight + 6) : 0
                                    visible: allowed

                                    Rectangle {
                                        anchors.fill: parent
                                        radius: 5
                                        color: mouseArea.containsMouse ? "#F5F8FA" : "transparent"
                                    }
                                    RowLayout {
                                        anchors.fill: parent
                                        anchors.leftMargin: 6
                                        anchors.rightMargin: 6
                                        spacing: 10
                                        Text {
                                            text: timestamp
                                            color: "#7B8996"
                                            font.pixelSize: 11
                                            font.family: "monospace"
                                            Layout.alignment: Qt.AlignTop
                                            Layout.topMargin: 3
                                        }
                                        Text {
                                            text: level
                                            color: window.levelColor(level)
                                            font.pixelSize: 11
                                            font.weight: Font.DemiBold
                                            font.family: "monospace"
                                            Layout.preferredWidth: 48
                                            Layout.alignment: Qt.AlignTop
                                            Layout.topMargin: 3
                                        }
                                        TextEdit {
                                            id: logText
                                            Layout.fillWidth: true
                                            text: message
                                            color: level === "DEBUG" ? "#7B8996" : "#3E4C59"
                                            font.pixelSize: 11
                                            font.family: "monospace"
                                            readOnly: true
                                            selectByMouse: true
                                            wrapMode: TextEdit.Wrap
                                            textFormat: TextEdit.PlainText
                                        }
                                    }
                                    MouseArea {
                                        id: mouseArea
                                        anchors.fill: parent
                                        hoverEnabled: true
                                        acceptedButtons: Qt.NoButton
                                    }
                                }

                                Text {
                                    anchors.centerIn: parent
                                    visible: logModel.count === 0
                                    text: "暂无日志"
                                    color: "#9AA7B3"
                                    font.pixelSize: 13
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Dialog {
        id: messageDialog
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 440
        title: window.dialogTitle
        standardButtons: Dialog.Ok

        background: Rectangle {
            radius: 14
            color: "#FFFFFF"
            border.color: window.dialogCritical ? "#D88B96" : "#BFD5E2"
        }
        header: Rectangle {
            implicitHeight: 56
            color: "transparent"
            Text {
                anchors.left: parent.left
                anchors.leftMargin: 18
                anchors.verticalCenter: parent.verticalCenter
                text: window.dialogTitle
                color: window.dialogCritical ? "#A33745" : "#182433"
                font.pixelSize: 16
                font.weight: Font.DemiBold
            }
        }
        contentItem: Text {
            text: window.dialogMessage
            color: "#43515F"
            font.pixelSize: 13
            wrapMode: Text.WordWrap
            leftPadding: 18
            rightPadding: 18
            topPadding: 6
            bottomPadding: 12
        }
    }

    Connections {
        target: backend
        function onDialogRequested(title, message, critical) {
            window.dialogTitle = title
            window.dialogMessage = message
            window.dialogCritical = critical
            messageDialog.open()
        }
    }

    Connections {
        target: logModel
        function onCountChanged() {
            if (autoScroll.checked)
                Qt.callLater(function() { logList.positionViewAtEnd() })
        }
    }
}
