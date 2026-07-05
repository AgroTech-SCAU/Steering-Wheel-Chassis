import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property string label: ""
    property string description: ""
    property string configKey: ""
    property bool checkedValue: false

    Layout.fillWidth: true
    implicitHeight: description.length > 0 ? 58 : 42

    onCheckedValueChanged: {
        if (toggle.checked !== root.checkedValue)
            toggle.checked = root.checkedValue
    }

    RowLayout {
        anchors.fill: parent
        spacing: 12
        ColumnLayout {
            Layout.fillWidth: true
            spacing: 3
            Text {
                text: root.label
                color: root.enabled ? "#344454" : "#9AA7B3"
                font.pixelSize: 13
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
            }
            Text {
                visible: root.description.length > 0
                text: root.description
                color: "#718092"
                font.pixelSize: 11
                Layout.fillWidth: true
                wrapMode: Text.WordWrap
            }
        }
        Switch {
            id: toggle
            checked: root.checkedValue
            enabled: root.enabled
            onToggled: backend.setConfigValue(root.configKey, checked)
            indicator: Rectangle {
                implicitWidth: 42
                implicitHeight: 23
                radius: 12
                color: toggle.checked ? "#168BC1" : "#D5DEE6"
                border.color: toggle.checked ? "#168BC1" : "#BBC7D1"
                Rectangle {
                    width: 17
                    height: 17
                    radius: 9
                    x: toggle.checked ? parent.width - width - 3 : 3
                    anchors.verticalCenter: parent.verticalCenter
                    color: toggle.enabled ? "#FFFFFF" : "#B3BEC8"
                    Behavior on x { NumberAnimation { duration: 120 } }
                }
            }
        }
    }
}
