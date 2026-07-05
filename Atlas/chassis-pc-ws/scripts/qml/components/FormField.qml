import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property string label: ""
    property string hint: ""
    property string configKey: ""
    property string fieldValue: ""
    property bool numeric: false

    Layout.fillWidth: true
    implicitHeight: hint.length > 0 ? 70 : 52

    onFieldValueChanged: {
        if (!input.activeFocus && input.text !== root.fieldValue)
            input.text = root.fieldValue
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 6

        RowLayout {
            Layout.fillWidth: true
            Text {
                text: root.label
                color: "#344454"
                font.pixelSize: 13
                Layout.fillWidth: true
            }
            Text {
                visible: root.hint.length > 0
                text: root.hint
                color: "#718092"
                font.pixelSize: 11
                horizontalAlignment: Text.AlignRight
            }
        }

        TextField {
            id: input
            Layout.fillWidth: true
            implicitHeight: 38
            text: root.fieldValue
            enabled: root.enabled
            color: enabled ? "#1F2D3A" : "#9AA7B3"
            selectionColor: "#278BB8"
            selectedTextColor: "#FFFFFF"
            placeholderTextColor: "#8A98A5"
            font.pixelSize: 13
            leftPadding: 12
            rightPadding: 12
            background: Rectangle {
                radius: 9
                color: input.enabled ? (input.activeFocus ? "#F2FAFE" : "#FFFFFF") : "#F1F4F7"
                border.width: 1
                border.color: input.activeFocus ? "#178BC3" : "#CAD5DF"
            }
            onEditingFinished: backend.setConfigValue(root.configKey, text)
        }
    }
}
