import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property string label: ""
    property string configKey: ""
    property string fieldValue: ""
    property var portModel: []

    Layout.fillWidth: true
    implicitHeight: 73

    onFieldValueChanged: {
        if (!field.activeFocus && field.text !== root.fieldValue)
            field.text = root.fieldValue
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 6
        Text {
            text: root.label
            color: "#344454"
            font.pixelSize: 13
        }
        RowLayout {
            Layout.fillWidth: true
            spacing: 8
            TextField {
                id: field
                Layout.fillWidth: true
                implicitHeight: 38
                text: root.fieldValue
                enabled: root.enabled
                color: enabled ? "#1F2D3A" : "#9AA7B3"
                font.pixelSize: 13
                leftPadding: 12
                rightPadding: 12
                background: Rectangle {
                    radius: 9
                    color: field.enabled ? (field.activeFocus ? "#F2FAFE" : "#FFFFFF") : "#F1F4F7"
                    border.width: 1
                    border.color: field.activeFocus ? "#178BC3" : "#CAD5DF"
                }
                onEditingFinished: backend.setConfigValue(root.configKey, text)
            }
            ToolButton {
                id: menuButton
                implicitWidth: 40
                implicitHeight: 38
                enabled: root.enabled
                text: "⌄"
                font.pixelSize: 18
                contentItem: Text {
                    text: menuButton.text
                    color: menuButton.enabled ? "#4A5A68" : "#A5AFB8"
                    font: menuButton.font
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle {
                    radius: 9
                    color: menuButton.hovered ? "#EDF4F8" : "#F8FAFC"
                    border.color: "#CAD5DF"
                }
                onClicked: portMenu.open()
                Menu {
                    id: portMenu
                    y: menuButton.height + 4
                    width: 330
                    background: Rectangle {
                        color: "#FFFFFF"
                        border.color: "#D9E2EC"
                        radius: 10
                    }
                    Repeater {
                        model: root.portModel
                        MenuItem {
                            width: portMenu.width
                            height: 48
                            contentItem: Column {
                                spacing: 2
                                Text {
                                    text: modelData
                                    color: "#2C3A47"
                                    font.pixelSize: 13
                                    font.weight: Font.DemiBold
                                }
                                Text {
                                    text: backend.portDescription(modelData)
                                    color: "#718092"
                                    font.pixelSize: 11
                                    elide: Text.ElideRight
                                    width: portMenu.width - 30
                                }
                            }
                            background: Rectangle {
                                radius: 7
                                color: parent.highlighted ? "#EAF6FC" : "transparent"
                            }
                            onTriggered: {
                                field.text = modelData
                                backend.setConfigValue(root.configKey, modelData)
                            }
                        }
                    }
                    MenuItem {
                        visible: root.portModel.length === 0
                        enabled: false
                        text: "未扫描到串口，可直接手动输入"
                    }
                }
            }
        }
        Text {
            text: backend.portDescription(field.text)
            color: "#718092"
            font.pixelSize: 11
            elide: Text.ElideRight
            Layout.fillWidth: true
        }
    }
}
