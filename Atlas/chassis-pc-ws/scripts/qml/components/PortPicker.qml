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
                onClicked: portPopup.open()
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

    Popup {
        id: portPopup
        x: Math.max(0, Math.min(root.width - width,
                               menuButton.mapToItem(root, menuButton.width - width, 0).x))
        y: menuButton.mapToItem(root, 0, menuButton.height + 4).y
        width: Math.min(300, root.width)
        height: root.portModel.length > 0
                ? Math.min(portList.contentHeight + 8, 228)
                : 46
        padding: 4
        modal: false
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        background: Rectangle {
            color: "#FFFFFF"
            border.color: "#D9E2EC"
            border.width: 1
            radius: 9
        }

        contentItem: Item {
            implicitWidth: 292
            implicitHeight: portPopup.height - portPopup.topPadding - portPopup.bottomPadding

            ListView {
                id: portList
                anchors.fill: parent
                clip: true
                model: root.portModel
                boundsBehavior: Flickable.StopAtBounds
                spacing: 0

                delegate: ItemDelegate {
                    width: portList.width
                    height: 50
                    hoverEnabled: true
                    leftPadding: 10
                    rightPadding: 10
                    topPadding: 5
                    bottomPadding: 5

                    contentItem: Column {
                        spacing: 2
                        Text {
                            width: parent.width
                            text: String(modelData)
                            color: "#243442"
                            font.pixelSize: 13
                            font.weight: Font.DemiBold
                            elide: Text.ElideRight
                        }
                        Text {
                            width: parent.width
                            text: backend.portDescription(String(modelData))
                            color: "#718092"
                            font.pixelSize: 11
                            elide: Text.ElideRight
                        }
                    }

                    background: Rectangle {
                        radius: 6
                        color: parent.hovered || parent.highlighted ? "#EAF6FC" : "transparent"
                    }

                    onClicked: {
                        field.text = String(modelData)
                        backend.setConfigValue(root.configKey, String(modelData))
                        portPopup.close()
                    }
                }
            }

            Text {
                anchors.centerIn: parent
                visible: root.portModel.length === 0
                text: "未扫描到串口，可直接手动输入"
                color: "#718092"
                font.pixelSize: 12
            }
        }
    }
}
