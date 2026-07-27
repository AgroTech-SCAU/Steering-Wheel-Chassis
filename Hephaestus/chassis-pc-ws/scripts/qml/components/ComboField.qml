import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property string label: ""
    property string hint: ""
    property string configKey: ""
    property string fieldValue: ""
    property var values: []

    Layout.fillWidth: true
    implicitHeight: hint.length > 0 ? 70 : 52

    onFieldValueChanged: {
        if (!field.activeFocus && field.text !== root.fieldValue)
            field.text = root.fieldValue
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
            }
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
                onClicked: valuePopup.open()
            }
        }
    }

    Popup {
        id: valuePopup
        x: Math.max(0, Math.min(root.width - width,
                               menuButton.mapToItem(root, menuButton.width - width, 0).x))
        y: menuButton.mapToItem(root, 0, menuButton.height + 4).y
        width: Math.min(170, root.width)
        height: Math.min(valueList.contentHeight + 8, 218)
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

        contentItem: ListView {
            id: valueList
            clip: true
            model: root.values
            boundsBehavior: Flickable.StopAtBounds

            delegate: ItemDelegate {
                width: valueList.width
                height: 34
                hoverEnabled: true
                leftPadding: 10
                rightPadding: 10

                contentItem: Text {
                    text: String(modelData)
                    color: "#243442"
                    font.pixelSize: 13
                    verticalAlignment: Text.AlignVCenter
                    elide: Text.ElideRight
                }

                background: Rectangle {
                    radius: 6
                    color: parent.hovered || parent.highlighted ? "#EAF6FC" : "transparent"
                }

                onClicked: {
                    field.text = String(modelData)
                    backend.setConfigValue(root.configKey, String(modelData))
                    valuePopup.close()
                }
            }
        }
    }
}
