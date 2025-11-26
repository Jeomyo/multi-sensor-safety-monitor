import QtQuick
import QtCharts

Item {
    id: root
    width: 300
    height: 120

    property string title: ""
    property string unit: ""
    property real value: 0
    property color lineColor: "white"
    property var values: []

    onValueChanged: {
        if (value === undefined || value === null) return
        if (isNaN(value)) return

        values.push(Number(value))
        if (values.length > 50) values.shift()

        series.clear()
        for (let i = 0; i < values.length; i++) {
            if (isNaN(values[i])) continue
            series.append(i, values[i])
        }
    }


    Text {
        text: title + ": " + value + " " + unit
        color: "white"
        anchors.left: parent.left
    }

    ChartView {
        id: chart
        anchors.fill: parent
        backgroundColor: "transparent"
        legend.visible: false

        ValueAxis {
            id: yAxis
            min: 0
            max: 100
        }

        ValueAxis {
            id: xAxis
            min: 0
            max: 50
        }

        LineSeries {
            id: series
            color: lineColor
            axisX: xAxis     //<-- 반드시 필요!!
            axisY: yAxis     //<-- 반드시 필요!!
        }
    }
}
