else if (topicName == "sensor/esp32/dht11") {
        QString utf8Msg = QString::fromUtf8(message);
        QJsonDocument doc = QJsonDocument::fromJson(utf8Msg.toUtf8());

        if (!doc.isObject()) {
            qDebug() << "❌ dht11 JSON 파싱 실패:" << utf8Msg;
            return;
        }

        QJsonObject obj = doc.object();

        int temp = obj.value("temperature").toInt();
        int hum  = obj.value("humidity").toInt();
        int ts   = obj.value("timestamp").toInt();

        qDebug() << "🌡 DHT11 → temp:" << temp << " hum:" << hum << " ts:" << ts;

        emit dht11Updated(temp, hum, ts);
    }
    else if (topicName == "sensor/esp32/pms7003") {
        QString utf8Msg = QString::fromUtf8(message);
        QJsonDocument doc = QJsonDocument::fromJson(utf8Msg.toUtf8());

        if (!doc.isObject()) {
            qDebug() << "❌ PMS7003 JSON 파싱 실패:" << utf8Msg;
            return;
        }

        QJsonObject obj = doc.object();

        int pm1  = obj.value("pm1_0").toInt();
        int pm25 = obj.value("pm2_5").toInt();
        int pm10 = obj.value("pm10").toInt();
        int ts   = obj.value("timestamp").toInt();

        qDebug() << "🌫 PMS7003 → pm1.0:" << pm1
                 << " pm2.5:" << pm25
                 << " pm10:" << pm10
                 << " ts:" << ts;

        emit pmsUpdated(pm1, pm25, pm10, ts);
    }
