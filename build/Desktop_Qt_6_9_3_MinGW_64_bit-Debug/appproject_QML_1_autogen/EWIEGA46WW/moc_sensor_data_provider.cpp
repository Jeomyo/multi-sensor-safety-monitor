/****************************************************************************
** Meta object code from reading C++ file 'sensor_data_provider.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.9.3)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../sensor_data_provider.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'sensor_data_provider.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.9.3. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN18SensorDataProviderE_t {};
} // unnamed namespace

template <> constexpr inline auto SensorDataProvider::qt_create_metaobjectdata<qt_meta_tag_ZN18SensorDataProviderE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "SensorDataProvider",
        "temperatureChanged",
        "",
        "humidityChanged",
        "dustChanged",
        "mapUpdated",
        "mapPoints",
        "obstaclesUpdated",
        "obstacles",
        "workersUpdated",
        "workers",
        "workStartModeChanged",
        "value",
        "workEndModeChanged",
        "systemMsgOneReceived",
        "systemMsgTwoReceived",
        "handleMessage",
        "message",
        "QMqttTopicName",
        "topic",
        "connectToBroker",
        "host",
        "port",
        "temperature",
        "humidity",
        "dust"
    };

    QtMocHelpers::UintData qt_methods {
        // Signal 'temperatureChanged'
        QtMocHelpers::SignalData<void()>(1, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'humidityChanged'
        QtMocHelpers::SignalData<void()>(3, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'dustChanged'
        QtMocHelpers::SignalData<void()>(4, 2, QMC::AccessPublic, QMetaType::Void),
        // Signal 'mapUpdated'
        QtMocHelpers::SignalData<void(QJsonArray)>(5, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QJsonArray, 6 },
        }}),
        // Signal 'obstaclesUpdated'
        QtMocHelpers::SignalData<void(QJsonArray)>(7, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QJsonArray, 8 },
        }}),
        // Signal 'workersUpdated'
        QtMocHelpers::SignalData<void(QJsonArray)>(9, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QJsonArray, 10 },
        }}),
        // Signal 'workStartModeChanged'
        QtMocHelpers::SignalData<void(int)>(11, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 12 },
        }}),
        // Signal 'workEndModeChanged'
        QtMocHelpers::SignalData<void(int)>(13, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 12 },
        }}),
        // Signal 'systemMsgOneReceived'
        QtMocHelpers::SignalData<void(int)>(14, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 12 },
        }}),
        // Signal 'systemMsgTwoReceived'
        QtMocHelpers::SignalData<void(int)>(15, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::Int, 12 },
        }}),
        // Slot 'handleMessage'
        QtMocHelpers::SlotData<void(const QByteArray &, const QMqttTopicName &)>(16, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { QMetaType::QByteArray, 17 }, { 0x80000000 | 18, 19 },
        }}),
        // Method 'connectToBroker'
        QtMocHelpers::MethodData<void(const QString &, int)>(20, 2, QMC::AccessPublic, QMetaType::Void, {{
            { QMetaType::QString, 21 }, { QMetaType::Int, 22 },
        }}),
    };
    QtMocHelpers::UintData qt_properties {
        // property 'temperature'
        QtMocHelpers::PropertyData<double>(23, QMetaType::Double, QMC::DefaultPropertyFlags, 0),
        // property 'humidity'
        QtMocHelpers::PropertyData<double>(24, QMetaType::Double, QMC::DefaultPropertyFlags, 1),
        // property 'dust'
        QtMocHelpers::PropertyData<double>(25, QMetaType::Double, QMC::DefaultPropertyFlags, 2),
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<SensorDataProvider, qt_meta_tag_ZN18SensorDataProviderE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject SensorDataProvider::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN18SensorDataProviderE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN18SensorDataProviderE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN18SensorDataProviderE_t>.metaTypes,
    nullptr
} };

void SensorDataProvider::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<SensorDataProvider *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->temperatureChanged(); break;
        case 1: _t->humidityChanged(); break;
        case 2: _t->dustChanged(); break;
        case 3: _t->mapUpdated((*reinterpret_cast< std::add_pointer_t<QJsonArray>>(_a[1]))); break;
        case 4: _t->obstaclesUpdated((*reinterpret_cast< std::add_pointer_t<QJsonArray>>(_a[1]))); break;
        case 5: _t->workersUpdated((*reinterpret_cast< std::add_pointer_t<QJsonArray>>(_a[1]))); break;
        case 6: _t->workStartModeChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 7: _t->workEndModeChanged((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 8: _t->systemMsgOneReceived((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 9: _t->systemMsgTwoReceived((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 10: _t->handleMessage((*reinterpret_cast< std::add_pointer_t<QByteArray>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<QMqttTopicName>>(_a[2]))); break;
        case 11: _t->connectToBroker((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<int>>(_a[2]))); break;
        default: ;
        }
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
        case 10:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType(); break;
            case 1:
                *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType::fromType< QMqttTopicName >(); break;
            }
            break;
        }
    }
    if (_c == QMetaObject::IndexOfMethod) {
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)()>(_a, &SensorDataProvider::temperatureChanged, 0))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)()>(_a, &SensorDataProvider::humidityChanged, 1))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)()>(_a, &SensorDataProvider::dustChanged, 2))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(QJsonArray )>(_a, &SensorDataProvider::mapUpdated, 3))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(QJsonArray )>(_a, &SensorDataProvider::obstaclesUpdated, 4))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(QJsonArray )>(_a, &SensorDataProvider::workersUpdated, 5))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(int )>(_a, &SensorDataProvider::workStartModeChanged, 6))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(int )>(_a, &SensorDataProvider::workEndModeChanged, 7))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(int )>(_a, &SensorDataProvider::systemMsgOneReceived, 8))
            return;
        if (QtMocHelpers::indexOfMethod<void (SensorDataProvider::*)(int )>(_a, &SensorDataProvider::systemMsgTwoReceived, 9))
            return;
    }
    if (_c == QMetaObject::ReadProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast<double*>(_v) = _t->temperature(); break;
        case 1: *reinterpret_cast<double*>(_v) = _t->humidity(); break;
        case 2: *reinterpret_cast<double*>(_v) = _t->dust(); break;
        default: break;
        }
    }
}

const QMetaObject *SensorDataProvider::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SensorDataProvider::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN18SensorDataProviderE_t>.strings))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SensorDataProvider::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    }
    if (_c == QMetaObject::ReadProperty || _c == QMetaObject::WriteProperty
            || _c == QMetaObject::ResetProperty || _c == QMetaObject::BindableProperty
            || _c == QMetaObject::RegisterPropertyMetaType) {
        qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    }
    return _id;
}

// SIGNAL 0
void SensorDataProvider::temperatureChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void SensorDataProvider::humidityChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void SensorDataProvider::dustChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void SensorDataProvider::mapUpdated(QJsonArray _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 3, nullptr, _t1);
}

// SIGNAL 4
void SensorDataProvider::obstaclesUpdated(QJsonArray _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 4, nullptr, _t1);
}

// SIGNAL 5
void SensorDataProvider::workersUpdated(QJsonArray _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 5, nullptr, _t1);
}

// SIGNAL 6
void SensorDataProvider::workStartModeChanged(int _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 6, nullptr, _t1);
}

// SIGNAL 7
void SensorDataProvider::workEndModeChanged(int _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 7, nullptr, _t1);
}

// SIGNAL 8
void SensorDataProvider::systemMsgOneReceived(int _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 8, nullptr, _t1);
}

// SIGNAL 9
void SensorDataProvider::systemMsgTwoReceived(int _t1)
{
    QMetaObject::activate<void>(this, &staticMetaObject, 9, nullptr, _t1);
}
QT_WARNING_POP
