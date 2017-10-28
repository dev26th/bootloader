#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <QString>
#include <QByteArray>

class Utils
{
public:
    static QString toHex(const QByteArray& arr);
    static QByteArray fromHex(const QString& s);
    static uint32_t crc32(const QByteArray& arr);
};

#endif // UTILS_H
