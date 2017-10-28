#include "utils.h"

const static char HEX[] = "0123456789ABCDEF";

QString Utils::toHex(const QByteArray& arr) {
    QString res;
    for(int i = 0, l = arr.size(); i < l; ++i) {
        uint8_t b = arr[i];
        res.append(HEX[(b & 0xF0) >> 4]);
        res.append(HEX[(b & 0x0F)]);
        if(i < l-1) res.append(' ');
    }
    return res;
}

static uint8_t fromHexDigit(QChar c) {
    for(uint8_t i = 0; i < sizeof(HEX); ++i)
        if(HEX[i] == c) return i;
    return sizeof(HEX);
}

QByteArray Utils::fromHex(const QString& s) {
    QString t = s.toUpper();
    t.replace("0X", "");
    t.replace(" ", "");
    t.replace(",", "");

    QByteArray res;
    for(QString::const_iterator i = t.begin(); i != t.end() && (i+1) != t.end(); ) {
        uint8_t d1 = fromHexDigit(*(i++));
        uint8_t d2 = fromHexDigit(*(i++));
        if(d1 >= 16 || d2 >= 16) throw "Wrong hex";
        res.append((uint8_t)((d1<<4) | d2));
    }
    return res;
}

uint32_t Utils::crc32(const QByteArray& arr) {
    uint32_t crc = 0xFFFFFFFF;
    for(int i = 0, l = arr.size(); i < l; ++i) {
        uint8_t b = arr[i];
        for(uint8_t pos = 0x01; pos; pos <<= 1) {
            uint32_t msb = crc & 0x80000000;
            if(b & pos) msb ^= 0x80000000;

            if(msb) crc = (crc << 1) ^ 0x4C11DB7;
            else    crc = (crc << 1);
        }
    }

    uint32_t r = 0;
    for(size_t i = 0; i < 32; ++i) {
        r <<= 1;
        r |= (crc & 0x01);
        crc >>= 1;
    }

    return r ^ 0xFFFFFFFF;
}
