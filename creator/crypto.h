#ifndef CRYPTO_H
#define CRYPTO_H

#include <QByteArray>

class Crypto
{
public:
    static const size_t KEY_LEN = 16;
    static const size_t IV_LEN = KEY_LEN;

public:
    static QByteArray random(size_t len);
    static QByteArray encrypt(QByteArray arr, QByteArray key, QByteArray iv);
};

#endif // CRYPTO_H
