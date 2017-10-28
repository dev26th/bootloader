#include "crypto.h"

#include <assert.h>
#include "tiny-AES-c/aes.h"

#ifdef __linux__
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <unistd.h>
#endif

#ifdef _WIN32
#   include <Windows.h>
#   include <wincrypt.h>
#endif

// ====================================================================================================================
QByteArray Crypto::random(size_t len) {
    QByteArray res;
    res.resize(len);
#ifdef __linux__
    int fd = open("/dev/urandom", O_RDONLY);
    if(fd != -1) {
        ssize_t n = read(fd, res.data(), len);
        if(n != (ssize_t)len) res.clear();

        close(fd);
    }
#elif _WIN32
    HCRYPTPROV prov;
    if(CryptAcquireContext(&prov, NULL, NULL, PROV_RSA_FULL, 0)) {
      if(!CryptGenRandom(prov, len, (BYTE *)res.data()))
        res.clear();

      (void)CryptReleaseContext(prov, 0);
    }
#else
#   error "Unknown platform"
#endif
    return res;
}

QByteArray Crypto::encrypt(QByteArray arr, QByteArray key, QByteArray iv) {
    assert(arr.size() % 16 == 0);
    assert(key.size() == 16);
    assert(iv.size() == 16);

    QByteArray res;
    res.resize(arr.size());
    AES_CBC_encrypt_buffer((uint8_t*)res.data(), (uint8_t*)arr.data(), arr.size(), (const uint8_t*)key.data(), (const uint8_t*)iv.data());

    return res;
}
