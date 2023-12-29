#ifndef _LITT_SERIALIZATION_H_
#define _LITT_SERIALIZATION_H_

#include <stdint.h>

namespace litt {

// Note: No endianness conversion performed here.

template <typename T> bool serialize(const T &e, uint8_t *&buf, size_t &size) {
  size_t sz = sizeof(T);
  if (sz >= size)
    return false;
  memcpy(buf, &e, sz);
  buf += sz;
  size -= sz;
  return true;
}

template <typename T> bool deserialize(T &e, const uint8_t *&buf, size_t &size) {
  size_t sz = sizeof(T);
  if (sz >= size)
    return false;
  memcpy(&e, buf, sz);
  buf += sz;
  size -= sz;
  return true;
}
} // namespace litt

#endif // _LITT_SERIALIZATION_H_
