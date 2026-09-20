#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

namespace diagnostic {
inline bool read_request(const uint8_t *d, size_t n) {
  if (!n) return false;
  switch (d[0]) {
    case 0x03: case 0x07: case 0x0a: return n == 1;
    case 0x01: return n == 2 && (d[1] == 0 || d[1] == 1);
    case 0x02: return n == 3 && d[2] == 0 &&
      (d[1] == 0 || d[1] == 2 || d[1] == 4 || d[1] == 5 || d[1] == 0x0c || d[1] == 0x0d || d[1] == 0x0f);
    case 0x09: return n == 2 && d[1] == 2;
    case 0x10: return n == 2 && (d[1] == 1 || d[1] == 3);
    case 0x19: return (n == 3 && (d[1] == 1 || d[1] == 2) && d[2] == 0xff) ||
                     (n == 6 && (d[1] == 4 || d[1] == 6) && d[5] == 0xff);
    case 0x22: return n == 3 && d[1] == 0xf1 && (d[2] == 0x87 || d[2] == 0x89 || d[2] == 0x90 || d[2] == 0x97);
    case 0x3e: return n == 2 && d[1] == 0;
    default: return false;
  }
}

inline bool frame(uint32_t a, const uint8_t *data, size_t size, uint8_t bus) {
  if (bus > 2 || !((a >= 0x600 && a <= 0x7ff) || a == 0x24b || a == 0x18db33f1 ||
      (a <= 0x1fffffff && (a & 0x1fff00ff) == 0x18da00f1))) return false;
  if (size != 8) return false;
  if (a == 0x7df || a == 0x18db33f1) return data[0] == 2 && data[1] == 1 && data[2] == 0;
  // Requests fit in one ISO-TP frame; only flow control is needed for long replies.
  for (size_t offset : {0U, 1U}) {
    auto d = data + offset;
    if (d[0] == 0x30 || (d[0] > 0 && d[0] <= 7 - offset && read_request(d + 1, d[0]))) return true;
  }
  return false;
}
}
