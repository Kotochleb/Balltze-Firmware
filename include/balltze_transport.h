#ifndef BALLTZE_TRANSPORT
#define BALLTZE_TRANSPORT

#include <Arduino.h>
#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

int clock_gettime(clockid_t unused, struct timespec *tp);
bool balltze_transport_open(struct uxrCustomTransport * transport);
bool balltze_transport_close(struct uxrCustomTransport * transport);
size_t balltze_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t balltze_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

#endif // BALLTZE_TRANSPORT