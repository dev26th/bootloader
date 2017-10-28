#pragma once

#include <unistd.h>

pid_t _getpid() {
    return 0;
}

void _exit(int status) {
    (void)status;
    while(1) {}
}

int _kill(pid_t pid, int sig) {
    (void)pid; (void)sig;
    return 0;
}

