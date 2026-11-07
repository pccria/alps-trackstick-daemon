/**
 * @file device.c
 * @brief Implementation of `/dev/input` discovery and safe `xinput` invocation.
 */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "io/device.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <linux/input.h>

int alps_device_find_stick(char *out_path, size_t path_max,
                           char *out_name, size_t name_max) {
    DIR *dir = opendir("/dev/input");
    if (!dir) return -1;

    struct dirent *ent;
    int found_fd = -1;

    while ((ent = readdir(dir)) != NULL) {
        if (strncmp(ent->d_name, "event", 5) != 0) continue;

        char path[512];
        int n = snprintf(path, sizeof(path), "/dev/input/%s", ent->d_name);
        if (n < 0 || (size_t)n >= sizeof(path)) continue;

        int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) continue;

        char name[256];
        memset(name, 0, sizeof(name));
        if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), name) >= 0) {
            name[sizeof(name) - 1] = '\0';
            if (strstr(name, "DualPoint Stick") != NULL) {
                if (out_path && path_max > 0) {
                    snprintf(out_path, path_max, "%s", path);
                }
                if (out_name && name_max > 0) {
                    snprintf(out_name, name_max, "%s", name);
                }
                found_fd = fd;
                break;
            }
        }
        close(fd);
    }

    closedir(dir);
    return found_fd;
}

/**
 * @brief Executes `/usr/bin/xinput` safely without shell interpolation.
 */
static void run_xinput_cmd(char *const argv[]) {
    pid_t pid = fork();
    if (pid < 0) return;

    if (pid == 0) {
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        execv("/usr/bin/xinput", argv);
        _exit(127);
    } else {
        int status = 0;
        waitpid(pid, &status, 0);
    }
}

void alps_device_apply_pointer_speed(const char *dev_name, float speed) {
    if (!dev_name || speed < -1.0f || speed > 1.0f) return;

    char speed_str[32];
    snprintf(speed_str, sizeof(speed_str), "%.2f", speed);

    char *const args_speed[] = {
        "xinput", "set-prop", (char *)dev_name, "libinput Accel Speed", speed_str, NULL
    };
    run_xinput_cmd(args_speed);

    char *const args_profile[] = {
        "xinput", "set-prop", (char *)dev_name, "libinput Accel Profile Enabled", "1", "0", NULL
    };
    run_xinput_cmd(args_profile);
}
