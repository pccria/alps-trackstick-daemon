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
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <sys/inotify.h>
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

int alps_device_monitor_create(void) {
    int ifd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (ifd < 0) return -1;

    int wd = inotify_add_watch(ifd, "/dev/input", IN_CREATE | IN_ATTRIB);
    if (wd < 0) {
        close(ifd);
        return -1;
    }
    return ifd;
}

int alps_device_monitor_drain(int inotify_fd) {
    if (inotify_fd < 0) return -1;

    char buf[4096] __attribute__((aligned(__alignof__(struct inotify_event))));
    int count = 0;

    while (1) {
        ssize_t len = read(inotify_fd, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            return -1;
        }
        if (len == 0) break;
        count++;
    }
    return count;
}

void alps_device_monitor_destroy(int inotify_fd) {
    if (inotify_fd >= 0) {
        close(inotify_fd);
    }
}

static const char *find_systemctl(void) {
    if (access("/bin/systemctl", X_OK) == 0) return "/bin/systemctl";
    if (access("/usr/bin/systemctl", X_OK) == 0) return "/usr/bin/systemctl";
    return NULL;
}

int alps_systemd_is_active(const char *service_name) {
    if (!service_name) return 0;
    const char *sysctl = find_systemctl();
    if (!sysctl) return -1;

    pid_t pid = fork();
    if (pid < 0) return -1;

    if (pid == 0) {
        int devnull = open("/dev/null", O_WRONLY);
        if (devnull >= 0) {
            dup2(devnull, STDOUT_FILENO);
            dup2(devnull, STDERR_FILENO);
            close(devnull);
        }
        char *const args[] = {
            (char *)sysctl, "is-active", "--quiet", (char *)service_name, NULL
        };
        execv(sysctl, args);
        _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        return 1;
    }
    return 0;
}

int alps_systemd_restart(const char *service_name) {
    if (!service_name) return -1;
    const char *sysctl = find_systemctl();
    if (!sysctl) return -1;

    pid_t pid = fork();
    if (pid < 0) return -1;

    if (pid == 0) {
        char *const args[] = {
            (char *)sysctl, "restart", (char *)service_name, NULL
        };
        execv(sysctl, args);
        _exit(127);
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0) return -1;

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
        return 0;
    }
    return -1;
}

void alps_service_offer_restart(const char *service_name, int force) {
    if (!service_name) service_name = "alps-stick";

    int active = alps_systemd_is_active(service_name);
    if (active < 0) {
        printf("Notice: systemctl not found. Restart your daemon process manually to apply changes.\n\n");
        return;
    }

    if (!active) {
        printf("Notice: '%s.service' is not currently active.\n", service_name);
        printf("To start the background service, run:\n");
        printf("  sudo systemctl start %s\n\n", service_name);
        return;
    }

    int do_restart = 0;
    if (force) {
        do_restart = 1;
    } else if (isatty(STDIN_FILENO)) {
        printf("Restart %s.service now to apply changes? [Y/n]: ", service_name);
        fflush(stdout);
        char ans[64];
        if (fgets(ans, sizeof(ans), stdin)) {
            char *p = ans;
            while (*p == ' ' || *p == '\t') p++;
            if (*p == '\0' || *p == '\n' || *p == '\r' || *p == 'y' || *p == 'Y') {
                do_restart = 1;
            }
        }
    }

    if (do_restart) {
        printf("Restarting %s.service...\n", service_name);
        fflush(stdout);
        int rc = alps_systemd_restart(service_name);
        if (rc == 0) {
            printf("✓ Successfully restarted %s.service!\n\n", service_name);
        } else {
            printf("Notice: Could not restart service directly. Run manually:\n");
            printf("  sudo systemctl restart %s\n\n", service_name);
        }
    } else {
        printf("To activate changes later, run:\n");
        printf("  sudo systemctl restart %s\n\n", service_name);
    }
}
