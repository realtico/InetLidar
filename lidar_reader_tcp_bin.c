#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define DEFAULT_MAX_POINTS 1000
#define DEFAULT_PORT 9999
#define DEBUG 1

#pragma pack(push, 1)
typedef struct {
    float angle;       // 4 bytes
    uint16_t distance; // 2 bytes (milímetros)
} LidarPoint;

typedef struct {
    uint32_t num_points;   // 4 bytes
    uint64_t timestamp_ms; // 8 bytes
} LidarHeader;
#pragma pack(pop)

LidarPoint last_points[DEFAULT_MAX_POINTS];
int last_count = 0;
pthread_mutex_t frame_mutex = PTHREAD_MUTEX_INITIALIZER;

long long current_millis() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000LL + ts.tv_nsec / 1000000;
}

int configure_serial(int fd) {
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) return -1;

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    return tcsetattr(fd, TCSANOW, &tty);
}

int read_byte(int fd) {
    unsigned char c;
    int n = read(fd, &c, 1);
    return (n == 1) ? c : -1;
}

void* serial_reader_thread(void* arg) {
    const char* serial_port = (const char*)arg;
    float *angles = malloc(sizeof(float) * DEFAULT_MAX_POINTS);
    int *dists = malloc(sizeof(int) * DEFAULT_MAX_POINTS);
    if (!angles || !dists) exit(1);

    int fd = open(serial_port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0 || configure_serial(fd) != 0) exit(1);

    int count = 0;
    static int frame_id = 0;
    static long long last_frame_time = 0;

    while (1) {
        int c = read_byte(fd);
        if (c != 0xAA) continue;
        if (read_byte(fd) != 0x55) continue;

        int CT = read_byte(fd);
        int LSN = read_byte(fd);
        int FSA = read_byte(fd) | (read_byte(fd) << 8);
        int LSA = read_byte(fd) | (read_byte(fd) << 8);
        read_byte(fd); read_byte(fd); // CS (ignore)

        float F = (FSA >> 1) / 64.0;
        float L = (LSA >> 1) / 64.0;

        for (int i = 0; i < LSN; ++i) {
            int Si = read_byte(fd) | (read_byte(fd) << 8);
            int dist = Si >> 2;
            float A_corr = (dist == 0) ? 0.0 : atan(19.16 * (dist - 90.15) / (dist * 90.15));
            float angle = F + ((L - F) / LSN) * i - A_corr;

            if (count < DEFAULT_MAX_POINTS) {
                angles[count] = angle;
                dists[count] = dist;
                count++;
            }
        }

        if (CT == 1) {
            long long ts = current_millis();
            pthread_mutex_lock(&frame_mutex);
            last_count = count;
            for (int i = 0; i < count; ++i) {
                last_points[i].angle = angles[i];
                last_points[i].distance = (uint16_t)dists[i];
            }
            pthread_mutex_unlock(&frame_mutex);
            if (DEBUG) {
                long long delta = ts - last_frame_time;
                last_frame_time = ts;
                printf("[Serial] Frame %d armazenado: %d pontos, delay=%lldms\n", ++frame_id, count, delta);
                fflush(stdout);
            }
            count = 0;
        }
    }

    close(fd);
    free(angles);
    free(dists);
    return NULL;
}

int main(int argc, char *argv[]) {
    const char* serial_port = (argc > 1) ? argv[1] : DEFAULT_SERIAL_PORT;
    int port = DEFAULT_PORT;

    pthread_t reader_thread;
    pthread_create(&reader_thread, NULL, serial_reader_thread, (void*)serial_port);

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    listen(server_fd, 5);
    printf("[LidarTCP] Aguardando conexoes na porta %d...\n", port);

    while (1) {
        int client_fd = accept(server_fd, NULL, NULL);
        if (client_fd < 0) continue;

        pthread_mutex_lock(&frame_mutex);
        LidarHeader header;
        header.num_points = last_count;
        header.timestamp_ms = current_millis();

        write(client_fd, &header, sizeof(header));
        write(client_fd, last_points, last_count * sizeof(LidarPoint));
        pthread_mutex_unlock(&frame_mutex);

        close(client_fd);
    }

    close(server_fd);
    return 0;
}