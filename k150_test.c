#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

// Test K150 communication directly
int main() {
    int fd;
    struct termios tty;
    unsigned char buffer[10];
    int bytes_read;
    
    printf("Testing K150 direct communication...\n");
    
    // Open serial port
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("ERROR: Cannot open /dev/ttyUSB0: %s\n", strerror(errno));
        return 1;
    }
    
    // Configure serial port (19200 8N1)
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        printf("ERROR: tcgetattr failed\n");
        close(fd);
        return 1;
    }
    
    cfsetospeed(&tty, B19200);
    cfsetispeed(&tty, B19200);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 50; // 5 second timeout
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("ERROR: tcsetattr failed\n");
        close(fd);
        return 1;
    }
    
    printf("Serial port configured (19200 8N1)\n");
    
    // Clear DTR and RTS first
    int status;
    ioctl(fd, TIOCMGET, &status);
    status &= ~(TIOCM_DTR | TIOCM_RTS);
    ioctl(fd, TIOCMSET, &status);
    usleep(100000); // 100ms
    
    // Set DTR high
    status |= TIOCM_DTR;
    ioctl(fd, TIOCMSET, &status);
    usleep(1000); // 1ms
    
    // Clear DTR (reset pulse)
    status &= ~TIOCM_DTR;
    ioctl(fd, TIOCMSET, &status);
    printf("DTR reset pulse sent\n");
    
    // Set RTS
    status |= TIOCM_RTS;
    ioctl(fd, TIOCMSET, &status);
    
    // Wait for response
    usleep(10000); // 10ms
    
    // Try to read response
    bytes_read = read(fd, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        printf("Received %d bytes: ", bytes_read);
        for (int i = 0; i < bytes_read; i++) {
            printf("0x%02X ", buffer[i]);
        }
        printf("\n");
        
        if (buffer[0] == 0x42) {
            printf("SUCCESS: K150 responded with 'B' (0x42) - Boot signal!\n");
        }
    } else {
        printf("No response from K150\n");
    }
    
    // Test start command
    printf("Sending start command 'P.' (0x50 0x2E)...\n");
    unsigned char start_cmd[2] = {0x50, 0x2E};
    write(fd, start_cmd, 2);
    usleep(50000); // 50ms
    
    bytes_read = read(fd, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        printf("Start response: %d bytes: ", bytes_read);
        for (int i = 0; i < bytes_read; i++) {
            printf("0x%02X ", buffer[i]);
        }
        printf("\n");
    }
    
    close(fd);
    return 0;
}
