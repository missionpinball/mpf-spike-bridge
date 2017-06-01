// Compile: /usr/local/musl/bin/musl-gcc --static bridge.c -Wall -o bridge
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 

int set_interface_attribs(int fd, int speed)
{
    struct termios tty;

    if (tcgetattr(fd, &tty) < 0) {
        printf("Error from tcgetattr: %s\n", strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, (speed_t)speed);
    cfsetispeed(&tty, (speed_t)speed);

    tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    /* setup for non-canonical mode */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;

    /* fetch bytes as they become available */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error from tcsetattr: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

int get_gpio_state(int gpio) {
    int fd = open("/dev/gpio", O_RDWR);
    int result = ioctl(fd, 0x3C02, gpio);
    close(fd);
    return result;
}

void set_gpio_on(int gpio) {
    int fd = open("/dev/gpio", O_RDWR);
    ioctl(fd, 0x3C03, gpio);
    close(fd);
}

void set_gpio_off(int gpio) {
    int fd = open("/dev/gpio", O_RDWR);
    ioctl(fd, 0x3C04, gpio);
    close(fd);
}

void set_backlight(int brightness) {
    int fd = open("/dev/backlight", O_RDWR);
    ioctl(fd, 0x4001, &brightness);
    close(fd);
}

int message_position = 0;
char message[259];  // 256 + 2 (node, length, response_len)
int fd;
int spi_fd;
int local_activity = 0;
char local_switches[8] = {0,0,0,0,0,0,0,0};

void process_message() {
    if (message[0] == 0x00) {
        //printf("Poll\n");
        // Intercept poll if we have local activity
        if (local_activity) {
            // Tell MPF that there is local activity
           printf("F0 "); 
        } else {
            // Or forward to bus
            write(fd, &message, 1);

            // Also check local inputs
            char tmp_inputs[8];
            read(spi_fd, tmp_inputs, sizeof tmp_inputs);
            if (memcmp(tmp_inputs, local_switches, 8)) {
                local_activity = 1;
            }

        }
    } else if (message[0] == 0x01) {
        // Sleep after certain messages to keep bus in sync. MPF specific
        struct timespec ts;
        ts.tv_sec = 0;
        ts.tv_nsec = message[1] * 1000000;
        nanosleep(&ts, NULL);
    } else if ((message[0] & 0xF0) == 0x80) {
        //printf("Bus message received\n");
        if ((message[0] & 0x0F) == 0x00 && (message[2] & 0xFF) == 0x11) {
            // Intercept input reads to board 0
            local_activity = 0;
            //printf("Read local switches\n");
            int j;
            read(spi_fd, local_switches, sizeof local_switches);
            char checksum = 0;
            for (j = 0; j < 8; j++)
            {
                printf("%02X ", local_switches[j]);
                checksum += local_switches[j];
            }
            checksum = 256 - checksum;
            printf("%02X 00x", checksum & 0xFF);

        } else if ((message[1] & 0xFF) == 0x04 && (message[2] & 0xFF) == 0x80) {
            // Intercept LED sets to backlight
            int brightness = ((message[4] & 0xFF) << 8);
            set_backlight(brightness);
        } else {
            // Send message to nodebus
            write(fd, &message, message[1] + 3);
        }
    } else {
        /*printf("Invalid message received");
        int j;
        for (j = 0; j < message[1] + 3; j++)
        {
            printf("%02X ", message[j]);
        }*/
    }
}

void process_byte(char bus_byte)
{
    if (message_position < 0 || message_position >= 256) {
        message_position = 0;
        memset(message, 0, sizeof message);
    }

    if (message_position == 0) {
        // First byte - type + node
        if (bus_byte == 0x00) {
            // Poll
            message[0] = bus_byte;
            process_message();
            message_position = -1;
        } else if (bus_byte == 0x01) {
            // Custom wait command to get bus sync
            message[0] = bus_byte;
            message_position = 1;
        } else if ((bus_byte & 0xF0) == 0x80) {
            // Node bus message
            message_position = 1;
            message[0] = bus_byte;
        } else {
            // Invalid. We probably lost sync.
            message_position = -1;
        }
    } else if (message_position == 1 && message[0] == 0x01) {
        message[1] = bus_byte;
        process_message();
        message_position = -1;
    } else if (message_position == 1) {
        // Second byte - length
        message[1] = bus_byte;
        message_position = 2;
    } else {
        message[message_position] = bus_byte;
        message_position += 1;

        // Data + readback
        if (message_position - 3 == message[1]) {
            // Message complete
            process_message();
            message_position = -1;
        }
    }
}

int main()
{
    printf("MPF Spike Bridge!\n");

    // Disable backlight
    set_backlight(0);

    // Enable Nodebus power
    set_gpio_on(0x6B);

    // Enable Amp
    //set_gpio_on(0x6A);

    // Open Spike bus
    char *portname = "/dev/ttyS4";

    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        printf("Error opening %s: %s\n", portname, strerror(errno));
        return -1;
    }

    // Set RTS and DTS on Node bus
    int flags;
    flags = TIOCM_RTS | TIOCM_DTR; 
    ioctl(fd, TIOCMBIS, &flags);

    // Open local ports on cpu
    char *spi_portname = "/dev/spi1";
    spi_fd = open (spi_portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (spi_fd < 0)
    {
        printf("Error opening %s: %s\n", spi_portname, strerror(errno));
        return -1;
    }

    // Open stdin
    int stdin_fd = 0;
    set_interface_attribs (fd, B460800);  // set speed

    setbuf(stdout, NULL);
    setbuf(stdin, NULL);

    struct termios old, new;
    if (tcgetattr (fileno (stdin), &old) != 0)
        return -1;
    new = old;
    new.c_lflag &= ~ECHO;
    if (tcsetattr (fileno (stdin), TCSAFLUSH, &new) != 0)
    {
        printf("Error setting stdin attributes");
        return -1;
    }

    // Loop variables
    char buffer[128];
    char last = 0;

    while (1) {
        fd_set set;
        /* Initialize the file descriptor set. */
        FD_ZERO (&set);
        FD_SET (stdin_fd, &set);
        FD_SET (fd, &set);

        select(spi_fd + 1, &set, NULL, NULL, NULL);
        int n;
        if (FD_ISSET(stdin_fd, &set)) {
            if (last != 0) {
                buffer[0] = last;
                n = read(stdin_fd, buffer + 1, sizeof buffer - 1);
                n += 1;
            } else {
                n = read(stdin_fd, buffer, sizeof buffer);
            }
//            printf("Got %02X bytes from stdin\n", n);
            int j = 0;
            int offset = 0;
            if (buffer[0] == ' ' || buffer[0] == '\n') {
                offset = 1;
            }
            for (j = offset; j < n - 1; j += 3)
            {
                unsigned int u;
                sscanf(buffer + j, "%2x", &u);
                char b = (char)u;
                process_byte(b);
//                write(fd, &b, 1);
//                printf("Sending: %02X\n", b);
            }
            if ((n - offset) % 3 == 1) {
                last = buffer[n - 1];
            } else {
                last = 0;
            }
        } else if (FD_ISSET(fd, &set)) {
            n = read(fd, buffer, sizeof buffer);
//            printf("Got %02X bytes from ttyS4\n", n);
            int j = 0;
            for (j = 0; j < n; j++)
            {
                printf("%02X ", buffer[j]);
            }
        } else {
            printf("Select failure\n");
        }
    }
    return 0;
}
