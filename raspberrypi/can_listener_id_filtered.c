#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>

#define CAN_INTERFACE "can0"
#define RESPONSE_CAN_ID 0x7E8
const unsigned char response_data[5] = {0x0F, 0xF0, 0x0F, 0xF0, 0x00};
#define RESPONSE_DLC sizeof(response_data)

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_frame resp_frame;
    int nbytes;

    struct can_filter rfilter[] = {
        { .can_id = 0x123, .can_mask = CAN_SFF_MASK },
        { .can_id = 0x456, .can_mask = CAN_SFF_MASK },
        { .can_id = 0x30, .can_mask = CAN_SFF_MASK }
    };
    int num_filters = sizeof(rfilter) / sizeof(struct can_filter);

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket creation error");
        return 1;
    }

    strcpy(ifr.ifr_name, CAN_INTERFACE);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl error (get interface index)");
        close(s);
        return 1;
    }

    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        perror("setsockopt filter error");
        close(s);
        return 1;
    }
    printf("CAN filter applied: %d\n", num_filters);
    for (int i = 0; i < num_filters; ++i) {
        printf("  - ID: 0x%X, Mask: 0x%X\n", rfilter[i].can_id, rfilter[i].can_mask);
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Binding error");
        close(s);
        return 1;
    }

    printf("Connected to '%s'. Waiting for messages...\n", CAN_INTERFACE);

    resp_frame.can_id = RESPONSE_CAN_ID;
    resp_frame.can_dlc = RESPONSE_DLC;
    memcpy(resp_frame.data, response_data, RESPONSE_DLC);

    while (1) {
        // 5. Read CAN frame
        nbytes = read(s, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            if (errno == ENETDOWN) {
                fprintf(stderr, "Network interface is down!\n");
                break; // Exit on interface down
            }
            perror("CAN RAW socket read error");
            continue;
        } else if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame received: %d bytes\n", nbytes);
            continue;
        }

        printf("\nReceived ");
        printf("ID:0x%X, ", frame.can_id & CAN_SFF_MASK);
        printf("DLC:%d\n", frame.can_dlc);
        printf("data:");
        for (int i = 0; i < frame.can_dlc; i++) {
            printf(" %02X", frame.data[i]);
        }
        printf("\n");
        printf("Humidity:%02d.%02d, Temperature:%02d.%02.d\n",
            frame.data[0], frame.data[1], frame.data[2], frame.data[3]);

        resp_frame.data[4] = frame.data[4];

        // send response
        nbytes = write(s, &resp_frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("CAN RAW scoket write error (response send failed)");
        } else if (nbytes < sizeof(struct can_frame)) {
            fprintf(stderr, "Incomplete CAN frame sent: %d bytes\n", nbytes);
        } else {
            printf("sent response.\n");
        }
    }

    close(s);
    printf("Disconnected from interface '%s'.\n", CAN_INTERFACE);

    return 0;
}
