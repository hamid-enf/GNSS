// examples/simple/simple_stream.c
// مثال استریم: شبیه‌سازی UART که بایت‌به‌بایت می‌آید

#include "gnss.h"
#include <stdio.h>
#include <string.h>

// شبیه‌سازی تابع UART receive
void simulate_uart_receive(const char* data)
{
    for (size_t i=0;i<strlen(data);i++) {
        gnss_feed_byte((uint8_t)data[i]);

        // هر وقت solution جدید داشتیم، چاپ کن
        if (gnss_has_new_solution()) {
            gnss_solution_t sol;
            if (gnss_get_solution(&sol)==GNSS_OK) {
                printf("[UART] New fix: lat=%.6f lon=%.6f sats=%d\n",
                    sol.latitude_deg, sol.longitude_deg, sol.satellites);
            }
        }
    }
}

int main(void)
{
    gnss_init();
    printf("=== Simple Stream Example ===\n");

    // استریم واقعی با نویز و چند جمله
    const char* stream1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
    const char* stream2 = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

    printf("Feeding stream1...\n");
    simulate_uart_receive(stream1);

    printf("Feeding stream2...\n");
    simulate_uart_receive(stream2);

    // مثال با DMA buffer
    printf("\n=== DMA Buffer Example ===\n");
    uint8_t dma_buf[128] = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.2,M,,*76\r\n";
    gnss_feed(dma_buf, strlen((char*)dma_buf));
    if (gnss_has_new_solution()) {
        gnss_solution_t sol;
        gnss_get_solution(&sol);
        printf("DMA solution: %.6f, %.6f\n", sol.latitude_deg, sol.longitude_deg);
    }

    return 0;
}
