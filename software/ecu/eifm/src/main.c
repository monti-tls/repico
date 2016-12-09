#include "platform.h"
#include "itm.h"

int main()
{
    itm_init();
    for (int i = 0; i < 100; ++i) {
        itm_printf("YOLO!\n");
    }

    for (;;);
}
