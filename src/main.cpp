#include "GamecubeConsole.hpp"
#include "inputs.hpp"
#include "keycode.h"
#include "keymap.hpp"
#include "logic.hpp"
#include "runtime_remapping.hpp"
#include "tusb.h"

#include "bsp/board.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/bootrom.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "hardware/clocks.h"

#include "nerfs/modes/MeleeLimits.hpp"
#include "nerfs/core/state.hpp"

#define LED_PIN 25

#define NUMBER_OF_INPUTS 24

// nerfs constants
#define SAMPLE_SPACING 1000U // 1 unit = 4 microseconds

// logging
#define OSC_PIN 16

void joybus_loop();

CFG_TUSB_MEM_SECTION extern hid_keyboard_report_t usb_keyboard_report;

bool mode_selected = false;

RectangleInput rectangleInput;
gc_report_t gcReport = default_gc_report;

const bool _nerfOn = GC_NERF_ON;
const uint8_t leadTime = 35; // estimated 35us to poll keyboard and convert to haybox input format

int main() {
    board_init();

    // Clock at 200MHz
    set_sys_clock_khz(200'000, true);

    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

#ifdef GC_3V3_PIN
    gpio_init(GC_3V3_PIN);
    gpio_set_dir(GC_3V3_PIN, GPIO_IN);
    gpio_pull_down(GC_3V3_PIN);

    sleep_ms(200);

    if (!gpio_get(GC_3V3_PIN)) {
        reset_usb_boot(0, 0);
    }
#endif

    initLogic(ParasolDashing::BAN, SlightSideB::BAN);

    KeyMapping *keymap = getKeymap();
    initInputs(keymap, NUMBER_OF_INPUTS);

    multicore_launch_core1(joybus_loop);

    tusb_init();

    //Testing with oscilloscope
    gpio_init(OSC_PIN);
    gpio_set_dir(OSC_PIN, GPIO_OUT);

    uint64_t lastLoopEnd = get_absolute_time();
    
    while (1) {
        
        if (_nerfOn) 
        {   
            busy_wait_until(lastLoopEnd + SAMPLE_SPACING);
            lastLoopEnd = get_absolute_time();
        }

        // Poll keyboard
        tuh_task();
        if (!mode_selected) {
            uint8_t key = findFirstPressedKey(&usb_keyboard_report);
            if (key != 0) {
                if (key == KC_ESC) {
                    sleep_ms(3000);
                    multicore_reset_core1();
                    remap();
                }
                mode_selected = true;
            }
        }

        // Map keyboard inputs to rectangle button state.
        rectangleInput = getRectangleInput(&usb_keyboard_report);

        if (_nerfOn) {
            // Implement Ruleset Nerfs
            InputState inputs;
            OutputState outputs;
            getIOStates(rectangleInput, inputs, outputs);

            OutputState nerfedOutputs;
            gpio_put(OSC_PIN, 1);
            limitOutputs(SAMPLE_SPACING / 4, _nerfOn ? AB_A : AB_B, inputs, outputs, nerfedOutputs);
            gpio_put(OSC_PIN, 0);
            makeNerfedReport(nerfedOutputs, &gcReport);
        } else {

            // Map rectangle button state to GC controller state.
            makeReport(rectangleInput, &gcReport);
        }
    }

    return 1;
}

void joybus_loop() {
    GamecubeConsole *gc = new GamecubeConsole(GC_DATA_PIN, pio0);

    while (1) {
        gc->WaitForPoll();
        gc->SendReport(&gcReport);
    }
}
