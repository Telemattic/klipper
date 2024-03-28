/* scuart_hw.c */

#include "scuart_hw.h"

#include <string.h>
#include "hardware/structs/clocks.h"
#include "hardware/structs/iobank0.h" // iobank0_hw
#include "hardware/structs/padsbank0.h" // padsbank0_hw
#include "hardware/structs/pio.h" // pio0_hw
#include "hardware/structs/resets.h" // resets_hw

#if 0

#undef PICO_NO_HARDWARE
#define PICO_NO_HARDWARE 1
#include "sc_uart.pio.h"

#else

#include <stdbool.h>
typedef unsigned int uint;

#ifndef SYS_CLK_KHZ
#define SYS_CLK_KHZ  125000u
#endif

#define sc_uart_offset_rx_start 0u
#define sc_uart_offset_rx_end 5u
#define sc_uart_offset_tx_start 5u
#define sc_uart_offset_tx_end 19u

static const uint16_t sc_uart_program_instructions[] = {
            //     .wrap_target
    0x2020, //  0: wait   0 pin, 0                   
    0xea27, //  1: set    x, 7                   [10]
    0x4001, //  2: in     pins, 1                    
    0x0642, //  3: jmp    x--, 2                 [6] 
    0x0000, //  4: jmp    0                          
    0x80a0, //  5: pull   block                      
    0xa047, //  6: mov    y, osr                     
    0xe081, //  7: set    pindirs, 1                 
    0x000e, //  8: jmp    14                         
    0x80a0, //  9: pull   block                      
    0xe000, // 10: set    pins, 0                    
    0xe627, // 11: set    x, 7                   [6] 
    0x6001, // 12: out    pins, 1                    
    0x064c, // 13: jmp    x--, 12                [6] 
    0xe701, // 14: set    pins, 1                [7] 
    0x0089, // 15: jmp    y--, 9                     
    0xe080, // 16: set    pindirs, 0                 
    0xc020, // 17: irq    wait 0                     
    0x0005, // 18: jmp    5                          
            //     .wrap
};

#endif

#define pico_default_asm_volatile(...) __asm volatile (".syntax unified\n" __VA_ARGS__)

// #include <hardware/sync.h> // save_and_disable_interrupts, restore_interrupts

#define USE_SDK 0

#define SCUART_SM_TX           0
#define SCUART_SM_RX           1

#define SCUART_IRQ_TX_COMPLETE (PIO_IRQ0_INTE_SM0_BITS          << SCUART_SM_TX)
#define SCUART_IRQ_TXNFULL     (PIO_IRQ0_INTE_SM0_TXNFULL_BITS  << SCUART_SM_TX)
#define SCUART_IRQ_RXNEMPTY    (PIO_IRQ0_INTE_SM0_RXNEMPTY_BITS << SCUART_SM_RX)

#if USE_SDK

static inline void
sc_uart_program_init(pio_hw_t* pio, uint32_t pin, uint32_t baud)
{
    pio_gpio_init(pio, pin);
    gpio_set_pulls(pin, false, false);
    
    pio_sm_set_pins_with_mask(pio, SCUART_SM_TX, 1u << pin, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio, SCUART_SM_TX, 0u << pin, 1u << pin);
    //    pio_sm_set_pindirs_with_mask(pio, SCUART_SM_RX, 0u << pin, 1u << pin);

    pio_sm_config cfg_tx = sc_uart_program_get_default_config(0);

    sm_config_set_out_pins(&cfg_tx, pin, 1);
    sm_config_set_set_pins(&cfg_tx, pin, 1);
    sm_config_set_in_pins(&cfg_tx, pin);
    sm_config_set_jmp_pin(&cfg_tx, pin);

    // SM transmits 1 bit per 8 execution cycles.
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&cfg_tx, div);

    pio_sm_config cfg_rx = cfg_tx;
    
    // OUT shifts to right, NO autopull, 8-bit data
    sm_config_set_out_shift(&cfg_tx, true, false, 8);
    sm_config_set_fifo_join(&cfg_tx, PIO_FIFO_JOIN_TX);
    
    // IN shifts to right, autopush, 8-bit data
    sm_config_set_in_shift(&cfg_rx, true, true, 8);
    sm_config_set_fifo_join(&cfg_rx, PIO_FIFO_JOIN_RX);

    pio_sm_init(pio, SCUART_SM_TX, sc_uart_offset_tx_start, &cfg_tx);
    pio_sm_init(pio, SCUART_SM_RX, sc_uart_offset_rx_start, &cfg_rx);
    pio_sm_set_enabled(pio, SCUART_SM_TX, true);
    pio_sm_set_enabled(pio, SCUART_SM_RX, true);
}

#else

typedef struct {
    uint32_t clkdiv;
    uint32_t execctrl;
    uint32_t shiftctrl;
    uint32_t pinctrl;
} pio_sm_config;

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

static inline void
my_pio_sm_set_enabled(pio_hw_t* pio, uint32_t sm, bool enabled)
{
    pio->ctrl = enabled ? (pio->ctrl | (1u << sm)) : (pio->ctrl & ~(1u << sm));
}

static inline void
my_pio_sm_restart(pio_hw_t* pio, uint32_t sm)
{
    hw_set_bits(&pio->ctrl, 1u << (PIO_CTRL_SM_RESTART_LSB + sm));
}

static inline void
my_pio_sm_clkdiv_restart(pio_hw_t* pio, uint32_t sm)
{
    hw_set_bits(&pio->ctrl, 1u << (PIO_CTRL_CLKDIV_RESTART_LSB + sm));
}

inline static void
my_pio_sm_exec(pio_hw_t* pio, uint32_t sm, uint32_t instr)
{
    pio->sm[sm].instr = instr;
}

static inline void
my_pio_sm_set_config(pio_hw_t* pio, uint32_t sm, const pio_sm_config *config)
{
    pio->sm[sm].clkdiv = config->clkdiv;
    pio->sm[sm].execctrl = config->execctrl;
    pio->sm[sm].shiftctrl = config->shiftctrl;
    pio->sm[sm].pinctrl = config->pinctrl;
}

uint
my_pio_encode_jmp(uint addr)
{
    // jump immediate instruction is 0x0000, address in 5 LSBs
    return addr & 0x1Fu;
}

static inline void
my_pio_sm_init(pio_hw_t* pio, uint32_t sm, uint32_t initial_pc, const pio_sm_config *config)
{
    // Halt the machine, set some sensible defaults
    my_pio_sm_set_enabled(pio, sm, false);

    my_pio_sm_set_config(pio, sm, config);

    // changing the FIFO join state clears the fifo
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
    hw_xor_bits(&pio->sm[sm].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);

    // Clear FIFO debug flags
    const uint32_t fdebug_sm_mask =
            (1u << PIO_FDEBUG_TXOVER_LSB) |
            (1u << PIO_FDEBUG_RXUNDER_LSB) |
            (1u << PIO_FDEBUG_TXSTALL_LSB) |
            (1u << PIO_FDEBUG_RXSTALL_LSB);
    pio->fdebug = fdebug_sm_mask << sm;

    // Finally, clear some internal SM state
    my_pio_sm_restart(pio, sm);
    my_pio_sm_clkdiv_restart(pio, sm);
    my_pio_sm_exec(pio, sm, my_pio_encode_jmp(initial_pc));
}

static inline void
scuart_hw_program_init(pio_hw_t* pio, uint32_t pin, uint32_t baud)
{
    // Set input enable on, output disable off, no pullup or pulldown
    hw_write_masked(&padsbank0_hw->io[pin],
                   PADS_BANK0_GPIO0_IE_BITS,
                    PADS_BANK0_GPIO0_IE_BITS
                    | PADS_BANK0_GPIO0_OD_BITS
                    | PADS_BANK0_GPIO0_PUE_BITS
                    | PADS_BANK0_GPIO0_PDE_BITS
    );
    
    // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
    // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
    iobank0_hw->io[pin].ctrl =
        (pio == pio0_hw
         ? IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO0_0
         : IO_BANK0_GPIO0_CTRL_FUNCSEL_VALUE_PIO1_0) << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
    
    pio_sm_config cfg_tx = {0, 0, 0, 0};

    cfg_tx.pinctrl =
          (pin << PIO_SM0_PINCTRL_OUT_BASE_LSB)   // first OUT pin
        | (1 << PIO_SM0_PINCTRL_OUT_COUNT_LSB)    // 1 OUT pin
        | (pin << PIO_SM0_PINCTRL_SET_BASE_LSB)   // first SET pin
        | (1 << PIO_SM0_PINCTRL_SET_COUNT_LSB)    // 1 SET pin
        | (pin << PIO_SM0_PINCTRL_IN_BASE_LSB);   // first IN pin

    // not using the wrap functionality, although we could
    cfg_tx.execctrl =
          (pin << PIO_SM0_EXECCTRL_JMP_PIN_LSB)   // JMP pin
        | (0 << PIO_SM0_EXECCTRL_WRAP_BOTTOM_LSB) // wrap TO this address
        | (31 << PIO_SM0_EXECCTRL_WRAP_TOP_LSB);  // wrap FROM this address

    // SM transmits 1 bit per 8 execution cycles.
    //
    // clkdiv is 16.8 fixed point, so instead of clk*256/(8*baud) we can just do clk*32/baud
    // uint32_t div = ;
    cfg_tx.clkdiv = ((SYS_CLK_KHZ * 1000 * 32) / baud) << PIO_SM0_CLKDIV_FRAC_LSB;

    pio_sm_config cfg_rx = cfg_tx;

    // OUT shifts to right, NO autopull, 8-bit data
    cfg_tx.shiftctrl =
          PIO_SM0_SHIFTCTRL_OUT_SHIFTDIR_BITS        // shift right
        | (8 << PIO_SM0_SHIFTCTRL_PULL_THRESH_LSB)   // 8-bit data
        | PIO_SM0_SHIFTCTRL_FJOIN_TX_BITS;           // PIO_FIFO_JOIN_TX
    
    // IN shifts to right, autopush, 8-bit data
    cfg_rx.shiftctrl =
          PIO_SM0_SHIFTCTRL_IN_SHIFTDIR_BITS         // shift right
        | PIO_SM0_SHIFTCTRL_AUTOPUSH_BITS            // autopush
        | (8 << PIO_SM0_SHIFTCTRL_PUSH_THRESH_LSB)   // 8-bit data
        | PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;           // PIO_FIFO_JOIN_RX

    my_pio_sm_init(pio, SCUART_SM_TX, sc_uart_offset_tx_start, &cfg_tx);
    my_pio_sm_init(pio, SCUART_SM_RX, sc_uart_offset_rx_start, &cfg_rx);

#if 1
    pio->sm[SCUART_SM_TX].instr = 0xe001; // set    pins, 1
    pio->sm[SCUART_SM_TX].instr = 0xe080; // set pindirs, 0
#endif
    
    my_pio_sm_set_enabled(pio, SCUART_SM_TX, true);
    my_pio_sm_set_enabled(pio, SCUART_SM_RX, true);
}

#endif

static inline void
pio_set_irq(pio_hw_t* pio, uint32_t source)
{
    hw_set_bits(&pio->inte0, source);
}

static inline void
pio_clear_irq(pio_hw_t* pio, uint32_t source)
{
    hw_clear_bits(&pio->inte0, source);
}

#if 0
static inline uint32_t
irq_disable(void)
{
    uint32_t status;
    pico_default_asm_volatile(
            "mrs %0, PRIMASK\n"
            "cpsid i"
            : "=r" (status) ::);
    return status;
    // return save_and_disable_interrupts();
}

static inline void
irq_enable(uint32_t status)
{
    pico_default_asm_volatile("msr PRIMASK,%0"::"r" (status) : );
    // restore_interrupts(status);
}
#endif

// rp2040 helper function to clear a hardware reset bit
static void
rp2040_clear_reset(uint32_t reset_bit)
{
    if (resets_hw->reset & reset_bit) {
        hw_clear_bits(&resets_hw->reset, reset_bit);
        while (!(resets_hw->reset_done & reset_bit))
            ;
    }
}

void
scuart_hw_init(struct scuart_hw* u, uint32_t pio_num, uint32_t pin, uint32_t baud)
{
    rp2040_clear_reset(pio_num
		       ? RESETS_RESET_PIO1_BITS
		       : RESETS_RESET_PIO0_BITS);
    
    pio_hw_t* pio = pio_num ? pio1_hw : pio0_hw;

    uint32_t i;
    for (i=0; i<ARRAY_SIZE(sc_uart_program_instructions); i++)
        pio->instr_mem[i] = sc_uart_program_instructions[i];

    scuart_hw_program_init(pio, pin, baud);

    u->pin = pin;
    u->pio = pio;
    u->flags = 0;
    u->wr_len = u->wr_pos = 0;
    u->rd_len = 0;

    // Set pio to tell us when the FIFO is NOT empty
    pio_set_irq(pio, SCUART_IRQ_RXNEMPTY);
    pio_set_irq(pio, SCUART_IRQ_TX_COMPLETE);
}

static inline bool
is_tx_fifo_full(pio_hw_t* pio)
{
    return (pio->fstat & (1u << (PIO_FSTAT_TXFULL_LSB + SCUART_SM_TX))) != 0;
}

static inline bool
is_tx_fifo_empty(pio_hw_t* pio)
{
    return (pio->fstat & (1u << (PIO_FSTAT_TXEMPTY_LSB + SCUART_SM_TX))) != 0;
}

static inline void
tx_fifo_push(pio_hw_t* pio, uint8_t c)
{
    pio->txf[SCUART_SM_TX] = c;
}

static inline uint8_t
tx_fifo_write(pio_hw_t* pio, uint8_t len, uint8_t* data)
{
    uint8_t i = 0;
    while (i < len && !is_tx_fifo_full(pio)) {
        tx_fifo_push(pio, data[i]);
        ++i;
    }
    return i;
}

void
scuart_hw_send_unsafe(struct scuart_hw* hw, uint8_t len, uint8_t* data)
{
    // can't start a write when we're doing a write already!
    if (hw->flags & SCUART_FLAG_WRITE)
        return;

    if (len >= sizeof(hw->wr_data))
        return;

    hw->flags |= SCUART_FLAG_WRITE;

    hw->wr_data[0] = len;
    memcpy(hw->wr_data+1, data, len);
    hw->wr_len = len+1;

    pio_hw_t* pio = hw->pio;
    
    hw->wr_pos = tx_fifo_write(pio, hw->wr_len, hw->wr_data);
        
    if (hw->wr_pos != hw->wr_len)
        pio_set_irq(pio, SCUART_IRQ_TXNFULL);
}

void
scuart_hw_send(struct scuart_hw* u, uint8_t len, uint8_t* data)
{
    // uint32_t save = irq_disable();

    scuart_hw_send_unsafe(u, len, data);
    
    // irq_enable(save);
}

static inline bool
is_rx_fifo_empty(pio_hw_t* pio)
{
    return (pio->fstat & (1u << (PIO_FSTAT_RXEMPTY_LSB + SCUART_SM_RX))) != 0;
}

static inline uint8_t
rx_fifo_pop(pio_hw_t* pio)
{
    return ((io_rw_8*)&pio->rxf[SCUART_SM_RX])[3];
}

static uint8_t
scuart_hw_recv_unsafe(struct scuart_hw* hw, uint8_t len, uint8_t* data)
{
    uint8_t ret = hw->rd_len;
    if (data)
        memcpy(data, hw->rd_data, len < ret ? len : ret );
    hw->rd_len = 0;
    return ret;
}

uint8_t
scuart_hw_recv(struct scuart_hw* u, uint8_t len, uint8_t* data)
{
    // uint32_t save = irq_disable();

    uint8_t ret = scuart_hw_recv_unsafe(u, len, data);
    
    // irq_enable(save);

    return ret;
}

void
scuart_hw_irq_handler(struct scuart_hw* hw)
{
    pio_hw_t* pio = hw->pio;
    uint32_t ints = pio->ints0;

    if (ints & SCUART_IRQ_TXNFULL) {

        uint8_t len = hw->wr_len - hw->wr_pos;
        uint8_t* data = hw->wr_data + hw->wr_pos;
        hw->wr_pos += tx_fifo_write(pio, len, data);
        
        if (hw->wr_pos == hw->wr_len)
            pio_clear_irq(pio, SCUART_IRQ_TXNFULL);
    }
    
    if (ints & SCUART_IRQ_RXNEMPTY) {

        while (!is_rx_fifo_empty(pio)) {
            uint8_t c = rx_fifo_pop(pio);
            if (hw->rd_len < sizeof(hw->rd_data)) {
                hw->rd_data[hw->rd_len] = c;
                hw->rd_len += 1;
            }
            else {
                // hw->rd_overflow += 1;
            }
        }
    }
    
    if (ints & SCUART_IRQ_TX_COMPLETE) {

        if (hw->callback)
            hw->callback(hw, SCUART_NOTIFY_TX);
        hw->flags &= ~SCUART_FLAG_WRITE;
        // clear the PIO interrupt, note this is done by state machine number,
        // not by interrupt bit
        pio->irq = 1u << SCUART_SM_TX;
    }
}
