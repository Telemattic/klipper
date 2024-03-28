#include <string.h>
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "internal.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN
#include "scuart_hw.h"

#include "hardware/structs/pio.h" // pio0_hw
#include "hardware/structs/resets.h"

enum {
    SU_IDLE,
    SU_SEND,
    SU_RECV
};

struct scuart_s {
    struct scuart_hw hw;
    struct timer timer;
    uint8_t state;
    uint32_t num_interrupts;
};

static struct task_wake scuart_wake;
static struct scuart_s* uarts = 0;

void
PIOx_IRQHandler(void)
{
    scuart_hw_irq_handler(&uarts->hw);
    uarts->num_interrupts += 1;
}

static void
scuart_output_debug_info(struct scuart_s* u)
{
    output("scuart: hw(fl=%c wp=%c wl=%c rl=%c) st=%c ni=%u",
	   u->hw.flags, u->hw.wr_pos, u->hw.wr_len, u->hw.rd_len,
	   u->state, u->num_interrupts);
#if 0
    pio_hw_t* pio = u->hw.pio;
    output("scuart: fstat=%u fdebug=%u flevel=%u reset=%u reset_done=%u",
	   pio->fstat, pio->fdebug, pio->flevel,
	   resets_hw->reset, resets_hw->reset_done);
#endif
}

void
command_config_scuart(uint32_t* args)
{
    if (uarts)
	shutdown("scuart already configured");
    
    struct scuart_s* u = oid_alloc(args[0], command_config_scuart
                                    , sizeof(*u));
    uint32_t pio_num = args[1];
    uint32_t pin = args[2];
    uint32_t baud = args[3];

    scuart_hw_init(&u->hw, pio_num, pin, baud);
    u->state = SU_IDLE;
    u->num_interrupts = 0;
    uarts = u;

    // Enable irqs
    armcm_enable_irq(PIOx_IRQHandler, PIO0_IRQ_0_IRQn, 1);

    output("scuart: oid=%u pio=%u pin=%u baud=%u", args[0], pio_num, pin, baud);

    scuart_output_debug_info(u);
}
DECL_COMMAND(command_config_scuart,
             "config_scuart oid=%c pio=%u pin=%u baud=%u");

static uint_fast8_t
scuart_flag_recv(struct timer* timer)
{
    struct scuart_s* u = container_of(timer, struct scuart_s, timer);
    u->state = SU_RECV;
    sched_wake_task(&scuart_wake);
    return SF_DONE;
}

void
command_scuart_send(uint32_t* args)
{
    struct scuart_s* u = oid_lookup(args[0], command_config_scuart);
    uint8_t tx_len = args[1];
    uint8_t* tx_data = command_decode_ptr(args[2]);

    if (u->state != SU_IDLE)
	return;

    scuart_output_debug_info(u);

    u->state = SU_SEND;
    scuart_hw_send(&u->hw, tx_len, tx_data);

    irq_disable();
    u->timer.func = scuart_flag_recv;
    u->timer.waketime = timer_read_time() + timer_from_us(500);
    sched_add_timer(&u->timer);
    irq_enable();

    scuart_output_debug_info(u);
}
DECL_COMMAND(command_scuart_send, "scuart_send oid=%c write=%*s");

// Report completed response message back to host
void
scuart_task(void)
{
    if (!sched_check_wake(&scuart_wake))
        return;

    uint8_t oid;
    struct scuart_s* u;
    foreach_oid(oid, u, command_config_scuart) {
    
	scuart_output_debug_info(u);
	
	if (SU_RECV != u->state)
	    continue;
	
	struct scuart_hw* hw = &u->hw;
	irq_disable();
	uint8_t len = hw->rd_len;
	uint8_t data[sizeof(hw->rd_data)];
	memcpy(data, hw->rd_data, sizeof(hw->rd_data));
	hw->rd_len = 0;
	u->state = SU_IDLE;
	irq_enable();
	
	sendf("scuart_response oid=%c read=%*s"
	      , oid, len, data);
	
	scuart_output_debug_info(u);
    }
}
DECL_TASK(scuart_task);

void
scuart_shutdown(void)
{
}
DECL_SHUTDOWN(scuart_shutdown);
