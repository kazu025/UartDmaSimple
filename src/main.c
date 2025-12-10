#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/regs/uart.h"
#include "hardware/regs/dreq.h"
#include "hardware/sync.h"

#define UART_ID       uart0
#define UART_BASE     UART0_BASE
#define BAUD_RATE     115200
#define UART_TX_PIN   0
#define UART_RX_PIN   1

#define RX_BUF_SIZE   256   // 2^N 
#define TX_BUF_SIZE   256   // 2^N 

/* RXリングバッファ */
static uint8_t rx_buf[RX_BUF_SIZE];

/* TXリングバッファ */
static uint8_t tx_buf[TX_BUF_SIZE];
static int dma_rx_chan = -1;
static int dma_tx_chan = -1;

/* TX書込み/読込位置ポインタ：割り込み処理で共有するのでvolatile */
static volatile uint16_t tx_head = 0;   // 書き込み位置
static volatile uint16_t tx_tail = 0;   // 読み出し位置

/* TX DMAで現在転送中のバイト数(IRQでクリアしてtailを進める)*/
static volatile uint32_t tx_dma_active_count = 0;

/* このフラグはIRQ内外で使う簡易ロック */
static volatile uint32_t tx_dma_running = false;

/* 初期化済みフラグ */
static bool uart_dma_inited = false;

static inline uint16_t mask_idx(uint16_t v, uint16_t size){
	return v & (size - 1);
}

/* --- LED --- */
int led_init(void){
	if(cyw43_arch_init()){
		printf("cyw_43_init() error!\n");
		return -1;
	}
	return 0;
}
void led_sw(void){
	static bool flag = true;
	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, flag);
	flag = !flag;
}
/* -------- */
/* DMA IRQ ハンドラ
 -	TX チャネルの完了 IRQ を受け取り、
	tail を進め、残データがあれば次の DMA を起動
*/
void dma_irq_handler(void){
	uint32_t ints = dma_hw->ints0;
	// TX DMA完了割り込みチェック
	if(ints & (1u << dma_tx_chan)){
		dma_hw->ints0 = (1u << dma_tx_chan);
		if(tx_dma_active_count){
			uint32_t save = save_and_disable_interrupts();
			tx_tail = (tx_tail + tx_dma_active_count) & (TX_BUF_SIZE -1 );
			tx_dma_active_count = 0;
			tx_dma_running = false;
			restore_interrupts(save);
		}
		/* 次のチャンクがあれば開始する（ここで再度安全にチェックして起動）
           計算は割り込み無効化済み領域でやるのが安全だが、
           ここはISRなのでそのまま読み出してもOK（tx_head は IRQ 外で更新されるがatomic性は確保されている想定） */
		uint16_t head = tx_head;
		uint16_t tail = tx_tail;
		uint32_t remaining = (head >= tail) ? (head - tail) : (TX_BUF_SIZE - tail);
		if(remaining > 0){
			tx_dma_active_count = remaining;
			dma_channel_set_read_addr(dma_tx_chan, &tx_buf[tail], false);
			dma_channel_set_write_addr(dma_tx_chan, (void*)(UART_BASE + UART_UARTDR_OFFSET), false);
			// start transfer
			tx_dma_running = true;
			dma_channel_set_trans_count(dma_tx_chan, remaining, true);
		}else{
			tx_dma_running = false; // nothing left
		}
	}
}
/*
 UART DMA初期化
 - RX:循環モード(ring)で常時DMA実行
 - TX:idleで待機、必要時のみstartする
*/
void init_uart_dma(void){
	if(uart_dma_inited) return;

	uart_init(UART_ID, BAUD_RATE);
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	gpio_pull_up(UART_RX_PIN);
	// UART FIFO ON
	uart_set_fifo_enabled(UART_ID, true);
	// -- 最初のゴミ(0xFF)対策
	uart_getc(UART_ID);
	while(uart_is_readable(UART_ID)) uart_getc(UART_ID);
	sleep_ms(10);
	// --
	uart_set_hw_flow(UART_ID, false, false);
	/* === RX DMA (ring) === */
    dma_rx_chan = dma_claim_unused_channel(true);
    dma_channel_config cr = dma_channel_get_default_config(dma_rx_chan);

    channel_config_set_transfer_data_size(&cr, DMA_SIZE_8);
    channel_config_set_read_increment(&cr, false);				// read from UART DR
    channel_config_set_write_increment(&cr, true);				// write to buffer
    channel_config_set_dreq(&cr, uart_get_dreq(UART_ID, false));// RX DREQ

    channel_config_set_ring(&cr, true, 8); // 256(2^8) バイト

	// configure and start the RX DMA in circular mode
	dma_channel_configure(
        dma_rx_chan,
        &cr,
        rx_buf,                                       // write addr
        (const void *)(UART_BASE + UART_UARTDR_OFFSET),    // read addr
        RX_BUF_SIZE,
        true
    );
	/* === TX DMA === */
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_channel_config ct = dma_channel_get_default_config(dma_tx_chan);

    channel_config_set_transfer_data_size(&ct, DMA_SIZE_8);
    channel_config_set_read_increment(&ct, true);	// read from buffer
    channel_config_set_write_increment(&ct, false);	// write to UART DR
    channel_config_set_dreq(&ct, uart_get_dreq(UART_ID, true)); // TX DREQ
	
	// configure and dont start the TX DMA(count 0)
	dma_channel_configure(dma_tx_chan, &ct,
		(void*)(UART_BASE + UART_UARTDR_OFFSET),	// write addr (UART)
		tx_buf,										// read addr
		0,											// count = 0 まだ送らない
		false);										// start しない
    // DMA IRQ: TX転送完了時次のチャンクを起動する
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_tx_chan, true);
	// RXチャネルIRQは不要（リングバッファに常に書き込む）
    dma_channel_set_irq0_enabled(dma_rx_chan, false);
	
	uart_dma_inited = true;
}

/*
RX の読み出し（メインループで呼ぶ）
  - DMA の write_addr を直接読んで「現在どこまで来ているか」を算出する
  - last_pos は関数内静的保持（呼び出しはシングルスレッド想定）
*/
int uart_dma_read_byte(void){
	static uint16_t last_pos = 0;

	uint32_t waddr = dma_hw->ch[dma_rx_chan].write_addr - (uintptr_t)rx_buf;
	uint16_t now_pos = waddr & (RX_BUF_SIZE - 1);
	if(last_pos == now_pos) { return -1; }
	uint8_t ch = rx_buf[last_pos];
	last_pos = (last_pos + 1) & (RX_BUF_SIZE - 1);
	return ch;
}
/*
TX の書き込み（アプリが使う）
  - バッファへ1バイト書き込み、DMAがidleなら即座に1チャンクを起動
  - head/tail更新は割り込み保護（save_and_disable_interrupts）して行う
*/
void uart_dma_write_byte(uint8_t b){
	uint16_t next_head;
	while(true){
		uint32_t save = save_and_disable_interrupts();
		next_head = (tx_head + 1) & (TX_BUF_SIZE - 1);
		// バッファフルなら待つ(実装に応じてブロッキングor失敗)
		if(next_head == tx_tail){
			restore_interrupts(save);
			tight_loop_contents(); // or sleep_ms(1);
			continue;
		}
		// 書込み
		tx_buf[tx_head] = b;
		tx_head = next_head;
		// もし、DMAがidleならキック
		bool need_start = !tx_dma_running;
		if(need_start){
			uint16_t head = tx_head;
			uint16_t tail = tx_tail;
			uint32_t count = (head >= tail) ? (head - tail) : (TX_BUF_SIZE - tail);
			if(count > 0){
				tx_dma_active_count = count; // IRQで進める
				dma_channel_set_read_addr(dma_tx_chan, &tx_buf[tx_tail], false);
				dma_channel_set_write_addr(dma_tx_chan, (void*)(UART_BASE + UART_UARTDR_OFFSET), false);
				tx_dma_running = true;
				dma_channel_set_trans_count(dma_tx_chan, count, true);
			}else{
				tx_dma_running = false;
			}
		}
		restore_interrupts(save);
		break;
	}
}

void uart_dma_write_string(const char *s){
	while(*s) uart_dma_write_byte(*s++);
}

// 簡易テスト：FIFO ON、ポーリングで受信確認
void test_uart_polling(void){
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_pull_up(UART_RX_PIN);
    uart_set_fifo_enabled(UART_ID, true); // ON
    printf("polling test start\r\n");
    while (1) {
        if (uart_is_readable(UART_ID)) {
            int c = uart_getc(UART_ID);
            printf("recv: %c (0x%02x)\r\n", (char)c, c);
        }
        tight_loop_contents();
    }
}

// ループバックテスト。GPIO0（TX）を物理的にGPIO1（RX）に接続して行う。
void loopback_test(void){
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(UART_ID, true);
    printf("loopback test start\r\n");
	sleep_ms(10);
	int c = uart_getc(UART_ID);
	printf("pre recv: %c\n", c);
	uart_putc(UART_ID, 'A');
    sleep_ms(10);
	if (uart_is_readable(UART_ID)) {
		int c = uart_getc(UART_ID);
		printf("loopback recv: %c\n", c);
	} else {
		printf("loopback no recv\n");
	}
    while(1) tight_loop_contents();
}

/*
	main
*/
int main(void){
	stdio_init_all();
	led_init();
	sleep_ms(2000);
//	test_uart_polling();
//	loopback_test();
	init_uart_dma();
	printf("start!\r\n");
	while(1){
		int ch = uart_dma_read_byte();
		if(ch >= 0){
			uart_dma_write_byte((uint8_t)ch);
			printf("read byte: %c\r\n", ch);
			led_sw();
		}
		tight_loop_contents();
	}
}
















