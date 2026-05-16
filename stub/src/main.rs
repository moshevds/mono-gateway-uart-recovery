#![no_std]
#![no_main]

use core::arch::{asm, global_asm};
use core::hint::spin_loop;
use core::panic::PanicInfo;
use core::ptr::{read_volatile, write_volatile};

use mono_uart_recovery_protocol as proto;

const UART_BASE: usize = 0x021c_0500;

const UART_RBR: usize = 0;
const UART_THR: usize = 0;
const UART_DLL: usize = 0;
const UART_DLM: usize = 1;
const UART_IER: usize = 1;
const UART_FCR: usize = 2;
const UART_LCR: usize = 3;
const UART_MCR: usize = 4;
const UART_LSR: usize = 5;

const UART_LCR_DLAB: u8 = 0x80;
const UART_LSR_DR: u8 = 0x01;
const UART_LSR_THRE: u8 = 0x20;
const UART_LSR_TEMT: u8 = 0x40;
const DEFAULT_UART_DLL: u8 = 0xa3;
const DEFAULT_UART_DLM: u8 = 0x00;

const STAGE1_LOAD_ADDR: u32 = proto::STAGE1_LOAD_ADDR;
const STAGE1_ENTRY_ADDR: u32 = proto::STAGE1_ENTRY_ADDR;
const STAGE1_MAX_LEN: u32 = proto::STAGE1_MAX_LEN;
const STUB_READY_INTERVAL: u32 = 250_000;
const CACHE_LINE_SIZE: u32 = 64;

global_asm!(
    r#"
    .section .text._start, "ax"
    .global _start
_start:
    ldr x0, =__stack_top
    mov sp, x0
    ldr x0, =__bss_start
    ldr x1, =__bss_end
    mov x2, xzr
1:
    cmp x0, x1
    b.hs 2f
    str x2, [x0], #8
    b 1b
2:
    bl rust_main
3:
    wfe
    b 3b
"#
);

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        spin_loop();
    }
}

struct LoadState {
    len: u32,
    received: u32,
    active: bool,
}

#[no_mangle]
extern "C" fn rust_main() -> ! {
    uart_init();
    send_ready();

    let mut decoder = proto::SlipDecoder::<{ proto::MAX_FRAME_LEN }>::new();
    let mut state = LoadState {
        len: 0,
        received: 0,
        active: false,
    };
    let mut idle_ticks = 0u32;

    loop {
        if let Some(byte) = uart_try_get() {
            idle_ticks = 0;
            match decoder.push(byte) {
                Ok(Some(_)) => {
                    let input = decoder.frame();
                    handle_frame(input, &mut state);
                    decoder.reset();
                }
                Ok(None) => {}
                Err(_) => {
                    decoder.reset();
                    send_simple(0, proto::OP_READY, proto::STATUS_BAD_FRAME);
                }
            }
        } else {
            idle_ticks = idle_ticks.wrapping_add(1);
            if idle_ticks == STUB_READY_INTERVAL {
                send_ready();
                idle_ticks = 0;
            }
            spin_loop();
        }
    }
}

fn handle_frame(input: &[u8], state: &mut LoadState) {
    let Ok((header, payload)) = proto::parse_frame(input) else {
        send_simple(0, proto::OP_READY, proto::STATUS_BAD_FRAME);
        return;
    };

    match header.op {
        proto::OP_HELLO => send_ready(),
        proto::OP_LOAD_BEGIN => handle_load_begin(header.seq, payload, state),
        proto::OP_LOAD_CHUNK => handle_load_chunk(header.seq, payload, state),
        proto::OP_EXEC => {
            let _ = handle_exec(header.seq, state);
        }
        proto::OP_RESET => {
            send_simple(header.seq, header.op, proto::STATUS_OK);
            loop {
                spin_loop();
            }
        }
        _ => send_simple(header.seq, header.op, proto::STATUS_UNSUPPORTED),
    }
}

fn handle_load_begin(seq: u8, payload: &[u8], state: &mut LoadState) {
    let (Ok(load_addr), Ok(len), Ok(entry_addr)) = (
        proto::get_u32(payload, 0),
        proto::get_u32(payload, 4),
        proto::get_u32(payload, 8),
    ) else {
        send_simple(seq, proto::OP_LOAD_BEGIN, proto::STATUS_BAD_REQUEST);
        return;
    };

    if load_addr != STAGE1_LOAD_ADDR
        || entry_addr != STAGE1_ENTRY_ADDR
        || len == 0
        || len > STAGE1_MAX_LEN
    {
        send_simple(seq, proto::OP_LOAD_BEGIN, proto::STATUS_RANGE);
        return;
    }

    state.len = len;
    state.received = 0;
    state.active = true;
    send_simple(seq, proto::OP_LOAD_BEGIN, proto::STATUS_OK);
}

fn handle_load_chunk(seq: u8, payload: &[u8], state: &mut LoadState) {
    let Ok(offset) = proto::get_u32(payload, 0) else {
        send_simple(seq, proto::OP_LOAD_CHUNK, proto::STATUS_BAD_REQUEST);
        return;
    };
    let data = if payload.len() >= 4 {
        &payload[4..]
    } else {
        &[]
    };
    let end = match offset.checked_add(data.len() as u32) {
        Some(end) => end,
        None => {
            send_simple(seq, proto::OP_LOAD_CHUNK, proto::STATUS_RANGE);
            return;
        }
    };

    if !state.active || data.is_empty() || offset != state.received || end > state.len {
        send_simple(seq, proto::OP_LOAD_CHUNK, proto::STATUS_RANGE);
        return;
    }

    let dst = (STAGE1_LOAD_ADDR + offset) as *mut u8;
    for (idx, byte) in data.iter().copied().enumerate() {
        unsafe { write_volatile(dst.add(idx), byte) };
    }
    state.received = end;
    send_simple(seq, proto::OP_LOAD_CHUNK, proto::STATUS_OK);
}

fn handle_exec(seq: u8, state: &LoadState) -> bool {
    if !state.active || state.received != state.len {
        send_simple(seq, proto::OP_EXEC, proto::STATUS_BAD_REQUEST);
        return false;
    }

    send_simple(seq, proto::OP_EXEC, proto::STATUS_OK);
    uart_wait_empty();
    sync_loaded_code(STAGE1_LOAD_ADDR, state.len);

    let entry: extern "C" fn() -> ! = unsafe { core::mem::transmute(STAGE1_ENTRY_ADDR as usize) };
    entry()
}

fn send_ready() {
    send_simple(0, proto::OP_READY, proto::STATUS_OK);
}

fn send_simple(seq: u8, op: u8, status: u8) {
    let mut frame_buf = [0u8; proto::HEADER_LEN];
    let mut slip_buf = [0u8; proto::HEADER_LEN * 2 + 2];
    let Ok(frame_len) = proto::build_frame(&mut frame_buf, seq, op, status, &[]) else {
        return;
    };
    let Ok(slip_len) = proto::slip_encode(&frame_buf[..frame_len], &mut slip_buf) else {
        return;
    };
    for &byte in &slip_buf[..slip_len] {
        uart_put(byte);
    }
}

fn sync_loaded_code(addr: u32, len: u32) {
    let mut current = addr & !(CACHE_LINE_SIZE - 1);
    let end = (addr + len + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);

    while current < end {
        unsafe { asm!("dc cvau, {}", in(reg) current as usize) };
        current += CACHE_LINE_SIZE;
    }
    unsafe { asm!("dsb ish") };

    current = addr & !(CACHE_LINE_SIZE - 1);
    while current < end {
        unsafe { asm!("ic ivau, {}", in(reg) current as usize) };
        current += CACHE_LINE_SIZE;
    }
    unsafe {
        asm!("dsb ish");
        asm!("isb");
    }
}

fn uart_init() {
    uart_configure(
        DEFAULT_UART_DLL,
        DEFAULT_UART_DLM,
        proto::UART_LCR_WLEN8,
        proto::UART_FCR_ENABLE_CLEAR,
        proto::UART_MCR_DTR_RTS,
    );
}

fn uart_configure(dll: u8, dlm: u8, lcr: u8, fcr: u8, mcr: u8) {
    uart_write(UART_IER, 0x00);
    uart_write(UART_LCR, UART_LCR_DLAB);
    uart_write(UART_DLL, dll);
    uart_write(UART_DLM, dlm);
    uart_write(UART_LCR, lcr);
    uart_write(UART_FCR, fcr);
    uart_write(UART_MCR, mcr);
}

fn uart_wait_empty() {
    while uart_read(UART_LSR) & UART_LSR_TEMT == 0 {
        spin_loop();
    }
}

fn uart_put(byte: u8) {
    while uart_read(UART_LSR) & UART_LSR_THRE == 0 {
        spin_loop();
    }
    uart_write(UART_THR, byte);
}

fn uart_try_get() -> Option<u8> {
    if uart_read(UART_LSR) & UART_LSR_DR == 0 {
        None
    } else {
        Some(uart_read(UART_RBR))
    }
}

fn uart_read(offset: usize) -> u8 {
    unsafe { read_volatile((UART_BASE + offset) as *const u8) }
}

fn uart_write(offset: usize, value: u8) {
    unsafe { write_volatile((UART_BASE + offset) as *mut u8, value) };
}
