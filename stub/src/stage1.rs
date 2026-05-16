#![no_std]
#![no_main]

use core::arch::global_asm;
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

const QSPI_BASE: usize = 0x0155_0000;
const QSPI_AHB_BASE: usize = 0x4000_0000;
const QSPI_AHB_BUFFER_SIZE: usize = 1024;

const QSPI_MCR: usize = 0x000;
const QSPI_IPCR: usize = 0x008;
const QSPI_BUF3CR: usize = 0x01c;
const QSPI_BFGENCR: usize = 0x020;
const QSPI_BUF0IND: usize = 0x030;
const QSPI_BUF1IND: usize = 0x034;
const QSPI_BUF2IND: usize = 0x038;
const QSPI_SFAR: usize = 0x100;
const QSPI_SMPR: usize = 0x108;
const QSPI_RBCT: usize = 0x110;
const QSPI_TBDR: usize = 0x154;
const QSPI_SR: usize = 0x15c;
const QSPI_SPTRCLR: usize = 0x16c;
const QSPI_SFA1AD: usize = 0x180;
const QSPI_SFA2AD: usize = 0x184;
const QSPI_SFB1AD: usize = 0x188;
const QSPI_SFB2AD: usize = 0x18c;
const QSPI_RBDR: usize = 0x200;
const QSPI_LUTKEY: usize = 0x300;
const QSPI_LCKCR: usize = 0x304;
const QSPI_LUT_BASE: usize = 0x310;

const QSPI_MCR_RESERVED_MASK: u32 = 0x000f_0000;
const QSPI_MCR_MDIS: u32 = 1 << 14;
const QSPI_MCR_CLR_TXF: u32 = 1 << 11;
const QSPI_MCR_CLR_RXF: u32 = 1 << 10;
const QSPI_MCR_END_CFG_MASK: u32 = 0x0000_000c;
const QSPI_MCR_SWRSTHD: u32 = 1 << 1;
const QSPI_MCR_SWRSTSD: u32 = 1 << 0;
const QSPI_SR_IP_ACC: u32 = 1 << 1;
const QSPI_SR_AHB_ACC: u32 = 1 << 2;
const QSPI_SPTRCLR_IPPTRC: u32 = 1 << 8;
const QSPI_SPTRCLR_BFPTRC: u32 = 1 << 0;
const QSPI_RBCT_WMRK_MASK: u32 = 0x1f;
const QSPI_RBCT_RXBRD_USEIPS: u32 = 1 << 8;
const QSPI_BUF3CR_ALLMST: u32 = 1 << 31;
const QSPI_LUTKEY_VALUE: u32 = 0x5af0_5af0;
const QSPI_LCKCR_LOCK: u32 = 1;
const QSPI_LCKCR_UNLOCK: u32 = 2;

const SEQID: u32 = 15;
const LUT_OFFSET: usize = SEQID as usize * 4 * 4;
const LUT_STOP: u32 = 0;
const LUT_CMD: u32 = 1;
const LUT_DUMMY: u32 = 3;
const LUT_MODE: u32 = 4;
const LUT_READ: u32 = 7;
const LUT_WRITE: u32 = 8;

const CMD_READ_ID: u8 = 0x9f;
const CMD_WRITE_ENABLE: u8 = 0x06;
const CMD_READ_STATUS: u8 = 0x05;
const CMD_FAST_READ_4B: u8 = 0x0c;
const CMD_PAGE_PROGRAM_4B: u8 = 0x12;
const CMD_SECTOR_ERASE_4B: u8 = 0xdc;

const FLASH_PAGE_SIZE: u32 = 256;
const STUB_MAX_DATA: usize = proto::MAX_DATA_LEN;
const STUB_READY_INTERVAL: u32 = 250_000;
const DEBUG_MAX_LEN: usize = 96;

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

#[no_mangle]
extern "C" fn rust_main() -> ! {
    uart_init();
    send_debug_static(b"stage1 boot");
    qspi_init();
    send_debug_static(b"qspi init done");

    let jedec_id = qspi_read_jedec_id().unwrap_or(0);
    send_debug_jedec(jedec_id);
    send_ready(jedec_id);

    let mut decoder = proto::SlipDecoder::<{ proto::MAX_FRAME_LEN }>::new();
    let mut frame_buf = [0u8; proto::MAX_FRAME_LEN];
    let mut payload_buf = [0u8; STUB_MAX_DATA + 8];
    let mut slip_buf = [0u8; proto::MAX_SLIP_LEN];
    let mut idle_ticks = 0u32;

    loop {
        if let Some(byte) = uart_try_get() {
            idle_ticks = 0;
            match decoder.push(byte) {
                Ok(Some(_)) => {
                    let input = decoder.frame();
                    handle_frame(
                        input,
                        &mut frame_buf,
                        &mut payload_buf,
                        &mut slip_buf,
                        jedec_id,
                    );
                    decoder.reset();
                }
                Ok(None) => {}
                Err(_) => {
                    decoder.reset();
                    send_debug_static(b"slip decode error");
                    send_simple(
                        0,
                        proto::OP_READY,
                        proto::STATUS_BAD_FRAME,
                        &[],
                        &mut frame_buf,
                        &mut slip_buf,
                    );
                }
            }
        } else {
            idle_ticks = idle_ticks.wrapping_add(1);
            if idle_ticks == STUB_READY_INTERVAL {
                send_ready(jedec_id);
                idle_ticks = 0;
            }
            spin_loop();
        }
    }
}

fn handle_frame(
    input: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    payload_buf: &mut [u8; STUB_MAX_DATA + 8],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
    jedec_id: u32,
) {
    let Ok((header, payload)) = proto::parse_frame(input) else {
        send_debug_static(b"bad frame header");
        send_simple(
            0,
            proto::OP_READY,
            proto::STATUS_BAD_FRAME,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };

    match header.op {
        proto::OP_HELLO => {
            let info = proto::DeviceInfo::gateway_dk(jedec_id);
            let Ok(len) = info.encode(payload_buf) else {
                send_simple(
                    header.seq,
                    header.op,
                    proto::STATUS_BAD_REQUEST,
                    &[],
                    frame_buf,
                    slip_buf,
                );
                return;
            };
            send_simple(
                header.seq,
                header.op,
                proto::STATUS_OK,
                &payload_buf[..len],
                frame_buf,
                slip_buf,
            );
        }
        proto::OP_INFO => {
            let info = proto::DeviceInfo::gateway_dk(jedec_id);
            let Ok(len) = info.encode(payload_buf) else {
                send_simple(
                    header.seq,
                    header.op,
                    proto::STATUS_BAD_REQUEST,
                    &[],
                    frame_buf,
                    slip_buf,
                );
                return;
            };
            send_simple(
                header.seq,
                header.op,
                proto::STATUS_OK,
                &payload_buf[..len],
                frame_buf,
                slip_buf,
            );
        }
        proto::OP_READ => handle_read(header.seq, payload, frame_buf, payload_buf, slip_buf),
        proto::OP_WRITE => handle_write(header.seq, payload, frame_buf, slip_buf),
        proto::OP_ERASE => handle_erase(header.seq, payload, frame_buf, slip_buf),
        proto::OP_CRC32 => handle_crc32(header.seq, payload, frame_buf, payload_buf, slip_buf),
        proto::OP_CONFIG_UART => {
            handle_config_uart(header.seq, payload, frame_buf, slip_buf, jedec_id)
        }
        proto::OP_RESET => {
            send_simple(
                header.seq,
                header.op,
                proto::STATUS_OK,
                &[],
                frame_buf,
                slip_buf,
            );
            loop {
                spin_loop();
            }
        }
        _ => send_simple(
            header.seq,
            header.op,
            proto::STATUS_UNSUPPORTED,
            &[],
            frame_buf,
            slip_buf,
        ),
    }
}

fn handle_read(
    seq: u8,
    payload: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    payload_buf: &mut [u8; STUB_MAX_DATA + 8],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
) {
    let Ok(offset) = proto::get_u32(payload, 0) else {
        send_simple(
            seq,
            proto::OP_READ,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    let Ok(len) = proto::get_u16(payload, 4) else {
        send_simple(
            seq,
            proto::OP_READ,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    let len = len as usize;
    if len > STUB_MAX_DATA || !range_ok(offset, len as u32) {
        send_simple(
            seq,
            proto::OP_READ,
            proto::STATUS_RANGE,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    proto::put_u32(payload_buf, 0, offset);
    if qspi_read(offset, &mut payload_buf[4..4 + len]).is_err() {
        send_simple(
            seq,
            proto::OP_READ,
            proto::STATUS_FLASH,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    send_simple(
        seq,
        proto::OP_READ,
        proto::STATUS_OK,
        &payload_buf[..4 + len],
        frame_buf,
        slip_buf,
    );
}

fn handle_write(
    seq: u8,
    payload: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
) {
    let Ok(offset) = proto::get_u32(payload, 0) else {
        send_simple(
            seq,
            proto::OP_WRITE,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    let data = if payload.len() >= 4 {
        &payload[4..]
    } else {
        &[]
    };
    if data.is_empty() || data.len() > STUB_MAX_DATA || !range_ok(offset, data.len() as u32) {
        send_simple(
            seq,
            proto::OP_WRITE,
            proto::STATUS_RANGE,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    if qspi_write(offset, data).is_err() {
        send_simple(
            seq,
            proto::OP_WRITE,
            proto::STATUS_FLASH,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    send_simple(
        seq,
        proto::OP_WRITE,
        proto::STATUS_OK,
        &[],
        frame_buf,
        slip_buf,
    );
}

fn handle_erase(
    seq: u8,
    payload: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
) {
    let Ok(offset) = proto::get_u32(payload, 0) else {
        send_simple(
            seq,
            proto::OP_ERASE,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    let Ok(len) = proto::get_u32(payload, 4) else {
        send_simple(
            seq,
            proto::OP_ERASE,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };

    if offset % proto::GATEWAY_DK_FLASH_ERASE_SIZE != 0
        || len == 0
        || len % proto::GATEWAY_DK_FLASH_ERASE_SIZE != 0
        || !range_ok(offset, len)
    {
        send_simple(
            seq,
            proto::OP_ERASE,
            proto::STATUS_RANGE,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    let mut current = offset;
    let end = offset + len;
    send_progress(seq, proto::OP_ERASE, 0, len, frame_buf, slip_buf);
    while current < end {
        if qspi_erase_sector(current).is_err() {
            send_simple(
                seq,
                proto::OP_ERASE,
                proto::STATUS_FLASH,
                &[],
                frame_buf,
                slip_buf,
            );
            return;
        }
        current += proto::GATEWAY_DK_FLASH_ERASE_SIZE;
        send_progress(
            seq,
            proto::OP_ERASE,
            current - offset,
            len,
            frame_buf,
            slip_buf,
        );
    }

    send_simple(
        seq,
        proto::OP_ERASE,
        proto::STATUS_OK,
        &[],
        frame_buf,
        slip_buf,
    );
}

fn handle_crc32(
    seq: u8,
    payload: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    payload_buf: &mut [u8; STUB_MAX_DATA + 8],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
) {
    let Ok(offset) = proto::get_u32(payload, 0) else {
        send_simple(
            seq,
            proto::OP_CRC32,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    let Ok(len) = proto::get_u32(payload, 4) else {
        send_simple(
            seq,
            proto::OP_CRC32,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    };
    if !range_ok(offset, len) {
        send_simple(
            seq,
            proto::OP_CRC32,
            proto::STATUS_RANGE,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    let mut state = proto::crc32_init();
    let mut current = offset;
    let end = offset + len;
    let mut last_progress = 0;
    send_progress(seq, proto::OP_CRC32, 0, len, frame_buf, slip_buf);
    while current < end {
        let chunk = min_usize(STUB_MAX_DATA, (end - current) as usize);
        if qspi_read(current, &mut payload_buf[..chunk]).is_err() {
            send_simple(
                seq,
                proto::OP_CRC32,
                proto::STATUS_FLASH,
                &[],
                frame_buf,
                slip_buf,
            );
            return;
        }
        state = proto::crc32_update_state(state, &payload_buf[..chunk]);
        current += chunk as u32;
        let done = current - offset;
        if done == len || done - last_progress >= proto::GATEWAY_DK_FLASH_ERASE_SIZE {
            send_progress(seq, proto::OP_CRC32, done, len, frame_buf, slip_buf);
            last_progress = done;
        }
    }

    proto::put_u32(payload_buf, 0, proto::crc32_finish(state));
    send_simple(
        seq,
        proto::OP_CRC32,
        proto::STATUS_OK,
        &payload_buf[..4],
        frame_buf,
        slip_buf,
    );
}

fn handle_config_uart(
    seq: u8,
    payload: &[u8],
    frame_buf: &mut [u8; proto::MAX_FRAME_LEN],
    slip_buf: &mut [u8; proto::MAX_SLIP_LEN],
    jedec_id: u32,
) {
    if payload.len() != proto::UART_CONFIG_LEN {
        send_simple(
            seq,
            proto::OP_CONFIG_UART,
            proto::STATUS_BAD_REQUEST,
            &[],
            frame_buf,
            slip_buf,
        );
        return;
    }

    send_simple(
        seq,
        proto::OP_CONFIG_UART,
        proto::STATUS_OK,
        payload,
        frame_buf,
        slip_buf,
    );
    uart_wait_empty();
    uart_configure(
        payload[proto::UART_CONFIG_DLL],
        payload[proto::UART_CONFIG_DLM],
        payload[proto::UART_CONFIG_LCR],
        payload[proto::UART_CONFIG_FCR],
        payload[proto::UART_CONFIG_MCR],
    );
    send_ready(jedec_id);
}

fn send_ready(jedec_id: u32) {
    let mut frame_buf = [0u8; proto::HEADER_LEN + proto::DeviceInfo::ENCODED_LEN];
    let mut payload_buf = [0u8; proto::DeviceInfo::ENCODED_LEN];
    let mut slip_buf = [0u8; (proto::HEADER_LEN + proto::DeviceInfo::ENCODED_LEN) * 2 + 2];
    let info = proto::DeviceInfo::gateway_dk(jedec_id);
    let Ok(payload_len) = info.encode(&mut payload_buf) else {
        return;
    };
    send_with_buffers(
        0,
        proto::OP_READY,
        proto::STATUS_OK,
        &payload_buf[..payload_len],
        &mut frame_buf,
        &mut slip_buf,
    );
}

fn send_debug_static(message: &[u8]) {
    let mut frame_buf = [0u8; proto::HEADER_LEN + DEBUG_MAX_LEN];
    let mut slip_buf = [0u8; (proto::HEADER_LEN + DEBUG_MAX_LEN) * 2 + 2];
    let len = min_usize(message.len(), DEBUG_MAX_LEN);
    send_with_buffers(
        0,
        proto::OP_DEBUG_PRINT,
        proto::STATUS_OK,
        &message[..len],
        &mut frame_buf,
        &mut slip_buf,
    );
}

fn send_debug_jedec(jedec_id: u32) {
    let mut message = [0u8; DEBUG_MAX_LEN];
    let prefix = b"jedec=0x";
    message[..prefix.len()].copy_from_slice(prefix);
    write_hex_u32(&mut message[prefix.len()..prefix.len() + 8], jedec_id);
    send_debug_static(&message[..prefix.len() + 8]);
}

fn send_progress(
    seq: u8,
    op: u8,
    done: u32,
    total: u32,
    frame_buf: &mut [u8],
    slip_buf: &mut [u8],
) {
    let mut payload = [0u8; proto::PROGRESS_LEN];
    payload[proto::PROGRESS_OP] = op;
    proto::put_u32(&mut payload, proto::PROGRESS_DONE, done);
    proto::put_u32(&mut payload, proto::PROGRESS_TOTAL, total);
    send_with_buffers(
        seq,
        proto::OP_PROGRESS,
        proto::STATUS_OK,
        &payload,
        frame_buf,
        slip_buf,
    );
}

fn write_hex_u32(out: &mut [u8], value: u32) {
    let hex = b"0123456789abcdef";
    let mut i = 0usize;
    while i < 8 {
        let shift = 28 - (i as u32 * 4);
        out[i] = hex[((value >> shift) & 0x0f) as usize];
        i += 1;
    }
}

fn send_simple(
    seq: u8,
    op: u8,
    status: u8,
    payload: &[u8],
    frame_buf: &mut [u8],
    slip_buf: &mut [u8],
) {
    send_with_buffers(seq, op, status, payload, frame_buf, slip_buf);
}

fn send_with_buffers(
    seq: u8,
    op: u8,
    status: u8,
    payload: &[u8],
    frame_buf: &mut [u8],
    slip_buf: &mut [u8],
) {
    let Ok(frame_len) = proto::build_frame(frame_buf, seq, op, status, payload) else {
        return;
    };
    let Ok(slip_len) = proto::slip_encode(&frame_buf[..frame_len], slip_buf) else {
        return;
    };
    for &byte in &slip_buf[..slip_len] {
        uart_put(byte);
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FlashError {
    Timeout,
    Range,
}

fn qspi_init() {
    qspi_write32(QSPI_MCR, QSPI_MCR_SWRSTSD | QSPI_MCR_SWRSTHD);
    delay(16);
    qspi_write32(QSPI_MCR, QSPI_MCR_MDIS | QSPI_MCR_RESERVED_MASK);

    let smpr = qspi_read32(QSPI_SMPR);
    qspi_write32(QSPI_SMPR, smpr & !0x0007_0061);

    qspi_write32(QSPI_BUF0IND, 0);
    qspi_write32(QSPI_BUF1IND, 0);
    qspi_write32(QSPI_BUF2IND, 0);
    qspi_write32(QSPI_BFGENCR, SEQID << 12);
    qspi_write32(QSPI_RBCT, QSPI_RBCT_WMRK_MASK);
    qspi_write32(
        QSPI_BUF3CR,
        QSPI_BUF3CR_ALLMST | ((QSPI_AHB_BUFFER_SIZE as u32 / 8) << 8),
    );

    let memsize = QSPI_AHB_BUFFER_SIZE as u32;
    let addr_offset = QSPI_AHB_BASE as u32;
    qspi_write32(QSPI_SFA1AD, memsize + addr_offset);
    qspi_write32(QSPI_SFA2AD, memsize * 2 + addr_offset);
    qspi_write32(QSPI_SFB1AD, memsize * 3 + addr_offset);
    qspi_write32(QSPI_SFB2AD, memsize * 4 + addr_offset);

    qspi_write32(QSPI_MCR, QSPI_MCR_RESERVED_MASK | QSPI_MCR_END_CFG_MASK);
}

fn qspi_read_jedec_id() -> Result<u32, FlashError> {
    let mut id = [0u8; 3];
    qspi_ip_read(CMD_READ_ID, 0, 0, 0, &mut id)?;
    Ok(((id[0] as u32) << 16) | ((id[1] as u32) << 8) | id[2] as u32)
}

fn qspi_read(offset: u32, out: &mut [u8]) -> Result<(), FlashError> {
    if out.is_empty() {
        return Ok(());
    }
    if !range_ok(offset, out.len() as u32) {
        return Err(FlashError::Range);
    }

    let mut done = 0usize;
    while done < out.len() {
        let chunk = min_usize(out.len() - done, QSPI_AHB_BUFFER_SIZE);
        qspi_read_ahb(offset + done as u32, &mut out[done..done + chunk])?;
        done += chunk;
    }
    Ok(())
}

fn qspi_read_ahb(offset: u32, out: &mut [u8]) -> Result<(), FlashError> {
    wait_qspi_idle()?;
    qspi_write32(QSPI_SFAR, QSPI_AHB_BASE as u32);
    qspi_write32(
        QSPI_MCR,
        qspi_read32(QSPI_MCR) | QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF,
    );
    qspi_write32(QSPI_SPTRCLR, QSPI_SPTRCLR_BFPTRC | QSPI_SPTRCLR_IPPTRC);
    qspi_prepare_lut(CMD_FAST_READ_4B, 4, offset, 1, true, false);

    let base = QSPI_AHB_BASE as *const u8;
    for (idx, byte) in out.iter_mut().enumerate() {
        *byte = unsafe { read_volatile(base.add(idx)) };
    }
    qspi_invalidate();
    Ok(())
}

fn qspi_write(mut offset: u32, mut data: &[u8]) -> Result<(), FlashError> {
    while !data.is_empty() {
        let page_left = FLASH_PAGE_SIZE - (offset % FLASH_PAGE_SIZE);
        let chunk = min_usize(min_usize(data.len(), 64), page_left as usize);
        qspi_write_enable()?;
        qspi_ip_write(CMD_PAGE_PROGRAM_4B, 4, offset, 0, &data[..chunk])?;
        qspi_wait_flash_ready()?;
        offset += chunk as u32;
        data = &data[chunk..];
    }
    Ok(())
}

fn qspi_erase_sector(offset: u32) -> Result<(), FlashError> {
    qspi_write_enable()?;
    qspi_ip_write(CMD_SECTOR_ERASE_4B, 4, offset, 0, &[])?;
    qspi_wait_flash_ready()
}

fn qspi_write_enable() -> Result<(), FlashError> {
    qspi_ip_write(CMD_WRITE_ENABLE, 0, 0, 0, &[])
}

fn qspi_wait_flash_ready() -> Result<(), FlashError> {
    for _ in 0..40_000_000u32 {
        let mut status = [0u8; 1];
        qspi_ip_read(CMD_READ_STATUS, 0, 0, 0, &mut status)?;
        if status[0] & 0x01 == 0 {
            return Ok(());
        }
        delay(8);
    }
    Err(FlashError::Timeout)
}

fn qspi_ip_read(
    opcode: u8,
    addr_nbytes: u8,
    addr: u32,
    dummy_nbytes: u8,
    out: &mut [u8],
) -> Result<(), FlashError> {
    qspi_exec_op(opcode, addr_nbytes, addr, dummy_nbytes, Some(out), &[])
}

fn qspi_ip_write(
    opcode: u8,
    addr_nbytes: u8,
    addr: u32,
    dummy_nbytes: u8,
    data: &[u8],
) -> Result<(), FlashError> {
    qspi_exec_op(opcode, addr_nbytes, addr, dummy_nbytes, None, data)
}

fn qspi_exec_op(
    opcode: u8,
    addr_nbytes: u8,
    addr: u32,
    dummy_nbytes: u8,
    read: Option<&mut [u8]>,
    write: &[u8],
) -> Result<(), FlashError> {
    wait_qspi_idle()?;
    qspi_write32(QSPI_SFAR, QSPI_AHB_BASE as u32);
    qspi_write32(
        QSPI_MCR,
        qspi_read32(QSPI_MCR) | QSPI_MCR_CLR_RXF | QSPI_MCR_CLR_TXF,
    );
    qspi_write32(QSPI_SPTRCLR, QSPI_SPTRCLR_BFPTRC | QSPI_SPTRCLR_IPPTRC);

    qspi_prepare_lut(
        opcode,
        addr_nbytes,
        addr,
        dummy_nbytes,
        read.is_some(),
        !write.is_empty(),
    );
    qspi_write32(QSPI_RBCT, QSPI_RBCT_WMRK_MASK | QSPI_RBCT_RXBRD_USEIPS);

    if !write.is_empty() {
        qspi_fill_txfifo(write);
    }

    let data_len = read.as_ref().map_or(write.len(), |buf| buf.len()) as u32;
    qspi_write32(QSPI_IPCR, data_len | (SEQID << 24));
    wait_qspi_idle()?;

    if let Some(out) = read {
        qspi_read_rxfifo(out);
    }

    qspi_invalidate();
    Ok(())
}

fn qspi_prepare_lut(
    opcode: u8,
    addr_nbytes: u8,
    addr: u32,
    dummy_nbytes: u8,
    read: bool,
    write: bool,
) {
    let mut lut = [0u32; 4];
    let mut idx = 0usize;

    lut_def(&mut lut, idx, LUT_CMD, 1, opcode as u32);
    idx += 1;

    let mut n = 0;
    while n < addr_nbytes {
        let shift = 8 * (addr_nbytes - n - 1);
        lut_def(&mut lut, idx, LUT_MODE, 1, (addr >> shift) & 0xff);
        idx += 1;
        n += 1;
    }

    if dummy_nbytes != 0 {
        lut_def(&mut lut, idx, LUT_DUMMY, 1, dummy_nbytes as u32 * 8);
        idx += 1;
    }

    if read {
        lut_def(&mut lut, idx, LUT_READ, 1, 0);
        idx += 1;
    } else if write {
        lut_def(&mut lut, idx, LUT_WRITE, 1, 0);
        idx += 1;
    }

    lut_def(&mut lut, idx, LUT_STOP, 0, 0);

    qspi_write32(QSPI_LUTKEY, QSPI_LUTKEY_VALUE);
    qspi_write32(QSPI_LCKCR, QSPI_LCKCR_UNLOCK);
    for (i, value) in lut.iter().copied().enumerate() {
        qspi_write32(QSPI_LUT_BASE + LUT_OFFSET + i * 4, value);
    }
    qspi_write32(QSPI_LUTKEY, QSPI_LUTKEY_VALUE);
    qspi_write32(QSPI_LCKCR, QSPI_LCKCR_LOCK);
}

fn lut_def(lut: &mut [u32; 4], idx: usize, ins: u32, pad: u32, opr: u32) {
    let val = (((ins << 10) | ((pad - 1) << 8) | opr) & 0xffff) << ((idx % 2) * 16);
    lut[idx / 2] |= val;
}

fn qspi_fill_txfifo(data: &[u8]) {
    let mut i = 0usize;
    while i < data.len() {
        let mut word = 0u32;
        let mut n = 0usize;
        while n < 4 && i + n < data.len() {
            word |= (data[i + n] as u32) << (8 * n);
            n += 1;
        }
        qspi_write32(QSPI_TBDR, word);
        i += 4;
    }
}

fn qspi_read_rxfifo(out: &mut [u8]) {
    let mut i = 0usize;
    while i < out.len() {
        let word = qspi_read32(QSPI_RBDR + (i / 4) * 4);
        let mut n = 0usize;
        while n < 4 && i + n < out.len() {
            out[i + n] = ((word >> (8 * n)) & 0xff) as u8;
            n += 1;
        }
        i += 4;
    }
}

fn wait_qspi_idle() -> Result<(), FlashError> {
    for _ in 0..1_000_000u32 {
        if qspi_read32(QSPI_SR) & (QSPI_SR_IP_ACC | QSPI_SR_AHB_ACC) == 0 {
            return Ok(());
        }
    }
    Err(FlashError::Timeout)
}

fn qspi_invalidate() {
    let reg = qspi_read32(QSPI_MCR);
    qspi_write32(QSPI_MCR, reg | QSPI_MCR_SWRSTHD | QSPI_MCR_SWRSTSD);
    delay(16);
    qspi_write32(QSPI_MCR, reg & !(QSPI_MCR_SWRSTHD | QSPI_MCR_SWRSTSD));
}

fn qspi_read32(offset: usize) -> u32 {
    u32::from_be(unsafe { read_volatile((QSPI_BASE + offset) as *const u32) })
}

fn qspi_write32(offset: usize, value: u32) {
    unsafe { write_volatile((QSPI_BASE + offset) as *mut u32, value.to_be()) };
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

fn range_ok(offset: u32, len: u32) -> bool {
    match offset.checked_add(len) {
        Some(end) => end <= proto::GATEWAY_DK_FLASH_SIZE,
        None => false,
    }
}

fn min_usize(a: usize, b: usize) -> usize {
    if a < b {
        a
    } else {
        b
    }
}

fn delay(count: u32) {
    for _ in 0..count {
        spin_loop();
    }
}
