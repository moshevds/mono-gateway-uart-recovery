#![cfg_attr(not(test), no_std)]

pub const MAGIC: [u8; 4] = *b"MGR1";
pub const VERSION: u8 = 1;

pub const OP_HELLO: u8 = 0x01;
pub const OP_INFO: u8 = 0x02;
pub const OP_READ: u8 = 0x03;
pub const OP_WRITE: u8 = 0x04;
pub const OP_ERASE: u8 = 0x05;
pub const OP_CRC32: u8 = 0x06;
pub const OP_RESET: u8 = 0x07;
pub const OP_CONFIG_UART: u8 = 0x08;
pub const OP_LOAD_BEGIN: u8 = 0x09;
pub const OP_LOAD_CHUNK: u8 = 0x0a;
pub const OP_EXEC: u8 = 0x0b;
pub const OP_READY: u8 = 0x80;
pub const OP_DEBUG_PRINT: u8 = 0x81;
pub const OP_PROGRESS: u8 = 0x82;

pub const STATUS_OK: u8 = 0;
pub const STATUS_BAD_FRAME: u8 = 1;
pub const STATUS_BAD_REQUEST: u8 = 2;
pub const STATUS_RANGE: u8 = 3;
pub const STATUS_FLASH: u8 = 4;
pub const STATUS_UNSUPPORTED: u8 = 5;

pub const HEADER_LEN: usize = 8;
pub const MAX_DATA_LEN: usize = 1024;
pub const MAX_FRAME_LEN: usize = HEADER_LEN + 4 + MAX_DATA_LEN;
pub const MAX_SLIP_LEN: usize = MAX_FRAME_LEN * 2 + 2;
pub const PROGRESS_LEN: usize = 9;
pub const PROGRESS_OP: usize = 0;
pub const PROGRESS_DONE: usize = 1;
pub const PROGRESS_TOTAL: usize = 5;

pub const YOCTO_NOR_IMAGE_SIZE: u32 = 32 * 1024 * 1024;
pub const GATEWAY_DK_FLASH_SIZE: u32 = 64 * 1024 * 1024;
pub const GATEWAY_DK_FLASH_ERASE_SIZE: u32 = 64 * 1024;
pub const GATEWAY_DK_FLASH_WRITE_GRANULE: u32 = 1;

pub const DEFAULT_UART_BAUD: u32 = 115_200;
pub const DEFAULT_FAST_UART_BAUD: u32 = 921_600;
pub const MONO_GATEWAY_DK_UART_CLOCK_HZ: u32 = 300_000_000;
pub const UART_CONFIG_LEN: usize = 5;
pub const UART_CONFIG_DLL: usize = 0;
pub const UART_CONFIG_DLM: usize = 1;
pub const UART_CONFIG_LCR: usize = 2;
pub const UART_CONFIG_FCR: usize = 3;
pub const UART_CONFIG_MCR: usize = 4;
pub const UART_LCR_WLEN8: u8 = 0x03;
pub const UART_FCR_ENABLE_CLEAR: u8 = 0x07;
pub const UART_MCR_DTR_RTS: u8 = 0x03;
pub const STAGE1_LOAD_ADDR: u32 = 0x1000_4000;
pub const STAGE1_ENTRY_ADDR: u32 = STAGE1_LOAD_ADDR;
pub const STAGE1_MAX_LEN: u32 = 0x0001_a000;
pub const SUPPORTED_UART_BAUDS: [u32; 18] = [
    9_600, 19_200, 38_400, 57_600, 115_200, 230_400, 460_800, 500_000, 576_000, 921_600, 1_000_000,
    1_152_000, 1_500_000, 2_000_000, 2_500_000, 3_000_000, 3_500_000, 4_000_000,
];

pub const SLIP_END: u8 = 0xc0;
pub const SLIP_ESC: u8 = 0xdb;
pub const SLIP_ESC_END: u8 = 0xdc;
pub const SLIP_ESC_ESC: u8 = 0xdd;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ProtocolError {
    Short,
    BadMagic,
    BadVersion,
    BufferTooSmall,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SlipError {
    Overflow,
    BadEscape,
    BufferTooSmall,
    FramePending,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Header {
    pub seq: u8,
    pub op: u8,
    pub status: u8,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DeviceInfo {
    pub flash_size: u32,
    pub image_size: u32,
    pub erase_size: u32,
    pub write_granule: u32,
    pub max_data: u16,
    pub jedec_id: u32,
}

impl DeviceInfo {
    pub const ENCODED_LEN: usize = 22;

    pub fn gateway_dk(jedec_id: u32) -> Self {
        Self {
            flash_size: GATEWAY_DK_FLASH_SIZE,
            image_size: YOCTO_NOR_IMAGE_SIZE,
            erase_size: GATEWAY_DK_FLASH_ERASE_SIZE,
            write_granule: GATEWAY_DK_FLASH_WRITE_GRANULE,
            max_data: MAX_DATA_LEN as u16,
            jedec_id,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ProtocolError> {
        if out.len() < Self::ENCODED_LEN {
            return Err(ProtocolError::BufferTooSmall);
        }

        put_u32(out, 0, self.flash_size);
        put_u32(out, 4, self.image_size);
        put_u32(out, 8, self.erase_size);
        put_u32(out, 12, self.write_granule);
        put_u16(out, 16, self.max_data);
        put_u32(out, 18, self.jedec_id);
        Ok(Self::ENCODED_LEN)
    }

    pub fn decode(input: &[u8]) -> Result<Self, ProtocolError> {
        if input.len() < Self::ENCODED_LEN {
            return Err(ProtocolError::Short);
        }

        Ok(Self {
            flash_size: get_u32(input, 0)?,
            image_size: get_u32(input, 4)?,
            erase_size: get_u32(input, 8)?,
            write_granule: get_u32(input, 12)?,
            max_data: get_u16(input, 16)?,
            jedec_id: get_u32(input, 18)?,
        })
    }
}

pub fn status_name(status: u8) -> &'static str {
    match status {
        STATUS_OK => "ok",
        STATUS_BAD_FRAME => "bad-frame",
        STATUS_BAD_REQUEST => "bad-request",
        STATUS_RANGE => "range",
        STATUS_FLASH => "flash",
        STATUS_UNSUPPORTED => "unsupported",
        _ => "unknown",
    }
}

pub fn uart_baud_supported(baud: u32) -> bool {
    matches!(
        baud,
        9_600
            | 19_200
            | 38_400
            | 57_600
            | 115_200
            | 230_400
            | 460_800
            | 500_000
            | 576_000
            | 921_600
            | 1_000_000
            | 1_152_000
            | 1_500_000
            | 2_000_000
            | 2_500_000
            | 3_000_000
            | 3_500_000
            | 4_000_000
    )
}

pub fn build_frame(
    out: &mut [u8],
    seq: u8,
    op: u8,
    status: u8,
    payload: &[u8],
) -> Result<usize, ProtocolError> {
    let len = HEADER_LEN + payload.len();
    if out.len() < len {
        return Err(ProtocolError::BufferTooSmall);
    }

    out[0..4].copy_from_slice(&MAGIC);
    out[4] = VERSION;
    out[5] = seq;
    out[6] = op;
    out[7] = status;
    out[HEADER_LEN..len].copy_from_slice(payload);
    Ok(len)
}

pub fn parse_frame(input: &[u8]) -> Result<(Header, &[u8]), ProtocolError> {
    if input.len() < HEADER_LEN {
        return Err(ProtocolError::Short);
    }
    if input[0..4] != MAGIC {
        return Err(ProtocolError::BadMagic);
    }
    if input[4] != VERSION {
        return Err(ProtocolError::BadVersion);
    }

    Ok((
        Header {
            seq: input[5],
            op: input[6],
            status: input[7],
        },
        &input[HEADER_LEN..],
    ))
}

pub fn put_u16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

pub fn put_u32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

pub fn get_u16(input: &[u8], offset: usize) -> Result<u16, ProtocolError> {
    if input.len() < offset + 2 {
        return Err(ProtocolError::Short);
    }
    Ok(u16::from_le_bytes([input[offset], input[offset + 1]]))
}

pub fn get_u32(input: &[u8], offset: usize) -> Result<u32, ProtocolError> {
    if input.len() < offset + 4 {
        return Err(ProtocolError::Short);
    }
    Ok(u32::from_le_bytes([
        input[offset],
        input[offset + 1],
        input[offset + 2],
        input[offset + 3],
    ]))
}

pub fn slip_encoded_len(input: &[u8]) -> usize {
    let mut len = 2;
    for &byte in input {
        len += match byte {
            SLIP_END | SLIP_ESC => 2,
            _ => 1,
        };
    }
    len
}

pub fn slip_encode(input: &[u8], out: &mut [u8]) -> Result<usize, SlipError> {
    let needed = slip_encoded_len(input);
    if out.len() < needed {
        return Err(SlipError::BufferTooSmall);
    }

    let mut pos = 0;
    out[pos] = SLIP_END;
    pos += 1;

    for &byte in input {
        match byte {
            SLIP_END => {
                out[pos] = SLIP_ESC;
                out[pos + 1] = SLIP_ESC_END;
                pos += 2;
            }
            SLIP_ESC => {
                out[pos] = SLIP_ESC;
                out[pos + 1] = SLIP_ESC_ESC;
                pos += 2;
            }
            _ => {
                out[pos] = byte;
                pos += 1;
            }
        }
    }

    out[pos] = SLIP_END;
    Ok(pos + 1)
}

pub struct SlipDecoder<const N: usize> {
    buf: [u8; N],
    len: usize,
    escaped: bool,
    pending: bool,
}

impl<const N: usize> SlipDecoder<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
            escaped: false,
            pending: false,
        }
    }

    pub fn reset(&mut self) {
        self.len = 0;
        self.escaped = false;
        self.pending = false;
    }

    pub fn frame(&self) -> &[u8] {
        &self.buf[..self.len]
    }

    pub fn push(&mut self, byte: u8) -> Result<Option<usize>, SlipError> {
        if self.pending {
            return Err(SlipError::FramePending);
        }

        if byte == SLIP_END {
            self.escaped = false;
            if self.len == 0 {
                return Ok(None);
            }
            self.pending = true;
            return Ok(Some(self.len));
        }

        let decoded = if self.escaped {
            self.escaped = false;
            match byte {
                SLIP_ESC_END => SLIP_END,
                SLIP_ESC_ESC => SLIP_ESC,
                _ => {
                    self.reset();
                    return Err(SlipError::BadEscape);
                }
            }
        } else if byte == SLIP_ESC {
            self.escaped = true;
            return Ok(None);
        } else {
            byte
        };

        if self.len == N {
            self.reset();
            return Err(SlipError::Overflow);
        }

        self.buf[self.len] = decoded;
        self.len += 1;
        Ok(None)
    }
}

impl<const N: usize> Default for SlipDecoder<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub fn crc32_init() -> u32 {
    0xffff_ffff
}

pub fn crc32_update_state(mut state: u32, bytes: &[u8]) -> u32 {
    for &byte in bytes {
        state ^= byte as u32;
        for _ in 0..8 {
            if state & 1 != 0 {
                state = (state >> 1) ^ 0xedb8_8320;
            } else {
                state >>= 1;
            }
        }
    }
    state
}

pub fn crc32_finish(state: u32) -> u32 {
    !state
}

pub fn crc32(bytes: &[u8]) -> u32 {
    crc32_finish(crc32_update_state(crc32_init(), bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_matches_known_vector() {
        assert_eq!(crc32(b"123456789"), 0xcbf4_3926);
    }

    #[test]
    fn slip_round_trip_escapes_reserved_bytes() {
        let input = [0x01, SLIP_END, 0x02, SLIP_ESC, 0x03];
        let mut encoded = [0; 16];
        let encoded_len = slip_encode(&input, &mut encoded).unwrap();

        let mut decoder = SlipDecoder::<16>::new();
        let mut got = None;
        for &byte in &encoded[..encoded_len] {
            if let Some(len) = decoder.push(byte).unwrap() {
                got = Some(len);
            }
        }

        assert_eq!(got, Some(input.len()));
        assert_eq!(decoder.frame(), input);
    }

    #[test]
    fn frame_round_trip() {
        let mut frame = [0; 64];
        let len = build_frame(&mut frame, 7, OP_INFO, STATUS_OK, b"abc").unwrap();
        let (header, payload) = parse_frame(&frame[..len]).unwrap();

        assert_eq!(header.seq, 7);
        assert_eq!(header.op, OP_INFO);
        assert_eq!(header.status, STATUS_OK);
        assert_eq!(payload, b"abc");
    }

    #[test]
    fn supported_baud_table_matches_default_fast_baud() {
        assert!(uart_baud_supported(DEFAULT_UART_BAUD));
        assert!(uart_baud_supported(DEFAULT_FAST_UART_BAUD));
        assert!(uart_baud_supported(4_000_000));
        assert!(!uart_baud_supported(123_456));
    }
}
