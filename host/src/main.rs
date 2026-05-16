mod serial;

use mono_uart_recovery_protocol as proto;
use serial::SerialPort;
use std::env;
use std::error::Error;
use std::fmt;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

type Result<T> = std::result::Result<T, AppError>;

const REQUEST_IDLE_TIMEOUT: Duration = Duration::from_secs(120);

#[derive(Debug)]
struct AppError(String);

impl AppError {
    fn new(message: impl Into<String>) -> Self {
        Self(message.into())
    }
}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl Error for AppError {}

impl From<std::io::Error> for AppError {
    fn from(value: std::io::Error) -> Self {
        Self(value.to_string())
    }
}

#[derive(Debug)]
struct Config {
    device: String,
    stage1: PathBuf,
    baud: u32,
    fast_uart: bool,
    fast_baud: u32,
    command: Command,
}

#[derive(Debug)]
enum Command {
    Info,
    Backup {
        output: PathBuf,
        offset: u32,
        length: LengthArg,
    },
    Restore {
        input: PathBuf,
        offset: u32,
        erase: bool,
        verify: bool,
    },
    Verify {
        input: PathBuf,
        offset: u32,
    },
    Erase {
        offset: u32,
        length: LengthArg,
    },
    Crc32 {
        offset: u32,
        length: u32,
    },
}

#[derive(Debug, Clone, Copy)]
enum LengthArg {
    Image,
    Full,
    Bytes(u32),
}

#[derive(Debug)]
struct Frame {
    seq: u8,
    op: u8,
    status: u8,
    payload: Vec<u8>,
}

struct Client {
    serial: SerialPort,
    rx: proto::SlipDecoder<{ proto::MAX_FRAME_LEN }>,
    seq: u8,
    timeout: Duration,
}

struct ProgressDisplay<'a> {
    label: &'a str,
    expected_op: u8,
    total: u32,
    shown: bool,
}

fn main() {
    if let Err(err) = run() {
        eprintln!("error: {err}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let config = parse_args(env::args().skip(1).collect())?;
    validate_baud("--baud", config.baud)?;
    validate_baud("--fast-baud", config.fast_baud)?;
    if config.fast_uart && config.fast_baud < config.baud {
        return Err(AppError::new(format!(
            "--fast-baud {} is lower than initial --baud {}; use --no-fast-uart to keep the initial speed",
            config.fast_baud, config.baud
        )));
    }

    let mut client = open_client(&config.device, config.baud)?;
    eprintln!(
        "waiting for Mono Gateway DK recovery loader on {} at {} baud...",
        config.device, config.baud
    );
    client.wait_ready()?;
    load_stage1(&mut client, &config.stage1)?;
    eprintln!(
        "waiting for loaded Mono Gateway DK recovery stage on {}...",
        config.device
    );
    client.wait_ready()?;

    if config.fast_uart && config.fast_baud != config.baud {
        eprintln!("switching recovery UART to {} baud...", config.fast_baud);
        client.configure_uart(config.fast_baud)?;
        drop(client);

        client = open_client(&config.device, config.fast_baud)?;
        eprintln!(
            "waiting for Mono Gateway DK recovery stage on {} at {} baud...",
            config.device, config.fast_baud
        );
        client.wait_ready()?;
    }

    let info = client.info()?;
    print_info(&info);

    match config.command {
        Command::Info => {}
        Command::Backup {
            output,
            offset,
            length,
        } => backup(&mut client, &info, output, offset, length)?,
        Command::Restore {
            input,
            offset,
            erase,
            verify,
        } => restore(&mut client, &info, input, offset, erase, verify)?,
        Command::Verify { input, offset } => verify_file(&mut client, &info, input, offset)?,
        Command::Erase { offset, length } => erase_only(&mut client, &info, offset, length)?,
        Command::Crc32 { offset, length } => {
            let crc = client.crc32(offset, length, "crc32")?;
            println!(
                "crc32 offset=0x{offset:08x} length={} value=0x{crc:08x}",
                length
            );
        }
    }

    Ok(())
}

fn open_client(device: &str, baud: u32) -> Result<Client> {
    let serial = SerialPort::open(device, baud)
        .map_err(|err| AppError::new(format!("failed to open {device} at {baud} baud: {err}")))?;
    Ok(Client::new(serial))
}

fn load_stage1(client: &mut Client, path: &Path) -> Result<()> {
    let image = fs::read(path).map_err(|err| {
        AppError::new(format!(
            "failed to read stage1 image {}: {err}",
            path.display()
        ))
    })?;
    let display_path = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    eprintln!(
        "loading recovery stage1 {} ({} bytes) over UART...",
        display_path.display(),
        image.len()
    );
    match image.len() {
        6418 => {
            eprintln!(
                "warning: this stage1 image predates erase/CRC32 progress reporting; rebuild it with ./build-stub.sh or pass --stage1 to the current image"
            );
        }
        6578 => {
            eprintln!(
                "warning: this stage1 image predates CRC32 progress reporting; rebuild it with ./build-stub.sh or pass --stage1 to the current image"
            );
        }
        _ => {}
    }
    client.load_stage1(&image)?;
    client.exec_stage1()
}

impl Client {
    fn new(serial: SerialPort) -> Self {
        Self {
            serial,
            rx: proto::SlipDecoder::new(),
            seq: 1,
            timeout: REQUEST_IDLE_TIMEOUT,
        }
    }

    fn wait_ready(&mut self) -> Result<()> {
        let mut next_hello = Instant::now();
        let mut hexdump = Hexdump::new();

        loop {
            let now = Instant::now();
            if now >= next_hello {
                let _ = self.send_raw(0, proto::OP_HELLO, proto::STATUS_OK, &[]);
                next_hello = Instant::now() + Duration::from_secs(1);
            }

            let idle_deadline = Instant::now() + Duration::from_millis(50);
            let deadline = if next_hello < idle_deadline {
                next_hello
            } else {
                idle_deadline
            };
            let Some(byte) = self.serial.read_byte_until(deadline)? else {
                hexdump.flush();
                continue;
            };

            hexdump.push(byte);

            match self.rx.push(byte) {
                Ok(Some(_)) => {
                    let decoded = self.rx.frame();
                    let result = parse_host_frame(decoded);
                    self.rx.reset();

                    let Some(frame) = result else {
                        continue;
                    };

                    if frame.op == proto::OP_DEBUG_PRINT {
                        print_debug_payload(&frame.payload);
                        continue;
                    }
                    if frame.op == proto::OP_READY && frame.status == proto::STATUS_OK {
                        hexdump.flush();
                        return Ok(());
                    }
                    if frame.op == proto::OP_READY {
                        eprintln!(
                            "ignoring READY frame with status {} ({})",
                            proto::status_name(frame.status),
                            frame.status
                        );
                    }
                }
                Ok(None) => {}
                Err(_) => {
                    self.rx.reset();
                }
            }
        }
    }

    fn info(&mut self) -> Result<proto::DeviceInfo> {
        let response = self.request(proto::OP_INFO, &[])?;
        proto::DeviceInfo::decode(&response.payload)
            .map_err(|err| AppError::new(format!("bad info response: {err:?}")))
    }

    fn read(&mut self, offset: u32, len: usize) -> Result<Vec<u8>> {
        if len > proto::MAX_DATA_LEN {
            return Err(AppError::new(format!(
                "read chunk too large: {len} > {}",
                proto::MAX_DATA_LEN
            )));
        }

        let mut payload = [0u8; 6];
        proto::put_u32(&mut payload, 0, offset);
        proto::put_u16(&mut payload, 4, len as u16);

        let response = self.request(proto::OP_READ, &payload)?;
        if response.payload.len() < 4 {
            return Err(AppError::new("short read response"));
        }

        let response_offset = proto::get_u32(&response.payload, 0)
            .map_err(|err| AppError::new(format!("bad read response: {err:?}")))?;
        if response_offset != offset {
            return Err(AppError::new(format!(
                "read offset mismatch: requested 0x{offset:08x}, got 0x{response_offset:08x}"
            )));
        }

        let data = response.payload[4..].to_vec();
        if data.len() != len {
            return Err(AppError::new(format!(
                "read length mismatch: requested {len}, got {}",
                data.len()
            )));
        }
        Ok(data)
    }

    fn erase(&mut self, offset: u32, len: u32) -> Result<()> {
        let mut payload = [0u8; 8];
        proto::put_u32(&mut payload, 0, offset);
        proto::put_u32(&mut payload, 4, len);
        let mut progress = ProgressDisplay::new("erase", proto::OP_ERASE, len);
        let result = self.request_with_progress(proto::OP_ERASE, &payload, &mut progress);
        if let Err(err) = result {
            if progress.shown {
                eprintln!();
            }
            return Err(err);
        }
        progress.finish();
        Ok(())
    }

    fn write_flash(&mut self, offset: u32, data: &[u8]) -> Result<()> {
        if data.len() > proto::MAX_DATA_LEN {
            return Err(AppError::new(format!(
                "write chunk too large: {} > {}",
                data.len(),
                proto::MAX_DATA_LEN
            )));
        }

        let mut payload = vec![0u8; 4 + data.len()];
        proto::put_u32(&mut payload, 0, offset);
        payload[4..].copy_from_slice(data);
        self.request(proto::OP_WRITE, &payload)?;
        Ok(())
    }

    fn crc32(&mut self, offset: u32, len: u32, label: &str) -> Result<u32> {
        let mut payload = [0u8; 8];
        proto::put_u32(&mut payload, 0, offset);
        proto::put_u32(&mut payload, 4, len);
        let mut progress = ProgressDisplay::new(label, proto::OP_CRC32, len);
        let result = self.request_with_progress(proto::OP_CRC32, &payload, &mut progress);
        let response = match result {
            Ok(response) => response,
            Err(err) => {
                if progress.shown {
                    eprintln!();
                }
                return Err(err);
            }
        };
        progress.finish();
        proto::get_u32(&response.payload, 0)
            .map_err(|err| AppError::new(format!("bad crc32 response: {err:?}")))
    }

    fn configure_uart(&mut self, baud: u32) -> Result<()> {
        let payload = uart_config_for_baud(baud)?;
        let response = self.request(proto::OP_CONFIG_UART, &payload)?;
        if response.payload != payload {
            return Err(AppError::new(format!(
                "device echoed unexpected UART config for requested baud {baud}"
            )));
        }
        Ok(())
    }

    fn load_stage1(&mut self, image: &[u8]) -> Result<()> {
        if image.is_empty() {
            return Err(AppError::new("stage1 image is empty"));
        }
        if image.len() > proto::STAGE1_MAX_LEN as usize {
            return Err(AppError::new(format!(
                "stage1 image is too large: {} bytes, max {} bytes",
                image.len(),
                proto::STAGE1_MAX_LEN
            )));
        }

        let mut begin = [0u8; 12];
        proto::put_u32(&mut begin, 0, proto::STAGE1_LOAD_ADDR);
        proto::put_u32(&mut begin, 4, image.len() as u32);
        proto::put_u32(&mut begin, 8, proto::STAGE1_ENTRY_ADDR);
        self.request(proto::OP_LOAD_BEGIN, &begin)?;

        let mut offset = 0usize;
        while offset < image.len() {
            let chunk_len = (image.len() - offset).min(proto::MAX_DATA_LEN);
            let mut payload = vec![0u8; 4 + chunk_len];
            proto::put_u32(&mut payload, 0, offset as u32);
            payload[4..].copy_from_slice(&image[offset..offset + chunk_len]);
            self.request(proto::OP_LOAD_CHUNK, &payload)?;
            offset += chunk_len;
        }
        Ok(())
    }

    fn exec_stage1(&mut self) -> Result<()> {
        self.request(proto::OP_EXEC, &[])?;
        Ok(())
    }

    fn request(&mut self, op: u8, payload: &[u8]) -> Result<Frame> {
        self.request_inner(op, payload, self.timeout, |_| Ok(false))
    }

    fn request_with_progress(
        &mut self,
        op: u8,
        payload: &[u8],
        progress: &mut ProgressDisplay<'_>,
    ) -> Result<Frame> {
        self.request_inner(op, payload, self.timeout, |frame| progress.handle(frame))
    }

    fn request_inner<F>(
        &mut self,
        op: u8,
        payload: &[u8],
        idle_timeout: Duration,
        mut handle_async: F,
    ) -> Result<Frame>
    where
        F: FnMut(&Frame) -> Result<bool>,
    {
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1).max(1);
        self.send_raw(seq, op, proto::STATUS_OK, payload)?;

        let mut deadline = Instant::now() + idle_timeout;
        loop {
            let frame = self
                .recv_frame_until(deadline)?
                .ok_or_else(|| AppError::new(format!("timed out waiting for op 0x{op:02x}")))?;
            if frame.op == proto::OP_DEBUG_PRINT {
                print_debug_payload(&frame.payload);
                deadline = Instant::now() + idle_timeout;
                continue;
            }
            if frame.seq == seq && handle_async(&frame)? {
                deadline = Instant::now() + idle_timeout;
                continue;
            }
            if frame.seq != seq || frame.op != op {
                continue;
            }
            if frame.status != proto::STATUS_OK {
                return Err(AppError::new(format!(
                    "device rejected op 0x{op:02x}: {} ({})",
                    proto::status_name(frame.status),
                    frame.status
                )));
            }
            return Ok(frame);
        }
    }

    fn send_raw(&mut self, seq: u8, op: u8, status: u8, payload: &[u8]) -> Result<()> {
        let mut frame = vec![0u8; proto::HEADER_LEN + payload.len()];
        let frame_len = proto::build_frame(&mut frame, seq, op, status, payload)
            .map_err(|err| AppError::new(format!("failed to build frame: {err:?}")))?;
        let mut slip = vec![0u8; proto::slip_encoded_len(&frame[..frame_len])];
        let slip_len = proto::slip_encode(&frame[..frame_len], &mut slip)
            .map_err(|err| AppError::new(format!("failed to encode slip frame: {err:?}")))?;
        self.serial.write_all_retry(&slip[..slip_len])?;
        Ok(())
    }

    fn recv_frame_until(&mut self, deadline: Instant) -> Result<Option<Frame>> {
        loop {
            let Some(byte) = self.serial.read_byte_until(deadline)? else {
                return Ok(None);
            };

            match self.rx.push(byte) {
                Ok(Some(_)) => {
                    let decoded = self.rx.frame();
                    let result = parse_host_frame(decoded);
                    self.rx.reset();
                    if let Some(frame) = result {
                        return Ok(Some(frame));
                    }
                }
                Ok(None) => {}
                Err(err) => {
                    self.rx.reset();
                    let _ = err;
                }
            }
        }
    }
}

impl<'a> ProgressDisplay<'a> {
    fn new(label: &'a str, expected_op: u8, total: u32) -> Self {
        Self {
            label,
            expected_op,
            total,
            shown: false,
        }
    }

    fn handle(&mut self, frame: &Frame) -> Result<bool> {
        if frame.op != proto::OP_PROGRESS {
            return Ok(false);
        }
        if frame.payload.len() < proto::PROGRESS_LEN {
            return Err(AppError::new("short progress response"));
        }
        if frame.payload[proto::PROGRESS_OP] != self.expected_op {
            return Ok(true);
        }

        let done = proto::get_u32(&frame.payload, proto::PROGRESS_DONE)
            .map_err(|err| AppError::new(format!("bad progress response: {err:?}")))?;
        let total = proto::get_u32(&frame.payload, proto::PROGRESS_TOTAL)
            .map_err(|err| AppError::new(format!("bad progress response: {err:?}")))?;
        let total = if total == 0 { self.total } else { total };
        let done = done.min(total);
        print_progress(self.label, done, total);
        self.shown = true;
        Ok(true)
    }

    fn finish(&mut self) {
        if !self.shown {
            print_progress(self.label, self.total, self.total);
            self.shown = true;
        }
        eprintln!();
    }
}

struct Hexdump {
    offset: usize,
    line: [u8; 16],
    len: usize,
}

impl Hexdump {
    fn new() -> Self {
        Self {
            offset: 0,
            line: [0; 16],
            len: 0,
        }
    }

    fn push(&mut self, byte: u8) {
        self.line[self.len] = byte;
        self.len += 1;
        if self.len == self.line.len() {
            self.flush();
        }
    }

    fn flush(&mut self) {
        if self.len == 0 {
            return;
        }

        eprint!("{:08x}  ", self.offset);
        for i in 0..16 {
            if i < self.len {
                eprint!("{:02x} ", self.line[i]);
            } else {
                eprint!("   ");
            }
            if i == 7 {
                eprint!(" ");
            }
        }

        eprint!(" |");
        for &byte in &self.line[..self.len] {
            let ch = if byte.is_ascii_graphic() || byte == b' ' {
                char::from(byte)
            } else {
                '.'
            };
            eprint!("{ch}");
        }
        eprintln!("|");

        self.offset += self.len;
        self.len = 0;
    }
}

fn parse_host_frame(decoded: &[u8]) -> Option<Frame> {
    proto::parse_frame(decoded)
        .ok()
        .map(|(header, payload)| Frame {
            seq: header.seq,
            op: header.op,
            status: header.status,
            payload: payload.to_vec(),
        })
}

fn print_debug_payload(payload: &[u8]) {
    let text = String::from_utf8_lossy(payload);
    let text = text.trim_end_matches(|c| c == '\r' || c == '\n');
    eprintln!("[stub] {text}");
}

fn backup(
    client: &mut Client,
    info: &proto::DeviceInfo,
    output: PathBuf,
    offset: u32,
    length: LengthArg,
) -> Result<()> {
    let length = resolve_length(info, length)?;
    check_range(info.flash_size, offset, length)?;
    let chunk = usize::from(info.max_data).clamp(1, proto::MAX_DATA_LEN);
    let mut writer = BufWriter::new(File::create(&output)?);
    let mut done = 0u32;

    eprintln!(
        "backing up {} bytes from NOR offset 0x{offset:08x} to {}",
        length,
        output.display()
    );

    while done < length {
        let this_len = chunk.min((length - done) as usize);
        let data = client.read(offset + done, this_len)?;
        writer.write_all(&data)?;
        done += this_len as u32;
        print_progress("backup", done, length);
    }
    writer.flush()?;
    eprintln!();
    Ok(())
}

fn restore(
    client: &mut Client,
    info: &proto::DeviceInfo,
    input: PathBuf,
    offset: u32,
    erase: bool,
    verify: bool,
) -> Result<()> {
    let file_len = fs::metadata(&input)?.len();
    if file_len > u64::from(u32::MAX) {
        return Err(AppError::new("input image is too large for this protocol"));
    }
    let file_len = file_len as u32;
    check_range(info.flash_size, offset, file_len)?;

    if file_len != info.image_size {
        eprintln!(
            "warning: input is {} bytes; Yocto QSPI firmware images are {} bytes",
            file_len, info.image_size
        );
    }

    if erase {
        if offset % info.erase_size != 0 {
            return Err(AppError::new(format!(
                "restore offset 0x{offset:08x} is not erase-sector aligned ({})",
                info.erase_size
            )));
        }
        let erase_len = align_up(file_len, info.erase_size)?;
        check_range(info.flash_size, offset, erase_len)?;
        eprintln!("erasing {} bytes at NOR offset 0x{offset:08x}", erase_len);
        client.erase(offset, erase_len)?;
    }

    let chunk = usize::from(info.max_data).clamp(1, proto::MAX_DATA_LEN);
    let mut reader = BufReader::new(File::open(&input)?);
    let mut buf = vec![0u8; chunk];
    let mut done = 0u32;
    let mut crc_state = proto::crc32_init();

    eprintln!(
        "writing {} bytes from {} to NOR offset 0x{offset:08x}",
        file_len,
        input.display()
    );

    while done < file_len {
        let this_len = chunk.min((file_len - done) as usize);
        reader.read_exact(&mut buf[..this_len])?;
        crc_state = proto::crc32_update_state(crc_state, &buf[..this_len]);
        client.write_flash(offset + done, &buf[..this_len])?;
        done += this_len as u32;
        print_progress("write", done, file_len);
    }
    eprintln!();

    if verify {
        let expected = proto::crc32_finish(crc_state);
        eprintln!("verifying device CRC32 over written range...");
        let actual = client.crc32(offset, file_len, "verify")?;
        if actual != expected {
            return Err(AppError::new(format!(
                "verify failed: host crc32=0x{expected:08x}, device crc32=0x{actual:08x}"
            )));
        }
        eprintln!("verify ok: crc32=0x{actual:08x}");
    }

    Ok(())
}

fn verify_file(
    client: &mut Client,
    info: &proto::DeviceInfo,
    input: PathBuf,
    offset: u32,
) -> Result<()> {
    let file_len = fs::metadata(&input)?.len();
    if file_len > u64::from(u32::MAX) {
        return Err(AppError::new("input image is too large for this protocol"));
    }
    let file_len = file_len as u32;
    check_range(info.flash_size, offset, file_len)?;

    let mut reader = BufReader::new(File::open(&input)?);
    let mut buf = [0u8; 64 * 1024];
    let mut state = proto::crc32_init();
    loop {
        let n = reader.read(&mut buf)?;
        if n == 0 {
            break;
        }
        state = proto::crc32_update_state(state, &buf[..n]);
    }
    let expected = proto::crc32_finish(state);
    eprintln!("verifying device CRC32 over input range...");
    let actual = client.crc32(offset, file_len, "verify")?;
    if actual != expected {
        return Err(AppError::new(format!(
            "verify failed: host crc32=0x{expected:08x}, device crc32=0x{actual:08x}"
        )));
    }
    println!("verify ok: crc32=0x{actual:08x}");
    Ok(())
}

fn erase_only(
    client: &mut Client,
    info: &proto::DeviceInfo,
    offset: u32,
    length: LengthArg,
) -> Result<()> {
    let length = resolve_length(info, length)?;
    check_range(info.flash_size, offset, length)?;
    if offset % info.erase_size != 0 {
        return Err(AppError::new(format!(
            "erase offset 0x{offset:08x} is not erase-sector aligned ({})",
            info.erase_size
        )));
    }
    if length == 0 || length % info.erase_size != 0 {
        return Err(AppError::new(format!(
            "erase length {length} is not a non-zero multiple of erase-sector size {}",
            info.erase_size
        )));
    }

    eprintln!("erasing {} bytes at NOR offset 0x{offset:08x}", length);
    client.erase(offset, length)?;
    eprintln!("erase complete");
    Ok(())
}

fn print_info(info: &proto::DeviceInfo) {
    println!(
        "device: flash={} bytes image={} bytes erase={} bytes max-frame-data={} jedec=0x{:06x}",
        info.flash_size, info.image_size, info.erase_size, info.max_data, info.jedec_id
    );
}

fn print_progress(label: &str, done: u32, total: u32) {
    let percent = if total == 0 {
        100
    } else {
        (u64::from(done) * 100 / u64::from(total)) as u32
    };
    eprint!("\r{label}: {done}/{total} bytes ({percent}%)");
}

fn resolve_length(info: &proto::DeviceInfo, length: LengthArg) -> Result<u32> {
    match length {
        LengthArg::Image => Ok(info.image_size),
        LengthArg::Full => Ok(info.flash_size),
        LengthArg::Bytes(value) => Ok(value),
    }
}

fn check_range(flash_size: u32, offset: u32, length: u32) -> Result<()> {
    let end = u64::from(offset) + u64::from(length);
    if end > u64::from(flash_size) {
        return Err(AppError::new(format!(
            "range 0x{offset:08x}..0x{end:08x} exceeds flash size 0x{flash_size:08x}"
        )));
    }
    Ok(())
}

fn align_up(value: u32, align: u32) -> Result<u32> {
    if align == 0 {
        return Err(AppError::new("device reported zero erase size"));
    }
    let value = u64::from(value);
    let align = u64::from(align);
    let aligned = value.div_ceil(align) * align;
    if aligned > u64::from(u32::MAX) {
        return Err(AppError::new("aligned erase range is too large"));
    }
    Ok(aligned as u32)
}

fn validate_baud(flag: &str, baud: u32) -> Result<()> {
    if !proto::uart_baud_supported(baud) {
        return Err(AppError::new(format!(
            "{flag} {baud} is not supported; supported baud rates: {}",
            supported_bauds()
        )));
    }
    uart_divisor(baud).map(|_| ()).ok_or_else(|| {
        AppError::new(format!(
            "{flag} {baud} cannot be represented by the UART divisor"
        ))
    })
}

fn supported_bauds() -> String {
    let mut out = String::new();
    for (index, baud) in proto::SUPPORTED_UART_BAUDS.iter().enumerate() {
        if index != 0 {
            out.push_str(", ");
        }
        out.push_str(&baud.to_string());
    }
    out
}

fn uart_config_for_baud(baud: u32) -> Result<[u8; proto::UART_CONFIG_LEN]> {
    let divisor = uart_divisor(baud).ok_or_else(|| {
        AppError::new(format!(
            "baud {baud} cannot be represented by the UART divisor"
        ))
    })?;
    Ok([
        (divisor & 0xff) as u8,
        ((divisor >> 8) & 0xff) as u8,
        proto::UART_LCR_WLEN8,
        proto::UART_FCR_ENABLE_CLEAR,
        proto::UART_MCR_DTR_RTS,
    ])
}

fn uart_divisor(baud: u32) -> Option<u32> {
    if baud == 0 {
        return None;
    }
    let divisor = (u64::from(proto::MONO_GATEWAY_DK_UART_CLOCK_HZ) + (u64::from(baud) * 8))
        / (u64::from(baud) * 16);
    if divisor == 0 || divisor > 0xffff {
        None
    } else {
        Some(divisor as u32)
    }
}

fn parse_args(args: Vec<String>) -> Result<Config> {
    let mut device = None;
    let mut stage1 = default_stage1_path();
    let mut baud = proto::DEFAULT_UART_BAUD;
    let mut fast_uart = true;
    let mut fast_baud = proto::DEFAULT_FAST_UART_BAUD;
    let mut i = 0;

    while i < args.len() {
        match args[i].as_str() {
            "-h" | "--help" => {
                print_usage();
                std::process::exit(0);
            }
            "-d" | "--device" => {
                i += 1;
                device = Some(require_arg(&args, i, "--device")?.to_string());
            }
            arg if arg.starts_with("--device=") => {
                device = Some(arg["--device=".len()..].to_string());
            }
            "--stage1" => {
                i += 1;
                stage1 = PathBuf::from(require_arg(&args, i, "--stage1")?);
            }
            arg if arg.starts_with("--stage1=") => {
                stage1 = PathBuf::from(&arg["--stage1=".len()..]);
            }
            "-b" | "--baud" => {
                i += 1;
                baud = require_arg(&args, i, "--baud")?
                    .parse()
                    .map_err(|_| AppError::new("invalid --baud value"))?;
            }
            arg if arg.starts_with("--baud=") => {
                baud = arg["--baud=".len()..]
                    .parse()
                    .map_err(|_| AppError::new("invalid --baud value"))?;
            }
            "--no-fast-uart" => {
                fast_uart = false;
            }
            "--fast-baud" => {
                i += 1;
                fast_baud = require_arg(&args, i, "--fast-baud")?
                    .parse()
                    .map_err(|_| AppError::new("invalid --fast-baud value"))?;
            }
            arg if arg.starts_with("--fast-baud=") => {
                fast_baud = arg["--fast-baud=".len()..]
                    .parse()
                    .map_err(|_| AppError::new("invalid --fast-baud value"))?;
            }
            _ => break,
        }
        i += 1;
    }

    let device = device.ok_or_else(|| AppError::new("missing --device /dev/ttyUSBX"))?;
    let command = parse_command(&args[i..])?;
    Ok(Config {
        device,
        stage1,
        baud,
        fast_uart,
        fast_baud,
        command,
    })
}

fn default_stage1_path() -> PathBuf {
    const STAGE1_NAME: &str = "mono-uart-recovery-stage1.bin";

    if let Ok(exe) = env::current_exe() {
        if let Some(exe_dir) = exe.parent() {
            let next_to_exe = exe_dir.join(STAGE1_NAME);
            if next_to_exe.exists() {
                return next_to_exe;
            }

            if let Some(target_dir) = exe_dir.parent() {
                let in_target = target_dir
                    .join("aarch64-unknown-none")
                    .join("release")
                    .join(STAGE1_NAME);
                if in_target.exists() {
                    return in_target;
                }
            }
        }
    }

    PathBuf::from("target/aarch64-unknown-none/release/mono-uart-recovery-stage1.bin")
}

fn parse_command(args: &[String]) -> Result<Command> {
    let Some(command) = args.first().map(String::as_str) else {
        return Err(AppError::new("missing command"));
    };

    match command {
        "info" => Ok(Command::Info),
        "backup" => {
            let mut offset = 0;
            let mut length = LengthArg::Image;
            let mut output = None;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--offset" => {
                        i += 1;
                        offset = parse_size_u32(require_arg(args, i, "--offset")?)?;
                    }
                    "--length" => {
                        i += 1;
                        length = parse_length_arg(require_arg(args, i, "--length")?)?;
                    }
                    arg if arg.starts_with("--offset=") => {
                        offset = parse_size_u32(&arg["--offset=".len()..])?;
                    }
                    arg if arg.starts_with("--length=") => {
                        length = parse_length_arg(&arg["--length=".len()..])?;
                    }
                    arg if arg.starts_with('-') => {
                        return Err(AppError::new(format!("unknown backup option: {arg}")));
                    }
                    path => {
                        if output.is_some() {
                            return Err(AppError::new("backup accepts one output path"));
                        }
                        output = Some(PathBuf::from(path));
                    }
                }
                i += 1;
            }
            Ok(Command::Backup {
                output: output.ok_or_else(|| AppError::new("backup requires output path"))?,
                offset,
                length,
            })
        }
        "restore" => {
            let mut offset = 0;
            let mut erase = true;
            let mut verify = true;
            let mut input = None;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--offset" => {
                        i += 1;
                        offset = parse_size_u32(require_arg(args, i, "--offset")?)?;
                    }
                    "--no-erase" => erase = false,
                    "--no-verify" => verify = false,
                    arg if arg.starts_with("--offset=") => {
                        offset = parse_size_u32(&arg["--offset=".len()..])?;
                    }
                    arg if arg.starts_with('-') => {
                        return Err(AppError::new(format!("unknown restore option: {arg}")));
                    }
                    path => {
                        if input.is_some() {
                            return Err(AppError::new("restore accepts one input path"));
                        }
                        input = Some(PathBuf::from(path));
                    }
                }
                i += 1;
            }
            Ok(Command::Restore {
                input: input.ok_or_else(|| AppError::new("restore requires input path"))?,
                offset,
                erase,
                verify,
            })
        }
        "verify" => {
            let mut offset = 0;
            let mut input = None;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--offset" => {
                        i += 1;
                        offset = parse_size_u32(require_arg(args, i, "--offset")?)?;
                    }
                    arg if arg.starts_with("--offset=") => {
                        offset = parse_size_u32(&arg["--offset=".len()..])?;
                    }
                    arg if arg.starts_with('-') => {
                        return Err(AppError::new(format!("unknown verify option: {arg}")));
                    }
                    path => {
                        if input.is_some() {
                            return Err(AppError::new("verify accepts one input path"));
                        }
                        input = Some(PathBuf::from(path));
                    }
                }
                i += 1;
            }
            Ok(Command::Verify {
                input: input.ok_or_else(|| AppError::new("verify requires input path"))?,
                offset,
            })
        }
        "erase" => {
            let mut offset = 0;
            let mut length = None;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--offset" => {
                        i += 1;
                        offset = parse_size_u32(require_arg(args, i, "--offset")?)?;
                    }
                    "--length" => {
                        i += 1;
                        length = Some(parse_length_arg(require_arg(args, i, "--length")?)?);
                    }
                    arg if arg.starts_with("--offset=") => {
                        offset = parse_size_u32(&arg["--offset=".len()..])?;
                    }
                    arg if arg.starts_with("--length=") => {
                        length = Some(parse_length_arg(&arg["--length=".len()..])?);
                    }
                    arg => return Err(AppError::new(format!("unknown erase option: {arg}"))),
                }
                i += 1;
            }
            Ok(Command::Erase {
                offset,
                length: length.ok_or_else(|| AppError::new("erase requires --length"))?,
            })
        }
        "crc32" => {
            let mut offset = None;
            let mut length = None;
            let mut i = 1;
            while i < args.len() {
                match args[i].as_str() {
                    "--offset" => {
                        i += 1;
                        offset = Some(parse_size_u32(require_arg(args, i, "--offset")?)?);
                    }
                    "--length" => {
                        i += 1;
                        length = Some(parse_size_u32(require_arg(args, i, "--length")?)?);
                    }
                    arg if arg.starts_with("--offset=") => {
                        offset = Some(parse_size_u32(&arg["--offset=".len()..])?);
                    }
                    arg if arg.starts_with("--length=") => {
                        length = Some(parse_size_u32(&arg["--length=".len()..])?);
                    }
                    arg => return Err(AppError::new(format!("unknown crc32 option: {arg}"))),
                }
                i += 1;
            }
            Ok(Command::Crc32 {
                offset: offset.ok_or_else(|| AppError::new("crc32 requires --offset"))?,
                length: length.ok_or_else(|| AppError::new("crc32 requires --length"))?,
            })
        }
        _ => Err(AppError::new(format!("unknown command: {command}"))),
    }
}

fn require_arg<'a>(args: &'a [String], index: usize, flag: &str) -> Result<&'a str> {
    args.get(index)
        .map(String::as_str)
        .ok_or_else(|| AppError::new(format!("{flag} requires a value")))
}

fn parse_length_arg(input: &str) -> Result<LengthArg> {
    match input {
        "image" => Ok(LengthArg::Image),
        "full" => Ok(LengthArg::Full),
        _ => parse_size_u32(input).map(LengthArg::Bytes),
    }
}

fn parse_size_u32(input: &str) -> Result<u32> {
    let value = parse_size(input)?;
    if value > u64::from(u32::MAX) {
        return Err(AppError::new(format!("value is too large: {input}")));
    }
    Ok(value as u32)
}

fn parse_size(input: &str) -> Result<u64> {
    let input = input.trim();
    if input.is_empty() {
        return Err(AppError::new("empty size value"));
    }

    let (number, multiplier) = match input.as_bytes().last().copied() {
        Some(b'k') | Some(b'K') => (&input[..input.len() - 1], 1024),
        Some(b'm') | Some(b'M') => (&input[..input.len() - 1], 1024 * 1024),
        Some(b'g') | Some(b'G') => (&input[..input.len() - 1], 1024 * 1024 * 1024),
        _ => (input, 1),
    };

    let base = if let Some(hex) = number
        .strip_prefix("0x")
        .or_else(|| number.strip_prefix("0X"))
    {
        u64::from_str_radix(hex, 16)
    } else {
        number.parse()
    }
    .map_err(|_| AppError::new(format!("invalid size value: {input}")))?;

    base.checked_mul(multiplier)
        .ok_or_else(|| AppError::new(format!("size value overflows: {input}")))
}

fn print_usage() {
    println!(
        "Usage:
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--baud 115200] [--fast-baud 921600] [--no-fast-uart] info
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] backup <out.bin> [--length image|full|SIZE] [--offset SIZE]
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] restore <firmware-qspi.bin> [--offset SIZE] [--no-erase] [--no-verify]
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] verify <firmware-qspi.bin> [--offset SIZE]
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] erase [--offset SIZE] --length image|full|SIZE
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] crc32 --offset SIZE --length SIZE"
    );
}
