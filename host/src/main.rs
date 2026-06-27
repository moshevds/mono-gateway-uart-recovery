use mono_uart_recovery_protocol as proto;
use serial2::SerialPort;
use std::env;
use std::error::Error;
use std::fmt;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

type Result<T> = std::result::Result<T, AppError>;

const REQUEST_IDLE_TIMEOUT: Duration = Duration::from_secs(120);
const MIB: u32 = 1024 * 1024;
const NOR_PARTITIONS: [NorPartition; 6] = [
    NorPartition {
        name: "rcw-bl2",
        offset: 0,
        size: NorPartitionSize::Bytes(MIB),
    },
    NorPartition {
        name: "uboot",
        offset: MIB,
        size: NorPartitionSize::Bytes(2 * MIB),
    },
    NorPartition {
        name: "uboot-env",
        offset: 3 * MIB,
        size: NorPartitionSize::Bytes(MIB),
    },
    NorPartition {
        name: "fman-ucode",
        offset: 4 * MIB,
        size: NorPartitionSize::Bytes(MIB),
    },
    NorPartition {
        name: "recovery-dtb",
        offset: 5 * MIB,
        size: NorPartitionSize::Bytes(MIB),
    },
    NorPartition {
        name: "kernel-initramfs",
        offset: 10 * MIB,
        size: NorPartitionSize::Rest,
    },
];

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
        selector: RestoreSelector,
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

#[derive(Debug, Clone, PartialEq, Eq)]
enum RestoreSelector {
    WholeImage,
    ByteRange(InclusiveByteRange),
    Partitions(Vec<String>),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct InclusiveByteRange {
    start: u32,
    end: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SourceRange {
    offset: u32,
    len: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RestoreRange {
    image_offset: u32,
    flash_offset: u32,
    write_len: u32,
    erase_len: u32,
}

#[derive(Debug, Clone, Copy)]
struct NorPartition {
    name: &'static str,
    offset: u32,
    size: NorPartitionSize,
}

#[derive(Debug, Clone, Copy)]
enum NorPartitionSize {
    Bytes(u32),
    Rest,
}

#[derive(Debug, Clone, Copy)]
struct VerifiedRange {
    flash_offset: u32,
    len: u32,
    expected_crc32: u32,
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
            selector,
        } => restore(&mut client, &info, input, offset, erase, verify, selector)?,
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
        .and_then(|mut serial| {
            serial.set_write_timeout(REQUEST_IDLE_TIMEOUT)?;
            serial.discard_buffers()?;
            Ok(serial)
        })
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
            let Some(byte) = read_byte_until(&mut self.serial, deadline)? else {
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
        Write::write_all(&mut self.serial, &slip[..slip_len])?;
        Write::flush(&mut self.serial)?;
        Ok(())
    }

    fn recv_frame_until(&mut self, deadline: Instant) -> Result<Option<Frame>> {
        loop {
            let Some(byte) = read_byte_until(&mut self.serial, deadline)? else {
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

fn read_byte_until(serial: &mut SerialPort, deadline: Instant) -> io::Result<Option<u8>> {
    serial.set_read_timeout(deadline.saturating_duration_since(Instant::now()))?;

    let mut byte = [0u8; 1];
    match Read::read_exact(serial, &mut byte) {
        Ok(()) => Ok(Some(byte[0])),
        Err(err) if err.kind() == io::ErrorKind::TimedOut => Ok(None),
        Err(err) => Err(err),
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
    selector: RestoreSelector,
) -> Result<()> {
    let file_len = fs::metadata(&input)?.len();
    if file_len > u64::from(u32::MAX) {
        return Err(AppError::new("input image is too large for this protocol"));
    }
    let file_len = file_len as u32;
    let ranges = resolve_restore_ranges(info, offset, file_len, &selector)?;
    let total_write = ranges
        .iter()
        .try_fold(0u32, |total, range| total.checked_add(range.write_len))
        .ok_or_else(|| AppError::new("selected restore ranges are too large"))?;

    if file_len != info.image_size {
        eprintln!(
            "warning: input is {} bytes; Yocto QSPI firmware images are {} bytes",
            file_len, info.image_size
        );
    }

    if erase {
        for range in &ranges {
            if range.flash_offset % info.erase_size != 0 {
                return Err(AppError::new(format!(
                    "restore offset 0x{:08x} is not erase-sector aligned ({})",
                    range.flash_offset, info.erase_size
                )));
            }
            if range.erase_len == 0 || range.erase_len % info.erase_size != 0 {
                return Err(AppError::new(format!(
                    "erase length {} at NOR offset 0x{:08x} is not a non-zero multiple of erase-sector size {}",
                    range.erase_len, range.flash_offset, info.erase_size
                )));
            }
            check_range(info.flash_size, range.flash_offset, range.erase_len)?;
            eprintln!(
                "erasing {} bytes at NOR offset 0x{:08x}",
                range.erase_len, range.flash_offset
            );
            client.erase(range.flash_offset, range.erase_len)?;
        }
    }

    let chunk = usize::from(info.max_data).clamp(1, proto::MAX_DATA_LEN);
    let mut reader = BufReader::new(File::open(&input)?);
    let mut buf = vec![0u8; chunk];
    let mut total_done = 0u32;
    let mut verified_ranges = Vec::with_capacity(ranges.len());

    if ranges.len() == 1 {
        let range = ranges[0];
        eprintln!(
            "writing {} bytes from {} offset 0x{:08x} to NOR offset 0x{:08x}",
            range.write_len,
            input.display(),
            range.image_offset,
            range.flash_offset
        );
    } else {
        eprintln!(
            "writing {} bytes from {} across {} NOR ranges",
            total_write,
            input.display(),
            ranges.len()
        );
    }

    for range in &ranges {
        reader.seek(SeekFrom::Start(u64::from(range.image_offset)))?;
        let mut range_done = 0u32;
        let mut crc_state = proto::crc32_init();

        while range_done < range.write_len {
            let this_len = chunk.min((range.write_len - range_done) as usize);
            reader.read_exact(&mut buf[..this_len])?;
            crc_state = proto::crc32_update_state(crc_state, &buf[..this_len]);
            client.write_flash(range.flash_offset + range_done, &buf[..this_len])?;
            range_done += this_len as u32;
            total_done += this_len as u32;
            print_progress("write", total_done, total_write);
        }

        verified_ranges.push(VerifiedRange {
            flash_offset: range.flash_offset,
            len: range.write_len,
            expected_crc32: proto::crc32_finish(crc_state),
        });
    }
    eprintln!();

    if verify {
        if verified_ranges.len() == 1 {
            eprintln!("verifying device CRC32 over written range...");
        } else {
            eprintln!(
                "verifying device CRC32 over {} written ranges...",
                verified_ranges.len()
            );
        }
        for range in &verified_ranges {
            let actual = client.crc32(range.flash_offset, range.len, "verify")?;
            if actual != range.expected_crc32 {
                return Err(AppError::new(format!(
                    "verify failed at NOR offset 0x{:08x}: host crc32=0x{:08x}, device crc32=0x{actual:08x}",
                    range.flash_offset, range.expected_crc32
                )));
            }
            eprintln!(
                "verify ok: offset=0x{:08x} length={} crc32=0x{actual:08x}",
                range.flash_offset, range.len
            );
        }
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

fn check_input_range(file_len: u32, offset: u32, length: u32) -> Result<()> {
    let end = u64::from(offset) + u64::from(length);
    if end > u64::from(file_len) {
        return Err(AppError::new(format!(
            "range 0x{offset:08x}..0x{end:08x} exceeds input image length 0x{file_len:08x}"
        )));
    }
    Ok(())
}

fn resolve_restore_ranges(
    info: &proto::DeviceInfo,
    flash_base_offset: u32,
    file_len: u32,
    selector: &RestoreSelector,
) -> Result<Vec<RestoreRange>> {
    let source_ranges = match selector {
        RestoreSelector::WholeImage => vec![SourceRange {
            offset: 0,
            len: file_len,
        }],
        RestoreSelector::ByteRange(range) => {
            vec![source_range_for_byte_range(info, file_len, *range)?]
        }
        RestoreSelector::Partitions(names) => {
            let mut ranges = Vec::with_capacity(names.len());
            for name in names {
                ranges.push(source_range_for_partition(file_len, name)?);
            }
            ranges
        }
    };

    let source_ranges = coalesce_source_ranges(source_ranges)?;
    let mut restore_ranges = Vec::with_capacity(source_ranges.len());
    for source in source_ranges {
        check_input_range(file_len, source.offset, source.len)?;
        let flash_offset = checked_add_u32(flash_base_offset, source.offset).ok_or_else(|| {
            AppError::new(format!(
                "restore target offset overflows: 0x{flash_base_offset:08x} + 0x{:08x}",
                source.offset
            ))
        })?;
        check_range(info.flash_size, flash_offset, source.len)?;
        restore_ranges.push(RestoreRange {
            image_offset: source.offset,
            flash_offset,
            write_len: source.len,
            erase_len: align_up(source.len, info.erase_size)?,
        });
    }
    Ok(restore_ranges)
}

fn source_range_for_byte_range(
    info: &proto::DeviceInfo,
    file_len: u32,
    range: InclusiveByteRange,
) -> Result<SourceRange> {
    if info.erase_size == 0 {
        return Err(AppError::new("device reported zero erase size"));
    }
    if range.end < range.start {
        return Err(AppError::new(format!(
            "byte range start 0x{:08x} is after end 0x{:08x}",
            range.start, range.end
        )));
    }

    let start = range.start - (range.start % info.erase_size);
    let end_exclusive = range
        .end
        .checked_add(1)
        .ok_or_else(|| AppError::new("byte range end is too large"))?;
    let end = align_up(end_exclusive, info.erase_size)?;
    let len = end.checked_sub(start).ok_or_else(|| {
        AppError::new(format!(
            "byte range 0x{:08x}..0x{:08x} cannot be aligned to erase sectors",
            range.start, range.end
        ))
    })?;
    check_input_range(file_len, start, len)?;
    Ok(SourceRange { offset: start, len })
}

fn source_range_for_partition(file_len: u32, name: &str) -> Result<SourceRange> {
    let partition = find_nor_partition(name).ok_or_else(|| {
        AppError::new(format!(
            "unknown NOR partition '{name}'; known partitions: {}",
            known_partition_names()
        ))
    })?;
    let end = match partition.size {
        NorPartitionSize::Bytes(len) => checked_add_u32(partition.offset, len)
            .ok_or_else(|| AppError::new(format!("partition {name} has an overflowing range")))?,
        NorPartitionSize::Rest => file_len,
    };
    if end <= partition.offset {
        return Err(AppError::new(format!(
            "partition {name} starts at 0x{:08x}, beyond input image length 0x{file_len:08x}",
            partition.offset
        )));
    }
    if end > file_len {
        return Err(AppError::new(format!(
            "partition {name} ends at 0x{end:08x}, beyond input image length 0x{file_len:08x}"
        )));
    }
    Ok(SourceRange {
        offset: partition.offset,
        len: end - partition.offset,
    })
}

fn coalesce_source_ranges(mut ranges: Vec<SourceRange>) -> Result<Vec<SourceRange>> {
    ranges.retain(|range| range.len != 0);
    ranges.sort_by_key(|range| range.offset);

    let mut out: Vec<SourceRange> = Vec::with_capacity(ranges.len());
    for range in ranges {
        if let Some(last) = out.last_mut() {
            let last_end = u64::from(last.offset) + u64::from(last.len);
            let range_end = u64::from(range.offset) + u64::from(range.len);
            if u64::from(range.offset) <= last_end {
                let merged_end = last_end.max(range_end);
                last.len = (merged_end - u64::from(last.offset)) as u32;
                continue;
            }
        }
        out.push(range);
    }

    if out.is_empty() {
        return Err(AppError::new("selected restore range is empty"));
    }
    Ok(out)
}

fn find_nor_partition(name: &str) -> Option<NorPartition> {
    NOR_PARTITIONS
        .iter()
        .copied()
        .find(|partition| partition.name == name)
}

fn known_partition_names() -> String {
    let mut out = String::new();
    for (index, partition) in NOR_PARTITIONS.iter().enumerate() {
        if index != 0 {
            out.push_str(", ");
        }
        out.push_str(partition.name);
    }
    out
}

fn checked_add_u32(lhs: u32, rhs: u32) -> Option<u32> {
    lhs.checked_add(rhs)
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
            let mut selector = None;
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
                    "--byte-range" => {
                        i += 1;
                        set_restore_selector(
                            &mut selector,
                            RestoreSelector::ByteRange(parse_byte_range_arg(require_arg(
                                args,
                                i,
                                "--byte-range",
                            )?)?),
                        )?;
                    }
                    "--partitions" => {
                        i += 1;
                        set_restore_selector(
                            &mut selector,
                            RestoreSelector::Partitions(parse_partition_list(require_arg(
                                args,
                                i,
                                "--partitions",
                            )?)?),
                        )?;
                    }
                    arg if arg.starts_with("--offset=") => {
                        offset = parse_size_u32(&arg["--offset=".len()..])?;
                    }
                    arg if arg.starts_with("--byte-range=") => {
                        set_restore_selector(
                            &mut selector,
                            RestoreSelector::ByteRange(parse_byte_range_arg(
                                &arg["--byte-range=".len()..],
                            )?),
                        )?;
                    }
                    arg if arg.starts_with("--partitions=") => {
                        set_restore_selector(
                            &mut selector,
                            RestoreSelector::Partitions(parse_partition_list(
                                &arg["--partitions=".len()..],
                            )?),
                        )?;
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
                selector: selector.unwrap_or(RestoreSelector::WholeImage),
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

fn set_restore_selector(
    slot: &mut Option<RestoreSelector>,
    selector: RestoreSelector,
) -> Result<()> {
    if slot.is_some() {
        return Err(AppError::new(
            "restore accepts only one of --byte-range or --partitions",
        ));
    }
    *slot = Some(selector);
    Ok(())
}

fn parse_byte_range_arg(input: &str) -> Result<InclusiveByteRange> {
    let (start, end) = input
        .split_once("..")
        .ok_or_else(|| AppError::new("byte range must use START..END"))?;
    if end.contains("..") {
        return Err(AppError::new("byte range must contain exactly one '..'"));
    }
    let start = parse_size_u32(start)?;
    let end = parse_size_u32(end)?;
    if end < start {
        return Err(AppError::new(format!(
            "byte range start 0x{start:08x} is after end 0x{end:08x}"
        )));
    }
    Ok(InclusiveByteRange { start, end })
}

fn parse_partition_list(input: &str) -> Result<Vec<String>> {
    let mut names = Vec::new();
    for raw_name in input.split(',') {
        let name = raw_name.trim();
        if name.is_empty() {
            return Err(AppError::new(
                "--partitions contains an empty partition name",
            ));
        }
        if find_nor_partition(name).is_none() {
            return Err(AppError::new(format!(
                "unknown NOR partition '{name}'; known partitions: {}",
                known_partition_names()
            )));
        }
        names.push(name.to_string());
    }
    if names.is_empty() {
        return Err(AppError::new(
            "--partitions requires at least one partition",
        ));
    }
    Ok(names)
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
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] restore <firmware-qspi.bin> [--offset SIZE] [--byte-range START..END | --partitions NAME[,NAME...]] [--no-erase] [--no-verify]
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] verify <firmware-qspi.bin> [--offset SIZE]
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] erase [--offset SIZE] --length image|full|SIZE
  mono-uart-recovery --device /dev/ttyUSB0 [--stage1 mono-uart-recovery-stage1.bin] [--fast-baud 921600] crc32 --offset SIZE --length SIZE

Restore selectors:
  --byte-range START..END selects all eraseblocks touched by the inclusive byte range.
  Mono Gateway NOR eraseblocks are 64 KiB.
  --partitions accepts: rcw-bl2, uboot, uboot-env, fman-ucode, recovery-dtb, kernel-initramfs"
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_info() -> proto::DeviceInfo {
        proto::DeviceInfo {
            flash_size: 64 * MIB,
            image_size: 32 * MIB,
            erase_size: 64 * 1024,
            write_granule: 1,
            max_data: proto::MAX_DATA_LEN as u16,
            jedec_id: 0,
        }
    }

    fn args(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| value.to_string()).collect()
    }

    #[test]
    fn byte_range_parser_accepts_decimal_and_hex() {
        assert_eq!(
            parse_byte_range_arg("1048576..0x1fffff").unwrap(),
            InclusiveByteRange {
                start: MIB,
                end: 2 * MIB - 1
            }
        );
    }

    #[test]
    fn byte_range_resolves_to_touched_eraseblocks() {
        let selector = RestoreSelector::ByteRange(InclusiveByteRange {
            start: MIB + 1,
            end: MIB + 64 * 1024,
        });
        let ranges = resolve_restore_ranges(&test_info(), 0, 32 * MIB, &selector).unwrap();

        assert_eq!(
            ranges,
            vec![RestoreRange {
                image_offset: MIB,
                flash_offset: MIB,
                write_len: 2 * 64 * 1024,
                erase_len: 2 * 64 * 1024,
            }]
        );
    }

    #[test]
    fn partitions_use_corrected_mono_gateway_layout() {
        let selector = RestoreSelector::Partitions(vec![
            "uboot".to_string(),
            "uboot-env".to_string(),
            "kernel-initramfs".to_string(),
        ]);
        let ranges = resolve_restore_ranges(&test_info(), 0, 32 * MIB, &selector).unwrap();

        assert_eq!(
            ranges,
            vec![
                RestoreRange {
                    image_offset: MIB,
                    flash_offset: MIB,
                    write_len: 3 * MIB,
                    erase_len: 3 * MIB,
                },
                RestoreRange {
                    image_offset: 10 * MIB,
                    flash_offset: 10 * MIB,
                    write_len: 22 * MIB,
                    erase_len: 22 * MIB,
                },
            ]
        );
    }

    #[test]
    fn restore_parser_rejects_multiple_selectors() {
        let err = parse_command(&args(&[
            "restore",
            "firmware.bin",
            "--partitions=uboot",
            "--byte-range=0..0xffff",
        ]))
        .unwrap_err();

        assert!(err
            .to_string()
            .contains("restore accepts only one of --byte-range or --partitions"));
    }

    #[test]
    fn restore_parser_accepts_partition_selector() {
        let command = parse_command(&args(&[
            "restore",
            "firmware.bin",
            "--partitions=rcw-bl2,uboot",
        ]))
        .unwrap();

        match command {
            Command::Restore { selector, .. } => {
                assert_eq!(
                    selector,
                    RestoreSelector::Partitions(vec!["rcw-bl2".to_string(), "uboot".to_string()])
                );
            }
            other => panic!("unexpected command: {other:?}"),
        }
    }

    #[test]
    fn restore_parser_rejects_unallocated_partition() {
        let err = parse_command(&args(&[
            "restore",
            "firmware.bin",
            "--partitions=unallocated",
        ]))
        .unwrap_err();

        assert!(err
            .to_string()
            .contains("unknown NOR partition 'unallocated'"));
    }
}
