use std::fs::{File, OpenOptions};
use std::io::{self, Read, Write};
use std::mem::MaybeUninit;
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::thread;
use std::time::{Duration, Instant};

const O_NOCTTY: i32 = 0o400;
const O_NONBLOCK: i32 = 0o4000;

const NCCS: usize = 32;

const IGNBRK: u32 = 0x0001;
const BRKINT: u32 = 0x0002;
const PARMRK: u32 = 0x0008;
const ISTRIP: u32 = 0x0020;
const INLCR: u32 = 0x0040;
const IGNCR: u32 = 0x0080;
const ICRNL: u32 = 0x0100;
const IXON: u32 = 0x0400;
const IXANY: u32 = 0x0800;
const IXOFF: u32 = 0x1000;

const OPOST: u32 = 0x0001;

const ISIG: u32 = 0x0001;
const ICANON: u32 = 0x0002;
const ECHO: u32 = 0x0008;
const ECHONL: u32 = 0x0040;
const IEXTEN: u32 = 0x8000;

const CSIZE: u32 = 0x0030;
const CS8: u32 = 0x0030;
const CSTOPB: u32 = 0x0040;
const CREAD: u32 = 0x0080;
const PARENB: u32 = 0x0100;
const CLOCAL: u32 = 0x0800;
const CRTSCTS: u32 = 0x8000_0000;

const VTIME: usize = 5;
const VMIN: usize = 6;

const TCSANOW: i32 = 0;
const TCIOFLUSH: i32 = 2;

const B9600: u32 = 0x000d;
const B19200: u32 = 0x000e;
const B38400: u32 = 0x000f;
const B57600: u32 = 0x1001;
const B115200: u32 = 0x1002;
const B230400: u32 = 0x1003;
const B460800: u32 = 0x1004;
const B500000: u32 = 0x1005;
const B576000: u32 = 0x1006;
const B921600: u32 = 0x1007;
const B1000000: u32 = 0x1008;
const B1152000: u32 = 0x1009;
const B1500000: u32 = 0x100a;
const B2000000: u32 = 0x100b;
const B2500000: u32 = 0x100c;
const B3000000: u32 = 0x100d;
const B3500000: u32 = 0x100e;
const B4000000: u32 = 0x100f;

#[repr(C)]
#[derive(Clone, Copy)]
struct Termios {
    c_iflag: u32,
    c_oflag: u32,
    c_cflag: u32,
    c_lflag: u32,
    c_line: u8,
    c_cc: [u8; NCCS],
    c_ispeed: u32,
    c_ospeed: u32,
}

unsafe extern "C" {
    fn tcgetattr(fd: i32, termios_p: *mut Termios) -> i32;
    fn tcsetattr(fd: i32, optional_actions: i32, termios_p: *const Termios) -> i32;
    fn cfsetspeed(termios_p: *mut Termios, speed: u32) -> i32;
    fn tcflush(fd: i32, queue_selector: i32) -> i32;
}

pub struct SerialPort {
    file: File,
    original: Termios,
}

impl SerialPort {
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(O_NOCTTY | O_NONBLOCK)
            .open(path)?;
        let fd = file.as_raw_fd();
        let original = get_termios(fd)?;
        let mut raw = original;

        raw.c_iflag &=
            !(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
        raw.c_oflag &= !OPOST;
        raw.c_lflag &= !(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        raw.c_cflag &= !(CSIZE | PARENB | CSTOPB | CRTSCTS);
        raw.c_cflag |= CS8 | CREAD | CLOCAL;
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;

        let speed = baud_to_speed(baud).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("unsupported baud rate: {baud}"),
            )
        })?;

        cvt(unsafe { cfsetspeed(&mut raw, speed) })?;
        cvt(unsafe { tcsetattr(fd, TCSANOW, &raw) })?;
        cvt(unsafe { tcflush(fd, TCIOFLUSH) })?;

        Ok(Self { file, original })
    }

    pub fn write_all_retry(&mut self, mut bytes: &[u8]) -> io::Result<()> {
        while !bytes.is_empty() {
            match self.file.write(bytes) {
                Ok(0) => thread::sleep(Duration::from_millis(1)),
                Ok(n) => bytes = &bytes[n..],
                Err(err) if err.kind() == io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(1));
                }
                Err(err) => return Err(err),
            }
        }
        self.file.flush()
    }

    pub fn read_byte_until(&mut self, deadline: Instant) -> io::Result<Option<u8>> {
        let mut byte = [0u8; 1];
        loop {
            match self.file.read(&mut byte) {
                Ok(1) => return Ok(Some(byte[0])),
                Ok(_) => {}
                Err(err) if err.kind() == io::ErrorKind::WouldBlock => {}
                Err(err) if err.kind() == io::ErrorKind::Interrupted => {}
                Err(err) => return Err(err),
            }

            if Instant::now() >= deadline {
                return Ok(None);
            }
            thread::sleep(Duration::from_millis(1));
        }
    }
}

impl Drop for SerialPort {
    fn drop(&mut self) {
        let _ = unsafe { tcsetattr(self.file.as_raw_fd(), TCSANOW, &self.original) };
    }
}

fn get_termios(fd: RawFd) -> io::Result<Termios> {
    let mut termios = MaybeUninit::<Termios>::uninit();
    cvt(unsafe { tcgetattr(fd, termios.as_mut_ptr()) })?;
    Ok(unsafe { termios.assume_init() })
}

fn cvt(ret: i32) -> io::Result<()> {
    if ret == -1 {
        Err(io::Error::last_os_error())
    } else {
        Ok(())
    }
}

fn baud_to_speed(baud: u32) -> Option<u32> {
    match baud {
        9600 => Some(B9600),
        19200 => Some(B19200),
        38400 => Some(B38400),
        57600 => Some(B57600),
        115200 => Some(B115200),
        230400 => Some(B230400),
        460800 => Some(B460800),
        500000 => Some(B500000),
        576000 => Some(B576000),
        921600 => Some(B921600),
        1000000 => Some(B1000000),
        1152000 => Some(B1152000),
        1500000 => Some(B1500000),
        2000000 => Some(B2000000),
        2500000 => Some(B2500000),
        3000000 => Some(B3000000),
        3500000 => Some(B3500000),
        4000000 => Some(B4000000),
        _ => None,
    }
}
