// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//! V4L2 capture, with Jetson hardware JPEG (VIC colour convert + NVJPG encode) through `jetson-multimedia`.

/// One captured frame, borrowed from the driver until the next `next` call.
pub struct Frame<'a> {
    /// JPEG when `jpeg` is set (hardware encoded), else the raw frame in the opened fourcc.
    pub data: &'a [u8],
    /// Bytes per row of a raw frame.
    pub stride: usize,
    pub jpeg: bool,
    /// The driver's capture timestamp, on whatever clock the driver stamps with.
    pub driver_stamp_s: f64,
    /// Time the hardware spent converting and encoding a `jpeg` frame.
    pub encode_s: f64,
}

/// The four-character code V4L2 names a pixel format by.
pub fn fourcc(code: &str) -> Result<u32, String> {
    let bytes = code.as_bytes();
    if bytes.len() != 4 {
        return Err(format!("fourcc {code:?} is not four characters"));
    }
    Ok(u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
}

#[cfg(target_os = "linux")]
pub use linux::Capture;

#[cfg(not(target_os = "linux"))]
pub struct Capture;

#[cfg(not(target_os = "linux"))]
impl Capture {
    pub fn open(_: &str, _: u32, _: u32, _: u32, _: bool, _: i32) -> Result<Self, String> {
        Err("V4L2 capture needs Linux".into())
    }

    pub fn is_hardware(&self) -> bool {
        false
    }

    pub fn next(&mut self, _: i32) -> Result<Option<Frame<'_>>, String> {
        Err("V4L2 capture needs Linux".into())
    }
}

#[cfg(target_os = "linux")]
mod v4l2 {
    //! The slice of `linux/videodev2.h` capture needs, for 64-bit Linux.

    use std::os::raw::{c_int, c_ulong};

    pub const BUF_TYPE_VIDEO_CAPTURE: u32 = 1;
    pub const MEMORY_MMAP: u32 = 1;
    pub const MEMORY_DMABUF: u32 = 4;
    pub const FIELD_ANY: u32 = 0;

    pub const VIDIOC_S_FMT: c_ulong = 0xc0d0_5605;
    pub const VIDIOC_REQBUFS: c_ulong = 0xc014_5608;
    pub const VIDIOC_QUERYBUF: c_ulong = 0xc058_5609;
    pub const VIDIOC_QBUF: c_ulong = 0xc058_560f;
    pub const VIDIOC_DQBUF: c_ulong = 0xc058_5611;
    pub const VIDIOC_STREAMON: c_ulong = 0x4004_5612;
    pub const VIDIOC_STREAMOFF: c_ulong = 0x4004_5613;

    #[repr(C)]
    #[derive(Clone, Copy, Default)]
    pub struct PixFormat {
        pub width: u32,
        pub height: u32,
        pub pixelformat: u32,
        pub field: u32,
        pub bytesperline: u32,
        pub sizeimage: u32,
        pub colorspace: u32,
        pub private: u32,
        pub flags: u32,
        pub ycbcr_enc: u32,
        pub quantization: u32,
        pub xfer_func: u32,
    }

    /// `struct v4l2_format`: a 200-byte union, 8-aligned by its pointer-bearing members.
    #[repr(C)]
    pub struct Format {
        pub kind: u32,
        pub fmt: FormatUnion,
    }

    #[repr(C)]
    pub union FormatUnion {
        pub pix: PixFormat,
        _raw: [u64; 25],
    }

    impl Format {
        pub fn capture(pix: PixFormat) -> Self {
            let mut fmt = FormatUnion { _raw: [0; 25] };
            fmt.pix = pix;
            Format {
                kind: BUF_TYPE_VIDEO_CAPTURE,
                fmt,
            }
        }

        pub fn pix(&self) -> PixFormat {
            // SAFETY: a capture format always holds `pix`.
            unsafe { self.fmt.pix }
        }
    }

    #[repr(C)]
    #[derive(Default)]
    pub struct RequestBuffers {
        pub count: u32,
        pub kind: u32,
        pub memory: u32,
        pub capabilities: u32,
        pub flags: u8,
        pub reserved: [u8; 3],
    }

    #[repr(C)]
    #[derive(Default)]
    pub struct Timecode {
        pub kind: u32,
        pub flags: u32,
        pub frames: u8,
        pub seconds: u8,
        pub minutes: u8,
        pub hours: u8,
        pub userbits: [u8; 4],
    }

    #[repr(C)]
    pub struct Buffer {
        pub index: u32,
        pub kind: u32,
        pub bytesused: u32,
        pub flags: u32,
        pub field: u32,
        pub timestamp: libc::timeval,
        pub timecode: Timecode,
        pub sequence: u32,
        pub memory: u32,
        /// `m.offset` (MMAP) or `m.fd` (DMABUF) in the low 32 bits of an 8-byte union.
        pub m: u64,
        pub length: u32,
        pub reserved2: u32,
        pub request_fd: c_int,
    }

    impl Buffer {
        pub fn new(memory: u32, index: u32) -> Self {
            // SAFETY: plain integers, all-zero is valid.
            let mut buffer: Buffer = unsafe { std::mem::zeroed() };
            buffer.kind = BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = memory;
            buffer.index = index;
            buffer
        }

        pub fn set_fd(&mut self, fd: c_int) {
            self.m = fd as u32 as u64;
        }

        pub fn offset(&self) -> u32 {
            self.m as u32
        }
    }
}

#[cfg(target_os = "linux")]
mod linux {
    use std::ffi::CString;
    use std::os::raw::{c_int, c_ulong, c_void};
    use std::time::Instant;

    use jetson_multimedia::{ColorFormat, Filter, Jetson, JpegEncoder, Surface, Tag};

    use super::v4l2::*;
    use super::Frame;

    const BUFFERS: u32 = 4;

    /// The hardware path: the camera fills NvBufSurface dmabufs, the VIC converts to YUV420, NVJPG encodes.
    struct Hardware {
        jetson: Jetson,
        camera: Vec<Surface>,
        yuv420: Surface,
        encoder: JpegEncoder,
    }

    /// An open V4L2 capture device.
    pub struct Capture {
        fd: c_int,
        maps: Vec<(*mut c_void, usize)>,
        stride: u32,
        quality: u8,
        hardware: Option<Hardware>,
        held: Option<u32>,
    }

    // One owner drives the device at a time; the mapped buffers belong to it.
    unsafe impl Send for Capture {}

    fn ioctl<T>(fd: c_int, request: c_ulong, arg: &mut T) -> std::io::Result<()> {
        loop {
            // SAFETY: arg is the struct the request expects, sized to match its encoded length.
            if unsafe { libc::ioctl(fd, request as _, arg as *mut T) } >= 0 {
                return Ok(());
            }
            let error = std::io::Error::last_os_error();
            if error.kind() != std::io::ErrorKind::Interrupted {
                return Err(error);
            }
        }
    }

    /// The NvBufSurface format the VIC reads a packed 4:2:2 fourcc as.
    fn surface_format(fourcc: u32) -> Option<ColorFormat> {
        match &fourcc.to_le_bytes() {
            b"UYVY" => Some(ColorFormat::Uyvy),
            b"YUYV" => Some(ColorFormat::Yuyv),
            b"VYUY" => Some(ColorFormat::Vyuy),
            b"YVYU" => Some(ColorFormat::Yvyu),
            _ => None,
        }
    }

    impl Capture {
        /// Opens and starts streaming. With `hardware`, frames come back JPEG-encoded at `jpeg_quality` when the
        /// Jetson encoder is available; otherwise (or without it) they come back raw.
        pub fn open(
            device: &str,
            width: u32,
            height: u32,
            fourcc: u32,
            hardware: bool,
            jpeg_quality: i32,
        ) -> Result<Self, String> {
            if hardware && jpeg_quality > 0 {
                match Self::start(device, width, height, fourcc, true, jpeg_quality) {
                    Ok(capture) => return Ok(capture),
                    // A fresh fd for the CPU path: the failed REQBUFS may have left this one claimed.
                    Err(error) => tracing::info!(%error, "no hardware JPEG, capturing raw"),
                }
            }
            Self::start(device, width, height, fourcc, false, jpeg_quality)
        }

        fn start(
            device: &str,
            width: u32,
            height: u32,
            fourcc: u32,
            hardware: bool,
            jpeg_quality: i32,
        ) -> Result<Self, String> {
            let path = CString::new(device).map_err(|e| e.to_string())?;
            // SAFETY: path is NUL-terminated.
            let fd = unsafe { libc::open(path.as_ptr(), libc::O_RDWR | libc::O_NONBLOCK) };
            if fd < 0 {
                return Err(format!(
                    "open {device}: {}",
                    std::io::Error::last_os_error()
                ));
            }
            let mut capture = Capture {
                fd,
                maps: Vec::new(),
                stride: 0,
                quality: jpeg_quality.clamp(1, 100) as u8,
                hardware: None,
                held: None,
            };
            capture.set_format(width, height, fourcc)?;
            if hardware {
                capture.start_hardware(width, height, fourcc)?;
            } else {
                capture.start_mmap()?;
            }
            let mut kind = BUF_TYPE_VIDEO_CAPTURE as c_int;
            ioctl(fd, VIDIOC_STREAMON, &mut kind).map_err(|e| format!("VIDIOC_STREAMON: {e}"))?;
            Ok(capture)
        }

        fn set_format(&mut self, width: u32, height: u32, fourcc: u32) -> Result<(), String> {
            let mut format = Format::capture(PixFormat {
                width,
                height,
                pixelformat: fourcc,
                field: FIELD_ANY,
                ..Default::default()
            });
            ioctl(self.fd, VIDIOC_S_FMT, &mut format).map_err(|e| format!("VIDIOC_S_FMT: {e}"))?;
            let pix = format.pix();
            if (pix.width, pix.height) != (width, height) {
                return Err(format!(
                    "driver gave {}x{}, not {width}x{height}",
                    pix.width, pix.height
                ));
            }
            self.stride = pix.bytesperline;
            Ok(())
        }

        fn request(&self, memory: u32) -> Result<(), String> {
            let mut request = RequestBuffers {
                count: BUFFERS,
                kind: BUF_TYPE_VIDEO_CAPTURE,
                memory,
                ..Default::default()
            };
            ioctl(self.fd, VIDIOC_REQBUFS, &mut request)
                .map_err(|e| format!("VIDIOC_REQBUFS: {e}"))?;
            if request.count != BUFFERS {
                return Err(format!(
                    "driver gave {} buffers, not {BUFFERS}",
                    request.count
                ));
            }
            Ok(())
        }

        fn start_mmap(&mut self) -> Result<(), String> {
            self.request(MEMORY_MMAP)?;
            for index in 0..BUFFERS {
                let mut buffer = Buffer::new(MEMORY_MMAP, index);
                ioctl(self.fd, VIDIOC_QUERYBUF, &mut buffer)
                    .map_err(|e| format!("VIDIOC_QUERYBUF: {e}"))?;
                // SAFETY: maps the driver buffer QUERYBUF just described; unmapped in Drop.
                let map = unsafe {
                    libc::mmap(
                        std::ptr::null_mut(),
                        buffer.length as usize,
                        libc::PROT_READ,
                        libc::MAP_SHARED,
                        self.fd,
                        buffer.offset() as libc::off_t,
                    )
                };
                if map == libc::MAP_FAILED {
                    return Err(format!("mmap: {}", std::io::Error::last_os_error()));
                }
                self.maps.push((map, buffer.length as usize));
                ioctl(self.fd, VIDIOC_QBUF, &mut buffer)
                    .map_err(|e| format!("VIDIOC_QBUF: {e}"))?;
            }
            Ok(())
        }

        // Only UYVY-family cameras: the VIC converts them, NVJPG encodes YUV420.
        fn start_hardware(&mut self, width: u32, height: u32, fourcc: u32) -> Result<(), String> {
            let format = surface_format(fourcc).ok_or("no hardware path for this pixel format")?;
            let jetson = Jetson::load().map_err(|e| e.to_string())?;
            let camera = (0..BUFFERS)
                .map(|_| Surface::new(&jetson, width, height, format, Tag::Camera))
                .collect::<Result<Vec<_>, _>>()
                .map_err(|e| e.to_string())?;
            let yuv420 = Surface::new(&jetson, width, height, ColorFormat::Yuv420, Tag::None)
                .map_err(|e| e.to_string())?;
            let encoder = JpegEncoder::new(&jetson, width, height).map_err(|e| e.to_string())?;
            self.request(MEMORY_DMABUF)?;
            for (index, surface) in camera.iter().enumerate() {
                let mut buffer = Buffer::new(MEMORY_DMABUF, index as u32);
                ioctl(self.fd, VIDIOC_QUERYBUF, &mut buffer)
                    .map_err(|e| format!("VIDIOC_QUERYBUF dmabuf: {e}"))?;
                buffer.set_fd(surface.fd());
                ioctl(self.fd, VIDIOC_QBUF, &mut buffer)
                    .map_err(|e| format!("VIDIOC_QBUF dmabuf: {e}"))?;
            }
            self.hardware = Some(Hardware {
                jetson,
                camera,
                yuv420,
                encoder,
            });
            Ok(())
        }

        /// Whether frames come back JPEG-encoded by the hardware.
        pub fn is_hardware(&self) -> bool {
            self.hardware.is_some()
        }

        fn memory(&self) -> u32 {
            if self.hardware.is_some() {
                MEMORY_DMABUF
            } else {
                MEMORY_MMAP
            }
        }

        fn release(&mut self) {
            let Some(index) = self.held.take() else {
                return;
            };
            let mut buffer = Buffer::new(self.memory(), index);
            if let Some(hardware) = &self.hardware {
                buffer.set_fd(hardware.camera[index as usize].fd());
            }
            let _ = ioctl(self.fd, VIDIOC_QBUF, &mut buffer);
        }

        /// The next frame, None on timeout. The frame borrows the driver's buffer until the next call.
        pub fn next(&mut self, timeout_ms: i32) -> Result<Option<Frame<'_>>, String> {
            self.release();
            let mut poll = libc::pollfd {
                fd: self.fd,
                events: libc::POLLIN,
                revents: 0,
            };
            // SAFETY: one valid pollfd.
            match unsafe { libc::poll(&mut poll, 1, timeout_ms) } {
                0 => return Ok(None),
                ready if ready < 0 => {
                    let error = std::io::Error::last_os_error();
                    if error.kind() == std::io::ErrorKind::Interrupted {
                        return Ok(None);
                    }
                    return Err(format!("poll: {error}"));
                }
                _ => {}
            }
            let mut buffer = Buffer::new(self.memory(), 0);
            if let Err(error) = ioctl(self.fd, VIDIOC_DQBUF, &mut buffer) {
                if error.kind() == std::io::ErrorKind::WouldBlock {
                    return Ok(None);
                }
                return Err(format!("VIDIOC_DQBUF: {error}"));
            }
            self.held = Some(buffer.index);
            let driver_stamp_s =
                buffer.timestamp.tv_sec as f64 + buffer.timestamp.tv_usec as f64 * 1e-6;
            let stride = self.stride as usize;
            if let Some(hardware) = &mut self.hardware {
                let start = Instant::now();
                let camera = &hardware.camera[buffer.index as usize];
                hardware
                    .jetson
                    .transform(camera, &hardware.yuv420, Filter::Nearest)
                    .map_err(|e| e.to_string())?;
                let data = hardware
                    .encoder
                    .encode(&hardware.yuv420, self.quality)
                    .map_err(|e| e.to_string())?;
                return Ok(Some(Frame {
                    data,
                    stride,
                    jpeg: true,
                    driver_stamp_s,
                    encode_s: start.elapsed().as_secs_f64(),
                }));
            }
            let (map, _) = self.maps[buffer.index as usize];
            Ok(Some(Frame {
                // SAFETY: the buffer stays dequeued until the next call, which needs &mut self and so cannot
                // happen while this borrow lives.
                data: unsafe {
                    std::slice::from_raw_parts(map.cast::<u8>(), buffer.bytesused as usize)
                },
                stride,
                jpeg: false,
                driver_stamp_s,
                encode_s: 0.0,
            }))
        }
    }

    impl Drop for Capture {
        fn drop(&mut self) {
            let mut kind = BUF_TYPE_VIDEO_CAPTURE as c_int;
            let _ = ioctl(self.fd, VIDIOC_STREAMOFF, &mut kind);
            for &(map, length) in &self.maps {
                // SAFETY: mapped in start_mmap and unmapped once.
                unsafe { libc::munmap(map, length) };
            }
            // The surfaces outlive the stream: close the device before they are destroyed.
            // SAFETY: fd is owned by self and closed once.
            unsafe { libc::close(self.fd) };
            self.hardware.take();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fourcc_packs_little_endian_like_v4l2() {
        // V4L2_PIX_FMT_UYVY = v4l2_fourcc('U','Y','V','Y')
        assert_eq!(fourcc("UYVY").unwrap(), 0x5956_5955);
        assert!(fourcc("UYV").is_err());
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn v4l2_structs_match_videodev2_on_64_bit_linux() {
        use std::mem::{offset_of, size_of};
        // Measured with offsetof/sizeof against linux/videodev2.h on aarch64.
        assert_eq!(size_of::<v4l2::Format>(), 208);
        assert_eq!(offset_of!(v4l2::Format, fmt), 8);
        assert_eq!(size_of::<v4l2::PixFormat>(), 48);
        assert_eq!(size_of::<v4l2::RequestBuffers>(), 20);
        assert_eq!(size_of::<v4l2::Buffer>(), 88);
        assert_eq!(offset_of!(v4l2::Buffer, timestamp), 24);
        assert_eq!(offset_of!(v4l2::Buffer, sequence), 56);
        assert_eq!(offset_of!(v4l2::Buffer, memory), 60);
        assert_eq!(offset_of!(v4l2::Buffer, m), 64);
        assert_eq!(offset_of!(v4l2::Buffer, length), 72);
    }
}
