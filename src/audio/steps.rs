use super::rhythm::RhythmData;

use embedded_fatfs::{Dir, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};

pub struct Steps<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    root: Dir<'a, IO, TP, OCC>,
}
