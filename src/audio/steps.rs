use super::rhythm::RhythmData;

use alloc::boxed::Box;
use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;

use embedded_fatfs::{Dir, Error, OemCpConverter, ReadWriteSeek, TimeProvider};
use embedded_io_async::{Read, Seek, SeekFrom};
use micromath::F32Ext;

pub struct Steps<'a, IO: ReadWriteSeek, TP: TimeProvider, OCC: OemCpConverter> {
    root: Dir<'a, IO, TP, OCC>,
    metadatas: Vec<super::WavMetadata>,
}

impl<'a, IO, TP, OCC> Steps<'a, IO, TP, OCC>
where
    IO: ReadWriteSeek,
    TP: TimeProvider,
    OCC: OemCpConverter,
{
    async fn paths_recursive(dir: &Dir<'a, IO, TP, OCC>) -> Result<Vec<String>, Error<IO::Error>> {
        let mut children = Vec::new();
        let mut iter = dir.iter();
        while let Some(entry) = iter.next().await {
            let entry = entry?;
            let ucs2 = entry.long_file_name_as_ucs2_units();
            if let Some(Ok(name)) = ucs2.map(alloc::string::String::from_utf16) {
                if !entry.attributes().contains(embedded_fatfs::FileAttributes::HIDDEN)
                    && !name.starts_with(".")
                {
                    if entry.is_dir() {
                        let grandchildren = Box::pin(Self::paths_recursive(&dir.open_dir(&name).await?)).await?;
                        for grandchild in grandchildren {
                            children.push(format!("{}/{}", name, grandchild));
                        }
                    }
                    children.push(name);
                }
            }
        }
        Ok(children)
    }

    pub async fn new(root: Dir<'a, IO, TP, OCC>) -> Result<Self, Error<IO::Error>> {
        let paths = Self::paths_recursive(&root).await?;
        defmt::info!("paths: {}", defmt::Debug2Format(&paths));

        let mut metadatas = Vec::new();
        for path in paths {
            match root.open_file(&path).await {
                Err(Error::InvalidInput) => (),
                Err(e) => return Err(e),
                Ok(mut file) => {
                    let len = root.open_meta(&path).await?.len();
                    let rhythm = super::rhythm::RhythmData::new(&mut file, len).await?;

                    metadatas.push(super::WavMetadata { path, rhythm });
                }
            }
        }

        Ok(Self {
            root,
            metadatas,
        })
    }
}
