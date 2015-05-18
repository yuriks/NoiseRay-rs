use rand::SeedableRng;
use rand::Rng as StdRng;

pub type Rng = self::XorShift1024Star;

/// This is the XorShift1024* generator.
/// Based on the C code provided at http://xorshift.di.unimi.it/

pub struct XorShift1024Star {
    s: [u64; 16],
    p: usize,
}

impl XorShift1024Star {
    pub fn new_unseeded() -> XorShift1024Star {
        XorShift1024Star {
            s: [
                0xd00b2f639592faf0, 0x1b748bead91f729b, 0xf80b426766fb00ac, 0x1773f0bd9fe62d72,
                0xfa3824c82db94c37, 0xad6d6f013d1d50b8, 0x6d07ad8224ae4327, 0x5d58864e8bfc315d,
                0xa9a16f6ea16d0bf0, 0x51b453d66acc9d37, 0x956ef667add1bda2, 0xaf5aca0d291a915a,
                0xfa3c060d47b87d8d, 0x750b2ddb01452e91, 0x99d372e94f4d7619, 0x3d00c7f84ed77ebc,
            ],
            p: 0,
        }
    }
}

impl StdRng for XorShift1024Star {
    #[inline]
    fn next_u32(&mut self) -> u32 {
        self.next_u64() as u32
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut s0 = self.s[self.p];
        self.p = (self.p + 1) % 16;
        let mut s1 = self.s[self.p];
        s1 ^= s1 << 31;
        s1 ^= s1 >> 11;
        s0 ^= s0 >> 30;
        self.s[self.p] = s0 ^ s1;
        return self.s[self.p] * 1181783497276652981;
    }
}

impl SeedableRng<[u64; 16]> for XorShift1024Star {
    #[inline]
    fn reseed(&mut self, seed: [u64; 16]) {
        self.s = seed;
        self.p = 0;
    }

    #[inline]
    fn from_seed(seed: [u64; 16]) -> XorShift1024Star {
        XorShift1024Star {
            s: seed,
            p: 0,
        }
    }
}
