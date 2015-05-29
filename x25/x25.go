package x25

// implements hash.Hash
type X25 struct {
	crc uint16
}

func New() *X25 {
	x := &X25{}
	x.Reset()
	return x
}

func (x *X25) Write(p []byte) (n int, err error) {
	for _, b := range p {
		tmp := b ^ byte(x.crc&0xff)
		tmp ^= (tmp << 4)
		x.crc = (x.crc >> 8) ^ (uint16(tmp) << 8) ^ (uint16(tmp) << 3) ^ (uint16(tmp) >> 4)
	}
	return len(p), nil
}

func (x *X25) Sum16() uint16 { return x.crc }

func (x *X25) Sum(in []byte) []byte {
	s := x.Sum16()
	return append(in, byte(s>>8), byte(s))
}

func (x *X25) Size() int { return 2 }

func (x *X25) BlockSize() int { return 1 }

func (x *X25) Reset() { x.crc = 0xffff }
