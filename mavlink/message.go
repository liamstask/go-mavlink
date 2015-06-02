package mavlink

import (
	"bufio"
	"errors"
	"io"

	"github.com/liamstask/go-mavlink/x25"
)

//go:generate mavgen -f definitions/common.xml
//go:generate mavgen -f definitions/ardupilotmega.xml
//go:generate mavgen -f definitions/ASLUAV.xml
//go:generate mavgen -f definitions/matrixpilot.xml
//go:generate mavgen -f definitions/pixhawk.xml
//go:generate mavgen -f definitions/ualberta.xml

const (
	startByte        = 0xfe
	numChecksumBytes = 2
)

var (
	ErrUnknownMsgID = errors.New("unknown msg id")
	ErrCrcFail      = errors.New("checksum did not match")
)

// basic type for encoding/decoding mavlink messages.
// use the Pack() and Unpack() routines on specific message
// types to convert them to/from the Packet type.
type Message interface {
	Pack(*Packet) error
	Unpack(*Packet) error
}

// wire type for encoding/decoding mavlink messages.
// use the ToPacket() and FromPacket() routines on specific message
// types to convert them to/from the Message type.
type Packet struct {
	SeqID    uint8 // Sequence of packet
	SysID    uint8 // ID of message sender system/aircraft
	CompID   uint8 // ID of the message sender component
	MsgID    uint8 // ID of message in payload
	Payload  []byte
	Checksum uint16
}

type Decoder struct {
	CurrSeqID uint8      // last seq id decoded
	Dialects  []*Dialect // dialects that can be decoded
	br        *bufio.Reader
}

type Encoder struct {
	CurrSeqID uint8      // last seq id encoded
	Dialects  []*Dialect // dialects that can be encoded
	bw        *bufio.Writer
}

func NewDecoder(r io.Reader) *Decoder {
	d := &Decoder{
		Dialects: []*Dialect{DialectCommon},
	}

	if v, ok := r.(*bufio.Reader); ok {
		d.br = v
	} else {
		d.br = bufio.NewReader(r)
	}

	return d
}

func NewEncoder(w io.Writer) *Encoder {

	e := &Encoder{
		Dialects: []*Dialect{DialectCommon},
	}

	if v, ok := w.(*bufio.Writer); ok {
		e.bw = v
	} else {
		e.bw = bufio.NewWriter(w)
	}

	return e
}

// Decoder reads and parses from its reader
// Typically, the caller will check the p.MsgID to see if it's
// a message they're interested in, and convert it to the
// corresponding type via Message.FromPacket()
func (dec *Decoder) Decode() (*Packet, error) {

	// discard bytes until our start byte
	for {
		c, err := dec.br.ReadByte()
		if err != nil {
			return nil, err
		}
		if c == startByte {
			break
		}
	}

	// hdr contains LENGTH, SEQ, SYSID, COMPID, MSGID
	hdr := make([]byte, 5)
	if _, err := io.ReadFull(dec.br, hdr); err != nil {
		return nil, err
	}

	payloadLen := hdr[0]
	p := &Packet{
		SeqID:  hdr[1],
		SysID:  hdr[2],
		CompID: hdr[3],
		MsgID:  hdr[4],
	}

	crc := x25.New()
	crc.Write(hdr)

	// read payload (if there is one) and checksum bytes
	buf := make([]byte, payloadLen+numChecksumBytes)
	n, err := io.ReadFull(dec.br, buf)
	if err != nil {
		return nil, err
	}

	p.Payload = buf[:n-numChecksumBytes]
	crc.Write(p.Payload)

	crcx, err := findCrcX(dec.Dialects, p.MsgID)
	if err != nil {
		return nil, err
	}
	crc.WriteByte(crcx)

	p.Checksum = bytesToU16(buf[n-numChecksumBytes:])

	// does the transmitted checksum match our computed checksum?
	if p.Checksum != crc.Sum16() {
		return nil, ErrCrcFail
	}

	return p, nil
}

// helper that accepts a Message, internally converts it to a Packet,
// sets the Packet's SeqID based on the
// and then writes it to its writer via EncodePacket()
func (enc *Encoder) Encode(sysID, compID uint8, m Message) error {
	var p Packet
	if err := m.Pack(&p); err != nil {
		return err
	}

	p.SysID, p.CompID = sysID, compID

	return enc.EncodePacket(&p)
}

// Encode writes p to its writer
func (enc *Encoder) EncodePacket(p *Packet) error {

	crc := x25.New()

	// header
	hdr := []byte{startByte, byte(len(p.Payload)), enc.CurrSeqID, p.SysID, p.CompID, p.MsgID}
	if err := enc.writeAndCheck(hdr); err != nil {
		return err
	}
	crc.Write(hdr[1:]) // don't include start byte

	// payload
	if err := enc.writeAndCheck(p.Payload); err != nil {
		return err
	}
	crc.Write(p.Payload)

	// crc extra
	crcx, err := findCrcX(enc.Dialects, p.MsgID)
	if err != nil {
		return err
	}
	crc.WriteByte(crcx)

	// crc
	crcBytes := u16ToBytes(crc.Sum16())
	if err := enc.writeAndCheck(crcBytes); err != nil {
		return err
	}

	err = enc.bw.Flush()
	if err == nil {
		enc.CurrSeqID++
	}

	return err
}

// helper to check both the write and writelen status
func (enc *Encoder) writeAndCheck(p []byte) error {
	n, err := enc.bw.Write(p)
	if err == nil && n != len(p) {
		return io.ErrShortWrite
	}

	return err
}

func u16ToBytes(v uint16) []byte {
	return []byte{byte(v & 0xff), byte(v >> 8)}
}

func bytesToU16(p []byte) uint16 {
	// NB: does not check size of p
	return (uint16(p[1]) << 8) | uint16(p[0])
}
