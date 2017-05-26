package mavlink

import (
	"bufio"
	"errors"
	"io"
	"sync"

	"github.com/cnord/go-mavlink/x25"
)

//go:generate mavgen -f definitions/common.xml
//go:generate mavgen -f definitions/ardupilotmega.xml
//go:generate mavgen -f definitions/ASLUAV.xml
//go:generate mavgen -f definitions/matrixpilot.xml
//go:generate mavgen -f definitions/ualberta.xml

const (
	startByte        = 0xfe
	numChecksumBytes = 2
	hdrLen           = 6
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
	MsgID() uint8
	MsgName() string
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
	sync.Mutex
	CurrSeqID uint8        // last seq id decoded
	Dialects  DialectSlice // dialects that can be decoded
	br        *bufio.Reader
	buffer    []byte // stores bytes we've read from br
}

type Encoder struct {
	sync.Mutex
	CurrSeqID uint8        // last seq id encoded
	Dialects  DialectSlice // dialects that can be encoded
	bw        *bufio.Writer
}

func NewDecoder(r io.Reader) *Decoder {
	d := &Decoder{
		Dialects: DialectSlice{DialectCommon},
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
		Dialects: DialectSlice{DialectCommon},
	}

	if v, ok := w.(*bufio.Writer); ok {
		e.bw = v
	} else {
		e.bw = bufio.NewWriter(w)
	}

	return e
}

// helper to create packet w/header populated with received bytes
func newPacketFromBytes(b []byte) (*Packet, int) {
	return &Packet{
		SeqID:  b[1],
		SysID:  b[2],
		CompID: b[3],
		MsgID:  b[4],
	}, int(b[0])
}

// Decoder reads and parses from its reader
// Typically, the caller will check the p.MsgID to see if it's
// a message they're interested in, and convert it to the
// corresponding type via Message.FromPacket()
func (dec *Decoder) Decode() (*Packet, error) {
	for {
		startFoundInBuffer := false
		// discard bytes in buffer before start byte
		for i, b := range dec.buffer {
			if b == startByte {
				dec.buffer = dec.buffer[i:]
				startFoundInBuffer = true
				break
			}
		}

		// if start not found, read until we see start byte
		if !startFoundInBuffer {
			for {
				c, err := dec.br.ReadByte()
				if err != nil {
					return nil, err
				}
				if c == startByte {
					dec.buffer = append(dec.buffer, c)
					break
				}
			}
		}

		if len(dec.buffer) < 2 {
			// read length byte
			bytesRead := make([]byte, 1)
			n, err := io.ReadAtLeast(dec.br, bytesRead, 1)
			if err != nil {
				return nil, err
			}

			dec.buffer = append(dec.buffer, bytesRead[:n]...)
		}

		// buffer[1] is LENGTH and we've already read len(buffer) bytes
		payloadLen := int(dec.buffer[1])
		packetLen := hdrLen + payloadLen + numChecksumBytes
		bytesNeeded := packetLen - len(dec.buffer)
		if bytesNeeded > 0 {
			bytesRead := make([]byte, bytesNeeded)
			n, err := io.ReadAtLeast(dec.br, bytesRead, bytesNeeded)
			if err != nil {
				return nil, err
			}
			dec.buffer = append(dec.buffer, bytesRead[:n]...)
		}

		// hdr contains LENGTH, SEQ, SYSID, COMPID, MSGID
		// (hdrLen - 1) because we don't include the start byte
		hdr := make([]byte, hdrLen-1)
		// don't include start byte
		hdr = dec.buffer[1:hdrLen]

		p, payloadLen := newPacketFromBytes(hdr)

		crc := x25.New()
		crc.Write(hdr)

		payloadStart := hdrLen
		p.Payload = dec.buffer[payloadStart : payloadStart+payloadLen]
		crc.Write(p.Payload)

		crcx, err := dec.Dialects.findCrcX(p.MsgID)
		if err != nil {
			dec.buffer = dec.buffer[1:]
			// return error here to allow caller to decide if stream is
			// corrupted or if we're getting the wrong dialect
			return p, err
		}
		crc.WriteByte(crcx)

		p.Checksum = bytesToU16(dec.buffer[payloadStart+payloadLen : payloadStart+payloadLen+numChecksumBytes])

		// does the transmitted checksum match our computed checksum?
		if p.Checksum != crc.Sum16() {
			// strip off start byte
			dec.buffer = dec.buffer[1:]
		} else {
			dec.CurrSeqID = p.SeqID
			dec.buffer = dec.buffer[packetLen:]
			return p, nil
		}
	}
}

// Decode a packet from a previously received buffer (such as a UDP packet),
// b must contain a complete message
func (dec *Decoder) DecodeBytes(b []byte) (*Packet, error) {

	if len(b) < hdrLen || b[0] != startByte {
		return nil, errors.New("invalid header")
	}

	p, payloadLen := newPacketFromBytes(b[1:])

	crc := x25.New()
	p.Payload = b[hdrLen : hdrLen+payloadLen]
	crc.Write(b[1 : hdrLen+payloadLen])

	crcx, err := dec.Dialects.findCrcX(p.MsgID)
	if err != nil {
		return p, err
	}
	crc.WriteByte(crcx)

	p.Checksum = bytesToU16(b[hdrLen+payloadLen:])

	// does the transmitted checksum match our computed checksum?
	if p.Checksum != crc.Sum16() {
		return p, ErrCrcFail
	}

	dec.CurrSeqID = p.SeqID
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
	crcx, err := enc.Dialects.findCrcX(p.MsgID)
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
