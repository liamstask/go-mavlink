package mavlink

import (
	"bytes"
	"testing"
)

func TestRoundTrip(t *testing.T) {

	cases := []struct{ seq uint32 }{
		{12345},
	}

	for _, c := range cases {
		p := Ping{
			Seq: c.seq,
		}

		var pkt Packet
		if err := p.Pack(&pkt); err != nil {
			t.Errorf("Pack fail %q (%q)", pkt, err)
		}

		var buf bytes.Buffer

		if err := NewEncoder(&buf).EncodePacket(&pkt); err != nil {
			t.Errorf("Encode fail %q", err)
		}

		pktOut, err := NewDecoder(&buf).Decode()
		if err != nil {
			t.Errorf("Decode fail %q", err)
		}

		if pktOut.MsgID != MSG_ID_PING {
			t.Errorf("MsgID fail, want %q, got %q", MSG_ID_PING, pktOut.MsgID)
		}

		var pingOut Ping
		if err := pingOut.Unpack(pktOut); err != nil {
			t.Errorf("Unpack fail %q", err)
		}

		if pingOut.Seq != c.seq {
			t.Errorf("Mismatch msg field, got %q, want %q", pingOut.Seq, c.seq)
		}
	}
}

func TestDecode(t *testing.T) {
	// decode a known good byte stream
	pktbytes := []byte{0xfe, 0x09, 0x0, 0x01, 0xC8, 0x00, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x5A, 0x3E}
	_, err := NewDecoder(bytes.NewBuffer(pktbytes)).Decode()
	if err != nil {
		t.Errorf("Decode fail:", err)
	}
}
