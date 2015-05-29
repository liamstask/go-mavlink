package x25

import (
	// "hash"
	"testing"
)

func TestParsePacket(t *testing.T) {
	x := New()
	x.Sum16()
	// if h, ok := x.(*hash.Hash); !ok {
	//     t.Errorf("X25 does not implement hash.Hash")
	// }
}
