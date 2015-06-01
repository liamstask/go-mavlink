package mavlink

// Dialect represents a set of message definitions.
// Some dialects have conflicting definitions for given message IDs,
// so a list of dialects must be provided to an Encoder/Decoder in
// order to specify which packets to use for the conflicting IDs.
//
// The 'DialectCommon' dialect is added to all Encoders/Decoders by default.
type Dialect struct {
	Name      string
	crcExtras map[uint8]uint8
}

// look up the crcextra for msgid within the given list of Dialects
func findCrcX(ds []*Dialect, msgid uint8) (uint8, error) {

	// http://www.mavlink.org/mavlink/crc_extra_calculation
	for _, d := range ds {
		if crcx, ok := d.crcExtras[msgid]; ok {
			return crcx, nil
		}
	}

	return 0, ErrUnknownMsgID
}
