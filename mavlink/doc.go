/*
Package mavlink provides support for encoding/decoding mavlink (http://qgroundcontrol.org/mavlink/start) messages.

Encoding is done via the Encoder type, which wraps an io.Writer, and decoding is done via the Decoder type, which wraps an io.Reader.

The Packet type represents the wire type used for message transmission/reception, and a variety of generated classes each implement the Message type.
A Message can be converted to a Packet by calling Message.Pack(p) and a Packet can be converted to a Message by calling Message.Unpack(p).

See the ReadMe for examples of typical usage.

*/
package mavlink
