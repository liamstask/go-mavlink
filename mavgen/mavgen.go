package main

import (
	"bytes"
	"encoding/xml"
	"errors"
	"fmt"
	"go/format"
	"io"
	"io/ioutil"
	"sort"
	"strconv"
	"strings"
	"text/template"

	"github.com/cnord/go-mavlink/x25"
)

type Dialect struct {
	Name        string
	StringSizes map[int]bool

	XMLName  xml.Name   `xml:"mavlink"`
	Version  string     `xml:"version"`
	Include  string     `xml:"include"`
	Enums    []*Enum    `xml:"enums>enum"`
	Messages []*Message `xml:"messages>message"`
}

type Enum struct {
	Name        string       `xml:"name,attr"`
	Description string       `xml:"description"`
	Entries     []*EnumEntry `xml:"entry"`
}

type EnumEntry struct {
	Value       uint8             `xml:"value,attr"`
	Name        string            `xml:"name,attr"`
	Description string            `xml:"description"`
	Params      []*EnumEntryParam `xml:"param"`
}

type EnumEntryParam struct {
	Index       uint8  `xml:"index,attr"`
	Description string `xml:",innerxml"`
}

type Message struct {
	// use uint32 instead of uint8 so that we can filter
	// msgids from mavlink v2, which are 24 bits wide.
	// see filtering in ParseDialect()
	ID          uint32          `xml:"id,attr"`
	Name        string          `xml:"name,attr"`
	Description string          `xml:"description"`
	Fields      []*MessageField `xml:"field"`
}

type MessageField struct {
	CType       string `xml:"type,attr"`
	Name        string `xml:"name,attr"`
	Enum        string `xml:"enum,attr"`
	Description string `xml:",innerxml"`
	GoType      string
	BitSize     int
	ArrayLen    int
	ByteOffset  int // from beginning of payload
}

var funcMap = template.FuncMap{
	"UpperCamelCase": UpperCamelCase,
}

func (f *MessageField) SizeInBytes() int {
	if f.ArrayLen > 0 {
		return f.BitSize / 8 * f.ArrayLen
	} else {
		return f.BitSize / 8
	}
}

func (m *Message) Size() int {
	sz := 0
	for _, f := range m.Fields {
		sz += f.SizeInBytes()
	}
	return sz
}

// CRC extra calculation: http://www.mavlink.org/mavlink/crc_extra_calculation
func (m *Message) CRCExtra() uint8 {
	hash := x25.New()

	fmt.Fprint(hash, m.Name+" ")
	for _, f := range m.Fields {
		cType := f.CType
		if cType == "uint8_t_mavlink_version" {
			cType = "uint8_t"
		}
		// type name for crc extra purposes does not include array portion
		if idx := strings.IndexByte(cType, '['); idx >= 0 {
			cType = cType[:idx]
		}
		fmt.Fprint(hash, cType+" "+f.Name+" ")
		if f.ArrayLen > 0 {
			hash.WriteByte(byte(f.ArrayLen))
		}
	}

	crc := hash.Sum16()
	return uint8((crc & 0xFF) ^ (crc >> 8))
}

// implementation of sort.Interface for Message
func (m *Message) Len() int {
	return len(m.Fields)
}

func (m *Message) Less(i, j int) bool {
	return m.Fields[i].BitSize < m.Fields[j].BitSize
}

func (m *Message) Swap(i, j int) {
	m.Fields[i], m.Fields[j] = m.Fields[j], m.Fields[i]
}

// convert names to upper camel case
func UpperCamelCase(s string) string {
	var b bytes.Buffer
	for _, frag := range strings.Split(s, "_") {
		if frag != "" {
			b.WriteString(strings.ToUpper(frag[:1]))
			b.WriteString(strings.ToLower(frag[1:]))
		}
	}
	return b.String()
}

// helper to pack a single element into a payload.
// can be called for a single field, or an element within a field's array.
func (f *MessageField) payloadPackPrimitive(offset, name string) string {

	if f.BitSize == 8 {
		return fmt.Sprintf("payload[%s] = byte(%s)", offset, name)
	}

	if f.IsFloat() {
		switch f.BitSize {
		case 32, 64:
			return fmt.Sprintf("binary.LittleEndian.PutUint%d(payload[%s:], math.Float%dbits(%s))", f.BitSize, offset, f.BitSize, name)
		}
	} else {
		switch f.BitSize {
		case 16, 32, 64:
			return fmt.Sprintf("binary.LittleEndian.PutUint%d(payload[%s:], uint%d(%s))", f.BitSize, offset, f.BitSize, name)
		}
	}

	panic("unhandled bitsize")
}

// produce a string that will pack this message's fields
// into a byte slice called 'payload'
func (f *MessageField) PayloadPackSequence() string {
	name := UpperCamelCase(f.Name)

	if f.ArrayLen > 0 {
		// optimize to copy() if possible
		if strings.HasSuffix(f.GoType, "byte") || strings.HasSuffix(f.GoType, "uint8") {
			return fmt.Sprintf("copy(payload[%d:], self.%s[:])", f.ByteOffset, name)
		}

		// pack each element in the array
		s := fmt.Sprintf("for i, v := range self.%s {\n", name)
		off := fmt.Sprintf("%d + i * %d", f.ByteOffset, f.BitSize/8)
		s += f.payloadPackPrimitive(off, "v") + "\n"
		s += fmt.Sprintf("}")
		return s
	}

	// pack a single field
	return f.payloadPackPrimitive(fmt.Sprintf("%d", f.ByteOffset), "self."+name)
}

func (f *MessageField) payloadUnpackPrimitive(offset string) string {

	if f.BitSize == 8 {
		return fmt.Sprintf("%s(p.Payload[%s])", goArrayType(f.GoType), offset)
	}

	if f.IsFloat() {
		switch f.BitSize {
		case 32, 64:
			return fmt.Sprintf("math.Float%dfrombits(binary.LittleEndian.Uint%d(p.Payload[%s:]))", f.BitSize, f.BitSize, offset)
		}
	} else {
		switch f.BitSize {
		case 16, 32, 64:
			return fmt.Sprintf("%s(binary.LittleEndian.Uint%d(p.Payload[%s:]))", goArrayType(f.GoType), f.BitSize, offset)
		}
	}

	panic("unhandled bitsize")
}

func (f *MessageField) PayloadUnpackSequence() string {
	name := UpperCamelCase(f.Name)

	if f.ArrayLen > 0 {
		// optimize to copy() if possible
		if strings.HasSuffix(f.GoType, "byte") || strings.HasSuffix(f.GoType, "uint8") {
			return fmt.Sprintf("copy(self.%s[:], p.Payload[%d:%d])", name, f.ByteOffset, f.ByteOffset+f.ArrayLen)
		}

		// unpack each element in the array
		s := fmt.Sprintf("for i := 0; i < len(self.%s); i++ {\n", name)
		off := fmt.Sprintf("%d + i * %d", f.ByteOffset, f.BitSize/8)
		s += fmt.Sprintf("self.%s[i] = %s\n", name, f.payloadUnpackPrimitive(off))
		s += fmt.Sprintf("}")
		return s
	}

	return fmt.Sprintf("self.%s = %s", name, f.payloadUnpackPrimitive(fmt.Sprintf("%d", f.ByteOffset)))
}

func SanitizeComments(s string) string {
	return strings.Replace(s, "\n", "\n// ", -1)
}

// read in an xml-based dialect file,
// and populate a Dialect struct with its contents
func ParseDialect(in io.Reader, name string) (*Dialect, error) {

	filebytes, err := ioutil.ReadAll(in)
	if err != nil {
		return nil, err
	}

	dialect := &Dialect{
		Name: name,
	}

	if err := xml.Unmarshal(filebytes, &dialect); err != nil {
		return nil, err
	}

	// filter out messages with MSG_ID > 256. these are from
	// mavlink v2 and do not fit in uint8
	filteredMessages := make([]*Message, len(dialect.Messages))
	n := 0
	for _, msg := range dialect.Messages {
		if msg.ID <= 0xff {
			filteredMessages[n] = msg
			n += 1
		}
	}
	dialect.Messages = filteredMessages[:n]

	if dialect.Include != "" {
		// generate(protocol.IncludeName())
	}

	return dialect, nil
}

// convert a C primitive type to its corresponding Go type.
// do not handle arrays or other constructs...just primitives.
func c2goPrimitive(ctype string) string {
	switch ctype {
	case "uint8_t", "uint16_t", "uint32_t", "uint64_t",
		"int8_t", "int16_t", "int32_t", "int64_t":
		idx := strings.IndexByte(ctype, '_')
		return ctype[:idx]
	case "char":
		return "byte"
	case "float":
		return "float32"
	case "double":
		return "float64"
	case "uint8_t_mavlink_version":
		return "uint8"
	default:
		panic(fmt.Sprintf("c2goPrimitive: unhandled primitive type - %s", ctype))
	}
}

func goArrayType(s string) string {
	idx := strings.IndexByte(s, ']')
	if idx < 0 {
		return s
	}
	return s[idx+1:]
}

func (f *MessageField) IsFloat() bool {
	return strings.HasPrefix(goArrayType(f.GoType), "float")
}

func GoTypeInfo(s string) (string, int, int, error) {

	var name string
	var bitsz, arraylen int
	var err error

	// array? leave the [N] but convert the primitive type name
	if idx := strings.IndexByte(s, '['); idx < 0 {
		name = c2goPrimitive(s)
	} else {
		name = s[idx:] + c2goPrimitive(s[:idx])
		if arraylen, err = strconv.Atoi(s[idx+1 : len(s)-1]); err != nil {
			return "", 0, 0, err
		}
	}

	// determine bit size for this type
	if strings.HasSuffix(name, "byte") {
		bitsz = 8
	} else {
		t := name[strings.IndexByte(name, ']')+1:]
		if sizeStart := strings.IndexAny(t, "8136"); sizeStart != -1 {
			if bitsz, err = strconv.Atoi(t[sizeStart:]); err != nil {
				return "", 0, 0, err
			}
		} else {
			return "", 0, 0, errors.New("Unknown message field size")
		}
	}

	return name, bitsz, arraylen, nil
}

// generate a .go source file from the given dialect
func (d *Dialect) GenerateGo(w io.Writer) error {
	// templatize to buffer, format it, then write out

	var bb bytes.Buffer

	bb.WriteString("package mavlink\n\n")

	bb.WriteString("import (\n")
	bb.WriteString("\"encoding/binary\"\n")
	bb.WriteString("\"fmt\"\n")
	bb.WriteString("\"math\"\n")
	bb.WriteString(")\n")

	bb.WriteString("//////////////////////////////////////////////////\n")
	bb.WriteString("//\n")
	bb.WriteString("// NOTE: do not edit,\n")
	bb.WriteString("// this file created automatically by mavgen.go\n")
	bb.WriteString("//\n")
	bb.WriteString("//////////////////////////////////////////////////\n\n")

	err := d.generateEnums(&bb)
	d.generateClasses(&bb)
	d.generateMsgIds(&bb)

	dofmt := true
	formatted := bb.Bytes()

	if dofmt {
		formatted, err = format.Source(bb.Bytes())
		if err != nil {
			return err
		}
	}

	n, err := w.Write(formatted)
	if err == nil && n != len(formatted) {
		return io.ErrShortWrite
	}

	return err
}

func (d *Dialect) generateEnums(w io.Writer) error {
	enumTmpl := `
{{range .Enums}}
// {{.Name}}: {{.Description}}
const ({{range .Entries}}
	{{.Name}} = {{.Value}} // {{.Description}}{{end}}
)
{{end}}
`
	// fill in missing enum values if necessary, and ensure description strings are valid.
	for _, e := range d.Enums {
		e.Description = strings.Replace(e.Description, "\n", " ", -1)
		e.Name = UpperCamelCase(e.Name)
		for i, ee := range e.Entries {
			if ee.Value == 0 {
				ee.Value = uint8(i)
			}
			ee.Description = strings.Replace(ee.Description, "\n", " ", -1)
		}
	}

	return template.Must(template.New("enums").Parse(enumTmpl)).Execute(w, d)
}

func (d *Dialect) generateMsgIds(w io.Writer) error {
	msgIdTmpl := `
// Message IDs
const ({{range .Messages}}
	MSG_ID_{{.Name}} = {{.ID}}{{end}}
)

// Dialect{{.Name | UpperCamelCase}} is the dialect represented by {{.Name}}.xml
var Dialect{{.Name | UpperCamelCase}} *Dialect = &Dialect{
	Name: "{{.Name}}",
	crcExtras: map[uint8]uint8{ {{range .Messages}}
		{{.ID}}: {{.CRCExtra}}, // MSG_ID_{{.Name}}{{end}}
	},
}
`
	return template.Must(template.New("msgIds").Funcs(funcMap).Parse(msgIdTmpl)).Execute(w, d)
}

// generate class definitions for each msg id.
// for now, pack/unpack payloads via encoding/binary since it
// is expedient and correct. optimize this if/when needed.
func (d *Dialect) generateClasses(w io.Writer) error {

	classesTmpl := `
{{range .Messages}}
{{$name := .Name | UpperCamelCase}}
// {{.Description}}
type {{$name}} struct { {{range .Fields}}
  {{.Name | UpperCamelCase}} {{.GoType}} // {{.Description}}{{end}}
}

func (self *{{$name}}) MsgID() uint8 {
	return {{.ID}}
}

func (self *{{$name}}) MsgName() string {
	return "{{.Name | UpperCamelCase}}"
}

func (self *{{$name}}) Pack(p *Packet) error {
	payload := make([]byte, {{ .Size }}){{range .Fields}}
	{{.PayloadPackSequence}}{{end}}

	p.MsgID = self.MsgID()
	p.Payload = payload
	return nil
}

func (self *{{$name}}) Unpack(p *Packet) error {
	if len(p.Payload) < {{ .Size }} {
		return fmt.Errorf("payload too small")
	}{{range .Fields}}
	{{.PayloadUnpackSequence}}{{end}}
	return nil
}
{{end}}
`
	for _, m := range d.Messages {
		m.Description = strings.Replace(m.Description, "\n", "\n// ", -1)

		for _, f := range m.Fields {
			f.Description = strings.Replace(f.Description, "\n", " ", -1)
			goname, gosz, golen, err := GoTypeInfo(f.CType)
			if err != nil {
				return err
			}
			f.GoType, f.BitSize, f.ArrayLen = goname, gosz, golen
		}

		// ensure fields are sorted according to their size,
		// http://www.mavlink.org/mavlink/crc_extra_calculation
		sort.Stable(sort.Reverse(m))

		// once sorted, calculate offsets for use in payload packing/unpacking
		offset := 0
		for _, f := range m.Fields {
			f.ByteOffset = offset
			offset += f.SizeInBytes()
		}
	}

	return template.Must(template.New("classesTmpl").Funcs(funcMap).Parse(classesTmpl)).Execute(w, d)
}
