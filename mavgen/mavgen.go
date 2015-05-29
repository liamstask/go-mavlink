package main

import (
	"bytes"
	"encoding/xml"
	"fmt"
	"go/format"
	"io"
	"io/ioutil"
	"strings"
	"text/template"
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
	ID          uint8           `xml:"id,attr"`
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
	ArrayLength int
	ByteOffset  int
	PackOp      string
}

func (msg *Message) Size() (size int) {
	for _, f := range msg.Fields {
		bitSize := f.BitSize
		if f.ArrayLength > 0 {
			bitSize *= f.ArrayLength
		}
		size += bitSize
	}
	return size / 8
}

func (msg *Message) CRCExtra() uint8 {
	// hash := x25.NewHash()

	// fmt.Fprint(hash, msg.Name+" ")
	for _, f := range msg.Fields {
		cType := f.CType
		if cType == "uint8_t_mavlink_version" {
			cType = "uint8_t"
		}
		// fmt.Fprint(hash, cType+" "+f.Name+" ")
		if f.ArrayLength > 0 {
			// hash.WriteByte(byte(f.ArrayLength))
		}
	}

	return 0
	// return uint8((hash.Sum & 0xFF) ^ (hash.Sum >> 8))
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

func SanitizeComments(s string) string {
	return strings.Replace(s, "\n", "\n// ", -1)
}

// read in an xml-based dialect file,
// and populate a Dialect struct with its contents
func ParseDialect(in io.Reader) (*Dialect, error) {

	filebytes, err := ioutil.ReadAll(in)
	if err != nil {
		return nil, err
	}

	dialect := &Dialect{}

	if err := xml.Unmarshal(filebytes, &dialect); err != nil {
		return nil, err
	}

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

// return the corresponding go type for the given c type
func c2goType(c string) string {

	// array? leave the [N] but convert the primitive type name
	if idx := strings.IndexByte(c, '['); idx != -1 {
		return c[idx:] + c2goPrimitive(c[:idx])
	}

	return c2goPrimitive(c)
}

// generate a .go source file from the given dialect
func (d *Dialect) GenerateGo(w io.Writer) error {
	// templatize to buffer, format it, then write out

	var bb bytes.Buffer

	bb.WriteString("package mavlink\n\n")

	bb.WriteString("import (\n")
	bb.WriteString("\"bytes\"\n")
	bb.WriteString("\"encoding/binary\"\n")
	bb.WriteString(")\n")

	bb.WriteString("//////////////////////////////////////////////////\n")
	bb.WriteString("//\n")
	bb.WriteString("// NOTE: do not edit,\n")
	bb.WriteString("// this file created automatically by mavgen.go\n")
	bb.WriteString("//\n")
	bb.WriteString("//////////////////////////////////////////////////\n\n")

	err := d.generateEnums(&bb)
	d.generateMsgIds(&bb)
	d.generateClasses(&bb)

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
`
	return template.Must(template.New("msgIds").Parse(msgIdTmpl)).Execute(w, d)
}

// generate class definitions for each msg id.
// for now, pack/unpack payloads via encoding/binary since it
// is expedient and correct. optimize this if/when needed.
func (d *Dialect) generateClasses(w io.Writer) error {

	classesTmpl := `
{{range .Messages}}
{{$name := .Name }}
// {{.Description}}
type {{$name}} struct { {{range .Fields}}
  {{.Name }} {{.GoType}} // {{.Description}}{{end}}
}

func (self *{{$name}}) MsgID() uint8 {
	return {{.ID}}
}

func (self *{{$name}}) MsgName() string {
	return "{{.Name}}"
}

func (self *{{$name}}) Pack(p *Packet) error {
	var buf bytes.Buffer
	for _, f := range []interface{} { {{range .Fields}}
		&self.{{.Name}},{{end}}
	} {
		if err := binary.Write(&buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}

	p.MsgID = self.MsgID()
	p.Payload = buf.Bytes()
	return nil
}

func (self *{{$name}}) Unpack(p *Packet) error {
	buf := bytes.NewBuffer(p.Payload)
	for _, f := range []interface{} { {{range .Fields}}
		&self.{{.Name}},{{end}}
	} {
		if err := binary.Read(buf, binary.LittleEndian, f); err != nil {
			return err
		}
	}
	return nil
}
{{end}}
`
	for _, m := range d.Messages {
		m.Description = strings.Replace(m.Description, "\n", "\n// ", -1)
		m.Name = UpperCamelCase(m.Name)

		for _, f := range m.Fields {
			f.Name = UpperCamelCase(f.Name)
			f.Description = strings.Replace(f.Description, "\n", " ", -1)
			f.GoType = c2goType(f.CType)
		}

		// sort.Stable(sort.Reverse(m))
	}

	return template.Must(template.New("classesTmpl").Parse(classesTmpl)).Execute(w, d)
}
