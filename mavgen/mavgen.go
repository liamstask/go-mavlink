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

	"github.com/liamstask/go-mavlink/x25"
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
	ArrayLen    int
}

var funcMap = template.FuncMap{
	"UpperCamelCase": UpperCamelCase,
}

func (m *Message) Size() (size int) {
	for _, f := range m.Fields {
		bitSize := f.BitSize
		if f.ArrayLen > 0 {
			bitSize *= f.ArrayLen
		}
		size += bitSize
	}
	return size / 8
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
		fmt.Fprint(hash, cType+" "+f.Name+" ")
		if f.ArrayLen > 0 {
			hash.Write([]byte{byte(f.ArrayLen)})
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

// CRC Extra, indexed by msg id
// http://www.mavlink.org/mavlink/crc_extra_calculation
var crcExtras = map[uint8]uint8{ {{range .Messages}}
	{{.ID}}: {{.CRCExtra}}, // MSG_ID_{{.Name}}{{end}}
}
`
	return template.Must(template.New("msgIds").Parse(msgIdTmpl)).Execute(w, d)
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
	var buf bytes.Buffer
	for _, f := range []interface{} { {{range .Fields}}
		&self.{{.Name | UpperCamelCase}},{{end}}
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
		&self.{{.Name | UpperCamelCase}},{{end}}
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
	}

	return template.Must(template.New("classesTmpl").Funcs(funcMap).Parse(classesTmpl)).Execute(w, d)
}
