package main

import (
	"testing"
)

func TestTypeConversions(t *testing.T) {

	cases := []struct {
		in              string
		name            string
		bitsz, arraylen int
	}{
		{"char", "byte", 8, 0},
		{"uint8_t", "uint8", 8, 0},
		{"uint16_t", "uint16", 16, 0},
		{"uint32_t", "uint32", 32, 0},
		{"uint64_t", "uint64", 64, 0},
		{"float", "float32", 32, 0},
		{"double", "float64", 64, 0},
		{"char[10]", "[10]byte", 8, 10},
		{"float[30]", "[30]float32", 32, 30},
	}

	for _, c := range cases {
		name, bitsz, arraylen, err := GoTypeInfo(c.in)
		// XXX: should test some cases that generate errors...
		if err != nil {
			t.Error("Type conversion err:", err)
		}
		if name != c.name {
			t.Errorf("Type Conversion for %q, got name %q, want %q", c.in, name, c.name)
		}
		if bitsz != c.bitsz {
			t.Errorf("Type Conversion for %q, got bitsz %q, want %q", c.in, bitsz, c.bitsz)
		}
		if arraylen != c.arraylen {
			t.Errorf("Type Conversion for %q, got arraylen %q, want %q", c.in, arraylen, c.arraylen)
		}
	}
}

func TestNameConversion(t *testing.T) {

	cases := []struct{ in, want string }{
		{"test", "Test"},
		{"_test_", "Test"},
		{"_test", "Test"},
		{"test_", "Test"},
		{"test_thing", "TestThing"},
		{"test_thing_", "TestThing"},
		{"TEST_THING", "TestThing"},
		{"_TEST_", "Test"},
		{"_TEST___THiNG__", "TestThing"},
		{"_TEST___THiNG_A__", "TestThingA"},
	}

	for _, c := range cases {
		got := UpperCamelCase(c.in)
		if got != c.want {
			t.Errorf("Upper Camel Conversion for %q, got %q, want %q", c.in, got, c.want)
		}
	}
}
