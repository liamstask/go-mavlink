package main

import (
	"testing"
)

func TestTypeConversions(t *testing.T) {

	cases := []struct{ in, want string }{
		{"char", "byte"},
		{"uint8_t", "uint8"},
		{"uint16_t", "uint16"},
		{"uint32_t", "uint32"},
		{"uint64_t", "uint64"},
		{"float", "float32"},
		{"double", "float64"},
		{"char[10]", "[10]byte"},
		{"float[30]", "[30]float32"},
	}

	for _, c := range cases {
		got := c2goType(c.in)
		if got != c.want {
			t.Errorf("Type Conversion for %q, got %q, want %q", c.in, got, c.want)
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
