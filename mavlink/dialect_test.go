package mavlink

import (
	"testing"
)

func TestAddRemove(t *testing.T) {

	ds := DialectSlice{DialectCommon}

	// verify initial state
	if len(ds) != 1 {
		t.Error("bad len after remove")
	}
	if ds.IndexOf(DialectCommon) != 0 {
		t.Error("couldn't find dialect")
	}

	// verify addition
	ds.Add(DialectArdupilotmega)
	if len(ds) != 2 {
		t.Error("bad len after add")
	}
	if ds.IndexOf(DialectArdupilotmega) != 1 {
		t.Error("couldn't find dialect")
	}

	// verify removal
	ds.Remove(DialectCommon)
	if len(ds) != 1 {
		t.Error("bad len after remove")
	}
	if ds.IndexOf(DialectCommon) >= 0 {
		t.Error("wrong dialect")
	}
	if ds.IndexOf(DialectArdupilotmega) != 0 {
		t.Error("wrong dialect")
	}
}
