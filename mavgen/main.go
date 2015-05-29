package main

import (
	"flag"
	"log"
	"os"
	"path/filepath"
	"strings"
)

var (
	infile  = flag.String("f", "", "mavlink definition file input")
	outfile = flag.String("o", "", "output file name; default input.go")
)

func main() {

	log.SetFlags(0)
	log.SetPrefix("mavgen: ")
	flag.Parse()

	fin, err := os.Open(*infile)
	if err != nil {
		log.Fatal("couldn't open input: ", err)
	}
	defer fin.Close()

	d, err := ParseDialect(fin)
	if err != nil {
		log.Fatal("parse fail: ", err)
	}

	fout, err := os.Create(findOutFile())
	if err != nil {
		log.Fatal("couldn't open output: ", err)
	}
	defer fout.Close()

	if err := d.GenerateGo(fout); err != nil {
		log.Fatal("couldn't write to output: ", err)
	}
}

func findOutFile() string {
	if *outfile == "" {
		base := strings.TrimSuffix(filepath.Base(*infile), ".xml")
		*outfile = strings.ToLower(base) + ".go"
	}

	dir, err := os.Getwd()
	if err != nil {
		log.Fatal("Getwd(): ", err)
	}

	return filepath.Join(dir, strings.ToLower(*outfile))
}
