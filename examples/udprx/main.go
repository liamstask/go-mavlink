package main

import (
	"flag"
	"log"
	"net"

	"github.com/cnord/go-mavlink/mavlink"
)

//////////////////////////////////////
//
// mavlink udp rx example
//
// listen to ardupilot SITL and prints received msgs, more info:
// http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/
//
// run via `go run main.go`
//
//////////////////////////////////////

var rxaddr = flag.String("addr", ":14550", "address to listen on")

func main() {

	flag.Parse()

	listenAndServe(*rxaddr)
}

func listenAndServe(addr string) {

	udpAddr, err := net.ResolveUDPAddr("udp", addr)
	if err != nil {
		log.Fatal(err)
	}

	conn, listenerr := net.ListenUDP("udp", udpAddr)
	if listenerr != nil {
		log.Fatal(listenerr)
	}

	log.Println("listening on", udpAddr)

	dec := mavlink.NewDecoder(conn)
	dec.Dialects.Add(mavlink.DialectArdupilotmega)

	for {
		pkt, err := dec.Decode()
		if err != nil {
			log.Println("Decode fail:", err)
			if pkt != nil {
				log.Println(*pkt)
			}
			continue
		}

		log.Println("msg rx:", pkt.MsgID, pkt.Payload)
	}
}
