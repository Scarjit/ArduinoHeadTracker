package main

import (
	"encoding/binary"
	"encoding/json"
	"github.com/tarm/serial"
	"log"
	"math"
	"net"
	"strings"
)

func main() {
	go reader()
	select {}
}

func reader() {
	c := &serial.Config{Name: "COM3", Baud: 115200}
	s, err := serial.OpenPort(c)
	if err != nil {
		log.Fatal(err)
	}

	conn, err := net.Dial("udp", "127.0.0.1:4242")
	if err != nil {
		log.Fatal(err)
	}

	var xBuf string

	var x, y, z float64

	for {
		buf := make([]byte, 128)
		var n int
		n, err = s.Read(buf)
		if err != nil {
			log.Fatal(err)
		}
		xBuf = xBuf + string(buf[:n])
		// Get position of \r\n in xBuf
		pos := strings.Index(xBuf, "\r\n")
		if pos == -1 {
			continue
		}
		// Get the line
		line := xBuf[:pos]
		// Remove the line from xBuf
		xBuf = xBuf[pos+2:]
		
		// Try unmarshalling it as JSON
		var tl TopLevel
		err = json.Unmarshal([]byte(line), &tl)
		if err != nil {
			continue
		}

		if tl.Status != "ready" {
			continue
		}

		angleX := Float64bytes(tl.Data["angleX"])
		angleY := Float64bytes(tl.Data["angleY"])
		angleZ := Float64bytes(tl.Data["angleZ"])

		// print angle data
		log.Printf("angleX: %f, angleY: %f, angleZ: %f", tl.Data["angleX"], tl.Data["angleY"], tl.Data["angleZ"])

		// concat the bytes
		b := make([]byte, 0, 24)
		b = append(b, Float64bytes(x)...)
		b = append(b, Float64bytes(y)...)
		b = append(b, Float64bytes(z)...)
		b = append(b, angleZ...)
		b = append(b, angleY...)
		b = append(b, angleX...)

		// Send to UDP
		_, err = conn.Write(b)
		if err != nil {
			log.Fatal(err)
		}
	}
}

type TopLevel struct {
	Data   map[string]float64 `json:"data"`
	Status string             `json:"status"`
}

func Float64bytes(float float64) []byte {
	bits := math.Float64bits(float)
	bytes := make([]byte, 8)
	binary.LittleEndian.PutUint64(bytes, bits)
	return bytes
}
