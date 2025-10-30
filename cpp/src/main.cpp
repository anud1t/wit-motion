#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>
#include <vector>

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
	uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i) {
		crc ^= data[i];
		for (int j = 0; j < 8; ++j) {
			if (crc & 1) crc = (crc >> 1) ^ 0xA001; else crc >>= 1;
		}
	}
	return crc;
}

static int set_interface_attribs(int fd, int speed) {
	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) return -1;
	cfmakeraw(&tty);
	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);
	tty.c_cflag |= (CLOCAL | CREAD);
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 3; // 300ms read timeout
	if (tcsetattr(fd, TCSANOW, &tty) != 0) return -1;
	return 0;
}

static speed_t baud_to_flag(int baud) {
	switch (baud) {
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
		default: return B115200;
	}
}

int main(int argc, char** argv) {
	std::string port = "/dev/ttyCH341USB0";
	int baud = 115200;
	int addr = 0x50;
	for (int i = 1; i < argc; ++i) {
		std::string a = argv[i];
		if (a == "--port" && i + 1 < argc) port = argv[++i];
		else if (a == "--baud" && i + 1 < argc) baud = std::atoi(argv[++i]);
		else if (a == "--addr" && i + 1 < argc) addr = std::strtol(argv[++i], nullptr, 0);
	}

	int fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) { std::perror("open"); return 1; }
	if (set_interface_attribs(fd, baud_to_flag(baud)) != 0) { std::perror("tcsetattr"); close(fd); return 1; }

	uint8_t req[8];
	req[0] = static_cast<uint8_t>(addr & 0xFF);
	req[1] = 0x03; // read holding regs
	req[2] = 0x00; // reg hi
	req[3] = 0x34; // reg lo
	req[4] = 0x00; // count hi
	req[5] = 0x0C; // count lo (12 regs => 24 bytes)
	uint16_t crc = crc16_modbus(req, 6);
	req[6] = crc & 0xFF; // CRC low
	req[7] = (crc >> 8) & 0xFF; // CRC high

	uint8_t hdr[3];
	uint8_t payload[24];
	uint8_t rcrc[2];

	while (true) {
		// write request
		ssize_t w = write(fd, req, sizeof(req));
		if (w < 0) { std::perror("write"); break; }
		usleep(20000);

		int n = read(fd, hdr, 3);
		if (n != 3) { usleep(200000); continue; }
		if (hdr[0] != (addr & 0xFF) || hdr[1] != 0x03 || hdr[2] != 24) { tcflush(fd, TCIFLUSH); usleep(200000); continue; }
		int m = read(fd, payload, 24);
		if (m != 24) { usleep(200000); continue; }
		int k = read(fd, rcrc, 2);
		if (k != 2) { usleep(200000); continue; }
		uint8_t frame_no_crc[27];
		std::memcpy(frame_no_crc, hdr, 3);
		std::memcpy(frame_no_crc + 3, payload, 24);
		uint16_t c = crc16_modbus(frame_no_crc, 27);
		uint16_t rc = rcrc[0] | (uint16_t(rcrc[1]) << 8);
		if (c != rc) { usleep(200000); continue; }

		// parse big-endian 12x int16
		int16_t vals[12];
		for (int i = 0; i < 12; ++i) {
			vals[i] = (int16_t)((payload[2*i] << 8) | payload[2*i + 1]);
		}
		auto f32 = [](int16_t v){ return double(v); };
		double acc[3] = { f32(vals[0]) / 32768.0 * 16.0, f32(vals[1]) / 32768.0 * 16.0, f32(vals[2]) / 32768.0 * 16.0 };
		double gyr[3] = { f32(vals[3]) / 32768.0 * 2000.0, f32(vals[4]) / 32768.0 * 2000.0, f32(vals[5]) / 32768.0 * 2000.0 };
		double mag[3] = { f32(vals[6]) * 13.0 / 1000.0, f32(vals[7]) * 13.0 / 1000.0, f32(vals[8]) * 13.0 / 1000.0 };
		double ang[3] = { f32(vals[9]) / 32768.0 * 180.0, f32(vals[10]) / 32768.0 * 180.0, f32(vals[11]) / 32768.0 * 180.0 };

		// Temperature (0x0040, 1 reg)
		uint8_t treq[8] = { (uint8_t)(addr & 0xFF), 0x03, 0x00, 0x40, 0x00, 0x01, 0, 0 };
		uint16_t tcrc = crc16_modbus(treq, 6);
		treq[6] = tcrc & 0xFF; treq[7] = (tcrc >> 8) & 0xFF;
		// Clear any residual bytes then request
		tcflush(fd, TCIFLUSH);
		usleep(2000);
		ssize_t tw = write(fd, treq, 8);
		if (tw != 8) { usleep(20000); continue; }
		usleep(20000);
		uint8_t thdr[3]; uint8_t tp[2]; uint8_t tcr[2]; double tempC = 0; bool haveT=false;
		if (read(fd, thdr, 3) == 3 && thdr[1] == 0x03 && thdr[2] == 2 && read(fd, tp, 2) == 2 && read(fd, tcr, 2) == 2) {
			int16_t tv = (int16_t)((tp[0] << 8) | tp[1]);
			tempC = ((double)tv) / 100.0; haveT=true;
		}

		// Quaternion (0x0051..0x0054, 4 regs)
		uint8_t qreq[8] = { (uint8_t)(addr & 0xFF), 0x03, 0x00, 0x51, 0x00, 0x04, 0, 0 };
		uint16_t qcrc = crc16_modbus(qreq, 6);
		qreq[6] = qcrc & 0xFF; qreq[7] = (qcrc >> 8) & 0xFF;
		tcflush(fd, TCIFLUSH);
		usleep(2000);
		ssize_t qw = write(fd, qreq, 8);
		if (qw != 8) { usleep(20000); continue; }
		usleep(20000);
		uint8_t qhdr[3]; uint8_t qp[8]; uint8_t qcr[2]; double q[4]; bool haveQ=false;
		if (read(fd, qhdr, 3) == 3 && qhdr[1] == 0x03 && qhdr[2] == 8 && read(fd, qp, 8) == 8 && read(fd, qcr, 2) == 2) {
			for (int i=0;i<4;++i) {
				int16_t v = (int16_t)((qp[2*i] << 8) | qp[2*i+1]);
				q[i] = ((double)v) / 32768.0;
			}
			haveQ=true;
		}

		// Print in a stable format, include placeholders when missing
		if (haveT && haveQ) {
			std::printf("acc:(%6.3f,%6.3f,%6.3f) gyr:(%7.3f,%7.3f,%7.3f) ang:(%6.3f,%6.3f,%6.3f) mag:(%7.3f,%7.3f,%7.3f) T:%5.2fC q:(%.4f,%.4f,%.4f,%.4f)\n",
				acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], ang[0], ang[1], ang[2], mag[0], mag[1], mag[2], tempC, q[0], q[1], q[2], q[3]);
		} else if (haveT) {
			std::printf("acc:(%6.3f,%6.3f,%6.3f) gyr:(%7.3f,%7.3f,%7.3f) ang:(%6.3f,%6.3f,%6.3f) mag:(%7.3f,%7.3f,%7.3f) T:%5.2fC\n",
				acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], ang[0], ang[1], ang[2], mag[0], mag[1], mag[2], tempC);
		} else if (haveQ) {
			std::printf("acc:(%6.3f,%6.3f,%6.3f) gyr:(%7.3f,%7.3f,%7.3f) ang:(%6.3f,%6.3f,%6.3f) mag:(%7.3f,%7.3f,%7.3f) q:(%.4f,%.4f,%.4f,%.4f)\n",
				acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], ang[0], ang[1], ang[2], mag[0], mag[1], mag[2], q[0], q[1], q[2], q[3]);
		} else {
			std::printf("acc:(%6.3f,%6.3f,%6.3f) gyr:(%7.3f,%7.3f,%7.3f) ang:(%6.3f,%6.3f,%6.3f) mag:(%7.3f,%7.3f,%7.3f)\n",
				acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2], ang[0], ang[1], ang[2], mag[0], mag[1], mag[2]);
		}
		usleep(200000);
	}

	close(fd);
	return 0;
}


