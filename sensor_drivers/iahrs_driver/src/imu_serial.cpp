#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>

// 시리얼 포트와 통신 속도 설정
#define SERIAL_PORT			"/dev/ttyUSB1"
#define SERIAL_SPEED		B115200

#define TRACE	printf

int serial_fd = -1;

int serial_open ()
{
	TRACE ("Try to open serial: %s\n", SERIAL_PORT); 

	serial_fd = open(SERIAL_PORT, O_RDWR|O_NOCTTY); // |O_NDELAY);
	if (serial_fd < 0) {
		printf ("Error unable to open %s\n", SERIAL_PORT);
		return -1;
	}
  	TRACE ("%s open success\n", SERIAL_PORT);

  	struct termios tio;
  	tcgetattr(serial_fd, &tio);
  	cfmakeraw(&tio);
	tio.c_cflag = CS8|CLOCAL|CREAD;
  	tio.c_iflag &= ~(IXON | IXOFF);
  	cfsetspeed(&tio, SERIAL_SPEED);
  	tio.c_cc[VTIME] = 0;
  	tio.c_cc[VMIN] = 0;

  	int err = tcsetattr(serial_fd, TCSAFLUSH, &tio);
  	if (err != 0) {
    	TRACE ("Error tcsetattr() function return error\n");
    	close(serial_fd);
		serial_fd = -1;
    	return -1;
  	}
	return 0;
}

int SendRecv(const char* command, double* returned_data, int data_length);

int main()
{
	serial_open();

	if (serial_fd >= 0) {
		// 명령에 따라 읽어오는 데이터 갯수가 달라진다.
		// 여기서는 최대 10개의 데이터를 읽도록 메모리를 할당한다.
        const int max_data = 10;
        double data[max_data];
        int no_data;

        for (int i = 0; i < 100; i++) {
            // 1) Euler angle 읽기 (roll, pitch, yaw)
            no_data = SendRecv("e\n", data, max_data);
            if (no_data >= 3) {
                printf("Euler angle = %f, %f, %f\n",
                    data[0], data[1], data[2]);
            }

            // 2) Quaternion 읽기 (r, v0, v1, v2)
            no_data = SendRecv("q\n", data, max_data);
            if (no_data >= 4) {
                printf("Quaternion = %f, %f, %f, %f\n",
                    data[0], data[1], data[2], data[3]);
            }

            // 3) 가속도 읽기 (ag → ax, ay, az)
            no_data = SendRecv("ag\n", data, max_data);
            if (no_data >= 3) {
                printf("Acceleration (ag) = %f, %f, %f (m/s^2)\n",
                    data[0], data[1], data[2]);
            }

            // 4) 속도 읽기 (v → vx, vy, vz)
            no_data = SendRecv("v\n", data, max_data);
            if (no_data >= 3) {
                printf("Velocity (v) = %f, %f, %f (m/s)\n",
                    data[0], data[1], data[2]);
            }

            // 5) 위치 읽기 (p → px, py, pz)
            no_data = SendRecv("p\n", data, max_data);
            if (no_data >= 3) {
                printf("Position (p) = %f, %f, %f (m)\n",
                    data[0], data[1], data[2]);
            }

            // 100ms 대기
            usleep(10 * 1000);
        }


		close (serial_fd);
	}
	return 0;
}

static unsigned long GetTickCount() 
{
	// ms 단위의 시간을 가져온다.
    struct timespec ts;
   
    clock_gettime (CLOCK_MONOTONIC, &ts);

    return ts.tv_sec*1000 + ts.tv_nsec/1000000;
}

int SendRecv(const char* command, double* returned_data, int data_length)
{
	// 응답을 기다리는 최대 시간 설정. 단위: ms
	#define COMM_RECV_TIMEOUT	30	

	// 시리얼의 rx 버퍼에 이미 들어온 문자열이 있다면, 명령을 전송하기 전에 읽어서 rx 버퍼를 비운다.
	char temp_buff[256];
	read (serial_fd, temp_buff, 256);

	int command_len = strlen(command);
	int n = write(serial_fd, command, command_len);
	if (n < 0) return -1;

	// 명령에 대한 응답 문자열을 저장하기 위한 버퍼 할당
	const int buff_size = 1024;
	int  recv_len = 0;
	char recv_buff[buff_size + 1];	// 마지막 EOS를 추가하기 위해 + 1

	unsigned long time_start = GetTickCount();

	while (recv_len < buff_size) {
		int n = read (serial_fd, recv_buff + recv_len, buff_size - recv_len);
		if (n < 0) {
			// 통신 도중 오류 발생
			return -1;
		}
		else if (n == 0) {
			// 아무런 데이터도 받지 못했다. 1ms 기다려본다.
			usleep(1000);
		}
		else if (n > 0) {
			recv_len += n;

			// 수신 문자열 끝에 \r 또는 \n이 들어왔는지 체크
			if (recv_buff[recv_len - 1] == '\r' || recv_buff[recv_len - 1] == '\n') {
				break;
			}
		}

		unsigned long time_current = GetTickCount();
		unsigned long time_delta = time_current - time_start;

		if (time_delta >= COMM_RECV_TIMEOUT) break;
	}
	recv_buff[recv_len] = '\0';

	// 에러가 리턴 되었는지 체크한다.
	if (recv_len > 0) {
		if (recv_buff[0] == '!') {
			return -1;
		}
	}

	// 보낸 명령과 돌아온 변수명이 같은지 비교한다.
	if (strncmp(command, recv_buff, command_len - 1) == 0) {
		if (recv_buff[command_len - 1] == '=') {
			int data_count = 0;

			char* p = &recv_buff[command_len];
			char* pp = NULL;

			for (int i = 0; i < data_length; i++) {
				if (p[0] == '0' && p[1] == 'x') {	// 16 진수
					returned_data[i] = strtol(p+2, &pp, 16);
					data_count++;
				}
				else {
					returned_data[i] = strtod(p, &pp);
					data_count++;
				}

				if (*pp == ',') {
					p = pp + 1;
				}
				else {
					break;
				}
			}
			return data_count;
		}
	}
	return 0;
}