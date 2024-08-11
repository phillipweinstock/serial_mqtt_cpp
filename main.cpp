#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <mqtt/async_client.h>

const std::string SERVER_ADDRESS("tcp://test.mosquitto.org:1883");
const std::string CLIENT_ID("lora_mqtt_cpp_client");
const std::string TOPIC("loraserial/");
mqtt::async_client cli(SERVER_ADDRESS,CLIENT_ID);



using namespace std;
int OpenSerial(const char* port)
{
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0)
    {
        std::cerr << "Port Error: " << strerror(errno) << std::endl;
        return -1;
    }
    struct termios tty;
    if(tcgetattr(fd,&tty) != 0)
    {
        std::cerr << "Attribute Error: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }
    cfsetospeed(&tty,B115200);
    cfsetispeed(&tty,B115200);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tcsetattr(fd,TCSANOW,&tty);
    return fd;




}
void PublishMqtt(const std::string& msg)
{
    mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC,msg);
    pubmsg->set_qos(0);
    cli.publish(pubmsg);
}
void ReadPub(int fd)
{
    static std::string buff;
    char buf[256];
    ssize_t n = read(fd,buf,sizeof(buf) -1);
    if(n>0)
    {
        buf[n] = '\0';
        std::string msg(buf);
        msg.erase(0,msg.find_first_not_of(" \t\r\n"));
        msg.erase(msg.find_last_not_of(" \t\r\n") + 1);
        if(!msg.empty())
        {
            std::cout << msg << std::endl;
            PublishMqtt(msg);
        }

    }

}

int main()
{

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(true);
    mqtt::connect_options connectionOptions = connOpts;
    cli.connect(connectionOptions)->wait();
    const char* port = "/dev/ttyACM0";
    int fd = OpenSerial(port);
    while(true)
    {
        ReadPub(fd);
        usleep(1000);
    }
    return 0;
}
