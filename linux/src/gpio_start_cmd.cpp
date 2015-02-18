#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <errno.h>
#include <inttypes.h>
#include <sys/time.h>
#include <getopt.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64
#define GPIO_INT_PIN 4
#define GPIO_LED_PIN 17
#define POLL_TIMEOUT 1000 // 1 seconde (1000 ms)

char usable_gpio_pins[] = {4,17,18,21,22,23,24,25,27,0};

static char *c_rising=(char *)"rising";
static char *c_falling=(char *)"falling";
static char *c_both=(char *)"both";
static char *c_none=(char *)"none";


int fd_gpio = 0;
int blink_pin = GPIO_LED_PIN;
int fd_blink_pin = 0;


int flush(int fd)
{
  char buf[10];

  while(read(fd,buf,sizeof(buf))>0);
}


unsigned long millis()
{
  struct timeval tv;
  gettimeofday(&tv,NULL);

  return 1000 * tv.tv_sec + tv.tv_usec/1000;
}


typedef enum { INPUT=0, OUTPUT=1 } gpioType_t;
typedef enum { FALLING=0, RISING=1, BOTH=2, NONE=3 } gpioEdge_t;
typedef enum { LOW=0, HIGH=1 } gpioValue_t;

class Gpio
{
 public:
   Gpio() { pin = -1; type = -1; io_fd=-1; };
   Gpio(int aPin, gpioType_t aType);

   int init();
   int init(int aPin, gpioType_t aType);
   int release();

   void setPin(int aPin) { pin = aPin; };
   void setType(gpioType_t aType) { type = aType; };

   int setEdge(gpioEdge_t eEdge);
   void setValue(gpioValue_t aValue);
   int getValue();
   int waitInterrupt(int timeout);

 protected:
   int _export();
   int _unexport();
   int _direction();

 private:
   int pin;
   int type;

   int io_fd;
   int edge_fd;
};


Gpio pinLed;
Gpio pinInt;


void Gpio::setValue(gpioValue_t aValue)
{
 char s_value[2];

  s_value[1]=0;

  switch(aValue)
  {
    case HIGH:
      s_value[0]='1';
      break;
    case LOW:
      s_value[0]='0';
      break;
    default:
      return;
  }

  if(write(io_fd, s_value, 2)<0)
    perror("");
}


int Gpio::release()
{
  close(io_fd);
  close(edge_fd);
  _unexport();
}


int Gpio::getValue()
{
   char buf[10];
   int ret;

   lseek(io_fd,0,0);
   ret=read(io_fd,buf,sizeof(buf));

   if(ret)
     return buf[0]-'0';
   else
     return -1;
}


Gpio::Gpio(int aPin, gpioType_t aType)
{
  pin=aPin;
  type=aType;
  io_fd=-1;
}


int Gpio::init()
{
  int ret;
  char buf[MAX_BUF];

  if(type<0 || pin <0)
    return -2;

  _export();

  ret=_direction();
  if(ret<0)
  {
    _unexport();
    return ret;
  }

  int len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", pin);
  edge_fd = open(buf, O_WRONLY);
  if (edge_fd <= 0)
  {
    _unexport();
    return edge_fd;
  }

  int option;
  switch(type)
  {
    case INPUT:
      option=O_RDONLY | O_NONBLOCK;
      break;
    case OUTPUT:
      option=O_WRONLY;
      break;
  }

  len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", pin);
  io_fd = open(buf, option);
  if(io_fd < 0)
  {
    close(edge_fd);
    _unexport();
  }
 
  return io_fd;
}


int Gpio::init(int aPin, gpioType_t aType)
{
  pin = aPin;
  type = aType;

  return init();
}


int Gpio::setEdge(gpioEdge_t eEdge)
{
 char *s_edge;

  switch(eEdge)
  {
    case RISING:
      s_edge=c_rising;
      break;
    case FALLING:
      s_edge=c_falling;
      break;
    case BOTH:
      s_edge=c_both;
      break;
    case NONE:
      s_edge=c_none;
      break;
    default:
      return -1;
  }

  int ret=write(edge_fd, s_edge, strlen(s_edge)+1);
  if(ret)
    flush(io_fd);
  return ret;
}


int Gpio::_direction()
{
  char buf[MAX_BUF];
  int ret;

  if(type<0 || pin<0)
  {
    return -2;
  } 
  int len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", pin);
  int fd = open(buf, O_WRONLY);
  if (fd <= 0)
    return fd;

  switch(type)
  {
    case INPUT:
      ret=write(fd, "in", 3);
      break;
    case OUTPUT:
      ret=write(fd, "out", 4);
      break;
    default:
      ret=-2; 
      break;
  }
  close(fd);

  return ret;
}


int Gpio::_export()
{
  char buf[MAX_BUF];

  if(pin < 0 && type < 0)
    return -2;

  int fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
  if (fd <= 0)
    return fd;

  int len = snprintf(buf, sizeof(buf), "%d", pin);
  int ret=write(fd, buf, len);
  close(fd);

  return ret;
}


int Gpio::_unexport()
{
  char buf[MAX_BUF];

  if(pin < 0 && type < 0)
    return -2;

  int fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
  if (fd <= 0)
    return fd;

  int len = snprintf(buf, sizeof(buf), "%d", pin);
  int ret=write(fd, buf, len);
  close(fd);

  return ret;
}


int Gpio::waitInterrupt(int timeout)
{
  struct pollfd fdset[2];
  int nfds = 2;
  int rc;
  char buf[MAX_BUF];

  memset((void*)fdset, 0, sizeof(fdset));
  fdset[0].fd = io_fd;
  fdset[0].events = POLLPRI;

  rc = poll(fdset, nfds, timeout);
  if (rc<0)
    return -2;

  // reception d'une interruption GPIO
  if (fdset[0].revents & POLLPRI)
  {
    lseek(fdset[0].fd,0,0);
    int len = read(fdset[0].fd, buf, MAX_BUF);
    if(len>0)
      return buf[0]-'0';
  }
  else
    return -1;
}


void clean_exit(int ret)
{
  pinLed.setValue(LOW);
  pinInt.release();
  pinLed.release();

  exit(ret);
}


static void exit_signal(int signal_number)
{
  clean_exit(0);
}


void blinkLed(int t)
{
  static unsigned long ledChrono = 0;
  static gpioValue_t led_state=LOW;
  
  unsigned long now=millis();

  if((now-ledChrono)>t)
  {
    ledChrono=now;
    if(led_state==LOW)
      led_state=HIGH;
    else
      led_state=LOW;
    pinLed.setValue(led_state); 
  }
}


void usage(char *cmd)
{
  fprintf(stderr,"\nusage : %s -i <input_pin> -c \"<shell_cmnd>\" [-o <led_output_pin>] [-d high_delay]\n",cmd);
  fprintf(stderr,"  usable pin for <input_pin> or <led_output_pin>: ");
  for(int i=0;usable_gpio_pins[i];i++)
  {
     if(i)
       fprintf(stderr,", %d",usable_gpio_pins[i]);
     else
       fprintf(stderr,"%d",usable_gpio_pins[i]);
  }
  fprintf(stderr,"\n");
}


int toPinNum(char *str)
{
  char *endStrPtr;
  long num;

  num=strtol((char *)str,&endStrPtr,10);
  if(endStrPtr[0]!=0)
    return -1;

  for(int i=0;usable_gpio_pins[i];i++)
  {
    if(usable_gpio_pins[i]==num)
      return (int)num; 
  }
  return -2;
}


int toDelay(char *str)
{
  char *endStrPtr;
  
  int delay=strtol((char *)str,&endStrPtr,10);
  if(endStrPtr[0]!=0)
    return -1;

  if(delay<0 || delay >10)
    return -2;

  delay=delay*1000;

  return delay;
}


/*
int logToFile()
{
   int fd=open("/tmp/mydebug.log", O_CREAT | O_APPEND | O_RDWR,  S_IWUSR | S_IRUSR);
   
   dup2(fd, 1);
   dup2(fd, 2);

   close(fd);
}
*/



int main(int argc, char *argv[])
{
  int ret;

  sigset_t new_mask;
  sigset_t old_mask;
  sigset_t pending_sig;

  char *command = NULL;
  int c;
  int input_pin = -1, output_pin = -1;
  int delay = 2000;

  while ((c = getopt (argc, (char **)argv, "c:i:o:d:")) != -1)
  {
    switch (c)
    {
      case 'c':
        command = (char *)malloc(strlen(optarg)+1);
        if(command)
           strcpy(command,optarg);
        else
        {
           fprintf(stderr,"ERROR : %s - internal error.\n",argv[0]);
           perror("");
           exit(1);
        }
        break;
           
      case 'i':
        input_pin = toPinNum(optarg);
        if(input_pin < 0)
        {
          fprintf(stderr,"ERROR : %s - incorrect <input_pin> (%s)\n",argv[0],optarg);
          usage(argv[0]);
          exit(1);
        }
        break;

      case 'o':
        output_pin = toPinNum(optarg);
        if(output_pin < 0)
        {
          fprintf(stderr,"ERROR : %s - incorrect <output_pin> (%s)\n",argv[0],optarg);
          usage(argv[0]);
          exit(1);
        }
        break;

      case 'd':
        delay=toDelay(optarg);
        if(delay < 0)
        {
          fprintf(stderr,"ERROR : %s - incorrect <delay> (%s), valid value : integer and 0 <= delay <= 10\n",argv[0],optarg);
          usage(argv[0]);
          exit(1);
        }
        break;

      default:
        exit(1);
        break;
     }
  }

  if(optind == (argc-1))
  {
    fprintf(stderr,"ERROR : %s - invalid parameters -- ",argv[0],optarg);
    for (int i = optind; i < argc; i++)
      fprintf (stderr,"%s ", argv[i]);
    fprintf(stderr,"\n");
    usage(argv[0]);
    exit(1);
  }

  if(!command)
  {
    fprintf(stderr,"ERROR : %s - no command\n",argv[0]);
    usage(argv[0]);
    exit(1);
  }

  if(input_pin == -1)
  {
    fprintf(stderr,"ERROR : %s - no input pin\n",argv[0]);
    usage(argv[0]);
    exit(1);
  }

  if(input_pin == output_pin)
  {
    fprintf(stderr,"ERROR : %s - <input_pin> and <output_pin> must be different\n",argv[0]);
    usage(argv[0]);
    exit(1);
  }

  sigemptyset(&new_mask);

  sigaddset(&new_mask,SIGINT);
  sigaddset(&new_mask,SIGQUIT);
  sigaddset(&new_mask,SIGTERM);
  sigaddset(&new_mask,SIGHUP);

  // maskage des signaux "d'arrêt" avant de manipuler le GPIO
  sigprocmask(SIG_SETMASK,&new_mask,&old_mask);

  signal(SIGINT,  exit_signal);
  signal(SIGQUIT, exit_signal);
  signal(SIGTERM, exit_signal);
  signal(SIGHUP,  exit_signal);

  pinLed.init(output_pin,OUTPUT);
  pinInt.init(input_pin,INPUT);

  pinInt.setEdge(BOTH); // interruption sur front montant et descendant

  // le GPIO est ouvert et opérationnel. Si des signaux sont arrivés entre temps, on s'arrête.
  if(sigpending(&pending_sig))
    exit_signal(0);

  // demaskage des signaux d'arrêt
  sigprocmask(SIG_SETMASK,&old_mask,0);

  while(1)
  {
    int val=pinInt.waitInterrupt(100);
    if(val==HIGH)
    {
       int sdelay = delay / 50;
       for(int i=0;i<sdelay;i++)
       { 
         val=pinInt.waitInterrupt(50);
         if(val==LOW)
            break;
         if(output_pin)
           blinkLed(100);
       }
       if(val==-1)
       {
          pinLed.setValue(HIGH);
          system(command);
          exit(0);
       }
    }
    if(output_pin>0)
      blinkLed(500);
  }
}
