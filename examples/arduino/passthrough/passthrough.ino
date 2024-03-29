#include <stdio.h>
#include <stdarg.h>
#include <limits.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <semphr.h>

#include <litt/opentherm/transport.h>

#include "arduino_litt_ds.h"
#include "arduino_litt_io.h"

using namespace litt;
using namespace litt::OpenTherm;

ArduinoIO* io = NULL;

static const unsigned master_in = 2, master_out = 4;
static const unsigned slave_in = 3, slave_out = 5;

static FILE uartf = {0};
static SemaphoreHandle_t log_mtx = NULL;
static StaticSemaphore_t log_mtx_state;

static int uart_putchar(char c, FILE *stream)
{
  Serial.write(c);
  return 0;
}

void vllog(const char *fmt, va_list args) {
  if (log_mtx)
    xSemaphoreTake(log_mtx, portMAX_DELAY);
  printf("[%012lu] ", micros());
  vprintf(fmt, args);
  printf("\n");
  fflush(stdout);
  if (log_mtx)
    xSemaphoreGive(log_mtx);
}

void llog(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  vllog(fmt, args);
  va_end(args);
}

class MyTransport : public Transport<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoQueue, ArduinoIO, 2>
{
public:
  MyTransport(const ArduinoPins &pins) : Transport(pins) {}
  virtual ~MyTransport() = default;

  virtual void on_opentherm_supported() {}
  virtual void on_opentherm_unsupported() {}
};

class Listener : public MyTransport
{
public:
  Listener(const ArduinoPins &pins, const char* name) :
    MyTransport(pins), name(name)
  {}

  virtual ~Listener() = default;

  virtual void start() override {}

  virtual bool process(const Frame &f) override {
    uint32_t msg = (uint32_t)f;
    char buf[32];
    sprintf(buf, "%08lx", msg);
    io.log("%s: %s", name, buf);
    return true;
  }

  virtual RequestID tx(const Frame & f, bool skip_if_busy = false,
    void (*callback)(Application*, RequestStatus, RequestID, const Frame &) = nullptr,
    Application *app = nullptr) override {}

protected:
  const char *name;
};

static Listener slave_listener({.rx=slave_in, .tx=slave_out, .owned=false, .tx_inverted = true}, "S");
static Listener master_listener({.rx=master_in, .tx=master_out, .owned=false, .tx_inverted = true}, "M");

void slave2master_isr() {
  digitalWrite(master_out, !digitalRead(slave_in));
  slave_listener.io.isr();
}

void master2slave_isr() {
  digitalWrite(slave_out, !digitalRead(master_in));
  master_listener.io.isr();
}

void slave_rx_task(void *) {
  slave_listener.rx_forever([](bool v){ digitalWrite(LED_BUILTIN, v ? HIGH : LOW); });
}

void master_rx_task(void *) {
  master_listener.rx_forever([](bool v){} );
}

static StaticTask_t slave_rx_task_buf;
static StackType_t slave_rx_task_stack[256];

static StaticTask_t master_rx_task_buf;
static StackType_t master_rx_task_stack[256];

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  log_mtx = xSemaphoreCreateMutexStatic(&log_mtx_state);
  xSemaphoreGive(log_mtx);

  pinMode(master_in, INPUT_PULLUP);
  pinMode(master_out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(master_in), master2slave_isr, CHANGE);

  pinMode(slave_in, INPUT_PULLUP);
  pinMode(slave_out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(slave_in), slave2master_isr, CHANGE);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  llog("Passthrough starting...");

  xTaskCreateStatic(slave_rx_task, "slave_rx_task", 256, NULL, 1, slave_rx_task_stack, &slave_rx_task_buf);
  xTaskCreateStatic(master_rx_task, "master_rx_task", 256, NULL, 1, master_rx_task_stack, &master_rx_task_buf);

  vTaskStartScheduler();
}

void loop() {}

extern "C" {
  void vAssertCalled(const char* file, int line) {
    llog("%s:%d: ASSERTION FAILED", file, line);
    fflush(stdout);
    // taskDISABLE_INTERRUPTS();
    // for( ;; );
  }
}
