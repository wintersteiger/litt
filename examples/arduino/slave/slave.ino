#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

#include "FreeRTOSConfig.h"

#include <Arduino_FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <litt/opentherm/transport.h>
#include <litt/opentherm/application.h>

#include "arduino_litt_ds.h"
#include "arduino_litt_io.h"

using namespace litt;
using namespace litt::OpenTherm;

ArduinoIO* io = NULL;

static FILE uartf = {0};
static SemaphoreHandle_t log_mtx = NULL;
static StaticSemaphore_t log_mtx_state;

static int uart_putchar(char c, FILE *stream)
{
  if (Serial.availableForWrite() > 1)
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

class MyTransport : public Slave<ArduinoTimer, ArduinoSemaphore, ArduinoTime, ArduinoQueue, ArduinoIO, 2>
{
public:
  MyTransport(const ArduinoPins &pins) : Slave(pins) {}
  virtual ~MyTransport() = default;

  virtual void on_opentherm_supported() {}
  virtual void on_opentherm_unsupported() {}
};

class SlaveApp : public SmallApplication
{
public:
  SlaveApp(const ArduinoPins &pins) :
    SmallApplication(transport),
    transport(pins)
  {
    transport.set_frame_callback(SmallApplication::sprocess, this);

    sconfig_smemberid = (uint16_t)0x0300;
    ot_version_master = 0.0f;
    ot_version_slave = 2.2f;

    randomSeed(0);
  }

  virtual ~SlaveApp() = default;

  virtual bool process(const Frame &f) {
    uint64_t now = time.get_us();
    last_frame_diff = (now - last_frame_time) / 1000;
    last_frame_time = now;
    SmallApplication::process(f);
  }

  virtual bool on_read(uint8_t data_id, uint16_t data_value = 0x0000) override {
    llog("%+05" PRId32 " Read-Data(%" PRIu8 ", %04" PRIx16 ")", last_frame_diff, data_id, data_value);
    delay(random(20, 800));
    return SmallApplication::on_read(data_id, data_value);
  }

  virtual bool on_write(uint8_t data_id, uint16_t data_value) override {
    llog("%+05" PRId32 " Write-Data(%" PRIu8 ", %04" PRIx16 ")", last_frame_diff, data_id, data_value);
    delay(random(20, 800));
    return SmallApplication::on_write(data_id, data_value);
  }

  virtual bool on_invalid_data(uint8_t data_id, uint16_t data_value) override {
    llog("%+05" PRId32 " Invalid-Data(%" PRIu8 ", %04" PRIx16 ")", last_frame_diff, data_id, data_value);
    delay(random(20, 800));
    return SmallApplication::on_invalid_data(data_id, data_value);
  }

  virtual void run() override {
    transport.rx_forever([](bool v){ digitalWrite(LED_BUILTIN, v ? HIGH : LOW); } );
  }

protected:
  MyTransport transport;
  ArduinoTime time;
  uint64_t last_frame_time;
  int32_t last_frame_diff;
};

static SlaveApp app({.rx=3, .tx=5, .owned=true, .tx_inverted = true});

static void rx_task(void *) { app.run(); }

static StaticTask_t rx_task_buf;
static StackType_t rx_task_stack[128];

void setup()
{
  Serial.begin(115200);
  // while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  log_mtx = xSemaphoreCreateMutexStatic(&log_mtx_state);
  xSemaphoreGive(log_mtx);

  fdev_setup_stream(&uartf, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartf;

  llog("Slave starting... ");

  xTaskCreateStatic(rx_task, "rx_task", 128, NULL, 1, rx_task_stack, &rx_task_buf);

  vTaskStartScheduler();
}

void loop() {}

extern "C" {
  void vAssertCalled(const char* file, int line) {
    llog("%s:%d: ASSERTION FAILED", file, line);
    fflush(stdout);
    // taskDISABLE_INTERRUPTS();
    for( ;; );
  }
}
