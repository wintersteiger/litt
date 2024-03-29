#include <stdio.h>

#include <iostream>
#include <unordered_set>

#include <pqxx/pqxx>
#include <pqxx/except.hxx>

#include <opentherm/transport.h>
#include <opentherm/application.h>

using namespace OpenTherm;

class MyDevice : public DeviceBase {
public:
  MyDevice() : DeviceBase() {}
  virtual ~MyDevice() = default;

  virtual RequestID tx(const Frame &f, bool skip_if_busy = false,
                       void (*callback)(Application *, RequestStatus, RequestID,
                                        const Frame &) = nullptr,
                       Application *app = nullptr) override
  {
    return NoRequestID;
  }
};


class MyApp : public RichApplication {
public:
  MyApp() : RichApplication(device) {
    device.set_frame_callback(Application::sprocess, this);

    if (const char* cs = std::getenv("PQCONNECTION")) {
      pqc = std::make_shared<pqxx::connection>(cs);

      pqxx::work tx{*pqc};
      tx.exec1("CREATE TABLE IF NOT EXISTS passthrough (time TIMESTAMPTZ NOT NULL, msg_type bytea NOT NULL, id smallint NOT NULL, value bytea NOT NULL);"
                "SELECT create_hypertable('passthrough','time', if_not_exists => TRUE);");
      tx.commit();
    }
  }

  virtual ~MyApp() = default;

  virtual void run() override {}

  void dev_process(const Frame &f) {
    device.process(f);

    if (pqc) {
      pqxx::nontransaction ntx(*pqc);
      static char msg_type_hex[3], value_hex[5];
      snprintf(msg_type_hex, sizeof(msg_type_hex), "%02x", f.msg_type());
      snprintf(value_hex, sizeof(value_hex), "%04x", f.value());
      ntx.exec0(std::string("INSERT INTO passthrough VALUES(NOW(), '\\x") + msg_type_hex + "', " + std::to_string(f.id()) + ", '\\x" + value_hex + "');");
    }
  }

  bool outp = false;
  uint16_t master_state = 0;

  virtual void on_read(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_read(id, value);

    if (outp && id == 0) {
      if (value != master_state) {
        Application::IDMeta &meta = MyApp::idmeta[id];
        printf("%s == %s", meta.data_object, ID::to_string(meta.type, value));
        printf("\t\tch: %d dhw: %d cool: %d otc: %d ch2: %d",
          (value & 0x0100) != 0,
          (value & 0x0200) != 0,
          (value & 0x0400) != 0,
          (value & 0x0800) != 0,
          (value & 0x1000) != 0);
      }
      master_state = value;
    }
  }

  virtual void on_read_ack(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_read_ack(id, value);
    if (outp)
    {
      Application::ID *idp = find(id);
      if (!idp)
        printf("unknown data ID");
      else
      {
        Application::IDMeta &meta = MyApp::idmeta[id];
        if ((id != 0 && id != 3) || idp->value != value) {
            printf("%s == %s", meta.data_object, ID::to_string(meta.type, value));

          if (id == 0 && (idp->value & 0x00FF) != (value & 0x00FF)) {
            printf("\t\tfault: %d ch: %d dhw: %d flame: %d",
              (value & 0x01) != 0,
              (value & 0x02) != 0,
              (value & 0x04) != 0,
              (value & 0x08) != 0);
          }
        }

        idp->value = value;
      }
    }
  }

  virtual void on_write_ack(uint8_t id, uint16_t value = 0x0000) override {
    Application::on_write_ack(id, value);
    if (outp)
    {
      Application::ID *idp = find(id);
      if (!idp)
        printf("unknown data ID");
      else
      {
        Application::IDMeta &meta = MyApp::idmeta[id];
        printf("%s := %s", meta.data_object, ID::to_string(meta.type, value));
        idp->value = value;
      }
    }
  }

protected:
  MyDevice device;

  std::shared_ptr<pqxx::connection> pqc = nullptr;
};

static MyApp app;

int main(int argc, const char **argv)
{
  if (argc != 1 && argc != 2) {
    std::cout << "Usage: " << argv[0] << " [filename]" << std::endl;
    return 1;
  }

  FILE *f = stdin;
  bool filter = false;

  if (argc == 2) {
    const char *filename = argv[1];
    f = fopen(filename, "r");
    if (!f) {
      std::cout << "failure to open " << filename << std::endl;
      return 2;
    }
  }

  uint64_t prev_time = 0, time = 0;
  char c = 'X';
  uint32_t msg = 0;

  while (!feof(f)) {
    int l = fscanf(f, "[%lu] %c: %08x\n", &time, &c, &msg);
    if (l != 3) {
      std::cout << "       fscanf failed: " << l << std::endl;
      do {
        fread(&c, 1, 1, f);
      } while (c != '\n' && !feof(f));
      continue;
    }
    else {
      double delta_t = (time-prev_time) / 1e6;
      const char *dev = c == 'S' ? "T" : "B";
      Frame f(msg);

      if (filter && c == 'S' && f.id() != 0)
        continue;

      std::unordered_set<uint16_t> filter = {10, 11, 12, 13, 15, 27, 113, 114, 125, 127};
      bool outp = !filter || filter.find(f.id()) == filter.end();
      if (outp)
        printf("%6.3f %s %s \t", delta_t, dev, f.to_string());
      app.outp = outp;
      app.dev_process(f);
      if (outp)
        printf("\n");
    }

    prev_time = time;
  }

  fclose(f);

  return 0;
}
