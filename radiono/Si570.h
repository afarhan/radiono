typedef enum 
{
  SI570_ERROR = 0,
  SI570_READY,
  SI570_SUCCESS
} Si570_Status;

class Si570
{
public:
  Si570(uint8_t i2c_address, uint32_t calibration_frequency);
  Si570_Status setFrequency(uint32_t newfreq);
  void debugSi570();

  Si570_Status status;

private:
  uint8_t i2c_address;
  uint8_t dco_reg[13];
  uint32_t f_center;
  uint32_t frequency;
  uint16_t hs, n1;
  uint32_t freq_xtal;
  uint64_t fdco;
  uint64_t rfreq;

  uint8_t i2c_read(uint8_t reg_address);
  int i2c_read(uint8_t reg_address, uint8_t *output, uint8_t length);

  void i2c_write(uint8_t reg_address, uint8_t data);
  int i2c_write(uint8_t reg_address, uint8_t *data, uint8_t length);

  bool read_si570();
  void write_si570();
  void qwrite_si570();

  uint8_t getHSDIV();
  uint8_t getN1();
  uint64_t getRFREQ();

  void setRFREQ(uint32_t fnew);
  int findDivisors(uint32_t f);
};
