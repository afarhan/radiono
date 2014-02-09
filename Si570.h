typedef enum {
  SUCCESS_BIGJUMP = 0,
  SUCCESS_SMALLJUMP
} Si570_Status;

class Si570
{
public:
  Si570(char i2c_address);

  Si570_Status setFrequency(unsigned long newfreq);

  Si570_Status status;

private:
  char i2c_address;
  unsigned char dco_reg[13];
  unsigned long bitval[38];
  unsigned long f_center;
  unsigned long frequency;
  unsigned int hs, n1;
  unsigned long freq_xtal;

  char i2c_read(int reg_address);
  void i2c_write(char reg_address, char data);

  void read_si570();
  void write_si570();
  void qwrite_si570();

  void setRfreq(unsigned long fnew);
  void setDividers (unsigned long f);
  void setBitvals(void);
};
