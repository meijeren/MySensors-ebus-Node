class CSensor
{
private:
  unsigned long m_Millis;
protected:
  const byte    m_ID;
  const char  * m_Name;
public:
  CSensor(byte a_ID, const char * a_Name;

  bool NeedsRefresh();

  void Touch();
};