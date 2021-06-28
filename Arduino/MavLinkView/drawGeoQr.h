#ifndef _DRAWGEOQR_H_

#define _DRAWGEOQR_H_

#include "qrcodegen.h"

//#define SERIAL_DEBUG

typedef enum
{
  MapProviderBing,
  MapProviderGoogle,
  MapProviderOsm,
  
  MapProviderCount,
} MapProvider_t;

const MapProvider_t MapProvider = MapProviderGoogle; // select map provider

const char * MapProviderString[MapProviderCount] =
{
  "https://www.bing.com/maps?sp=point.%li.%lu_%li.%lu_UAV",
  "https://maps.google.com/maps?q=%li.%lu,%li.%lu",
  "https://www.openstreetmap.org/?&mlat=%li.%lu&mlon=%li.%lu"
};

#define MaxQrVersion  8
#define QrBorder      1
#define MaxQrSize     (64 - 2 * QrBorder) // given by display height of 64 pixel

static void drawGeoQr(uint8_t drawX, uint8_t drawY, int32_t latE7, int32_t lonE7) 
{
  char s[100];
  uint8_t qrcode[qrcodegen_BUFFER_LEN_FOR_VERSION(MaxQrVersion)];
  uint8_t tempBuffer[qrcodegen_BUFFER_LEN_FOR_VERSION(MaxQrVersion)];
  unsigned char XBM_bits[MaxQrSize * ((MaxQrSize + 7) / 8)];

  snprintf(s, sizeof(s), MapProviderString[MapProvider], 
    latE7 / 10000000L, abs(latE7) % 10000000L, lonE7 / 10000000L, abs(lonE7) % 10000000L);

#ifdef SERIAL_DEBUG
  Serial.println(s);
#endif

  bool ok = qrcodegen_encodeText(s, tempBuffer, qrcode, qrcodegen_Ecc_HIGH,
    qrcodegen_VERSION_MIN, MaxQrVersion, qrcodegen_Mask_AUTO, true);
  
  int size = qrcodegen_getSize(qrcode);
  
#ifdef SERIAL_DEBUG
  Serial.print("size = ");
  Serial.println(size);
#endif
  
  if(ok && (size <= MaxQrSize))
  {
    memset(XBM_bits, 0, sizeof(XBM_bits));
    for (int y = 0; y < size; y++) 
    {
      for (int x = 0; x < size; x++) 
      {
        if(qrcodegen_getModule(qrcode, x, y))
          XBM_bits[((size + 7) / 8) * y + x / 8] |= 1 << (x % 8);
      }
    }

    u8g2.setDrawColor(1);
    u8g2.drawBox(drawX - size / 2 - QrBorder, drawY - size / 2 - QrBorder, size + 2 * QrBorder, size + 2 * QrBorder);
    u8g2.setDrawColor(0);
    u8g2.drawXBM(drawX - size / 2, drawY - size / 2, size, size, XBM_bits);
  }
}
#endif // #ifndef _DRAWGEOQR_H_
